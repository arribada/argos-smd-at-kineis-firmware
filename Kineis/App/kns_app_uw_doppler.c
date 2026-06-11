/**
 * @file    kns_app_uw_doppler.c
 * @brief   UW_DOPPLER application - Autonomous underwater/surface tracker
 *
 * @details
 * Main application for wildlife tracking tags. Monitors water conductivity via
 * a Salt Water Switch (SWS) and transmits satellite messages when the device
 * surfaces after a dive.
 *
 * State machine:
 *   BOOT -> BOOT_DEPLOY_LED -> INIT_MAC -> WAIT_MAC_READY -> MONITORING
 *                                                               |
 *                            SURFACE_TX -> WAIT_TX_DONE --------+
 *                               |
 *                       SHUTDOWN_BLINK (reed hold >10s)
 *
 * TX scheduling:
 *   - First TX is immediate upon surface detection
 *   - Subsequent TXs: T(n) = T_initial * (1 + growth/100)^n
 *   - Interval capped at T_max, count limited to max_count
 *   - Scheduling resets when device goes back underwater
 *
 * MAC profile compatibility:
 *   - BLIND (retx_nb=0): app handles scheduling, MAC sends once per request
 *   - BASIC: immediate single TX per request, same app-level scheduling
 *
 * Robustness:
 *   - IWDG watchdog (16s timeout) refreshed every loop iteration
 *   - State timeouts on all waiting states (configurable per state)
 *   - Error tracking via TAMP backup registers (survives resets)
 *   - Event logging in SRAM2 retention RAM (256 entries, survives resets)
 *   - NVM config persistence with CRC32 integrity
 *   - Reed switch shutdown cancellable by removing magnet during blink
 *
 * TX payload (UW Doppler format, MSB-first bit-packed):
 *   [0..2]    header = 0b011 (UW Doppler type indicator)
 *   [3..10]   last_known_pos = 0 (8 bits, reserved for future GPS hint)
 *   [11..17]  battery_encoded (mV - 2700) / 20, capped 0..127
 *   [18]      is_low_battery flag
 *   [19..]    zero pad up to modulation frame size
 *   LDA2/LDA2L: CRC8 (linkit-style poly 0x8380) at byte 23
 *   VLDA4(24)/LDK(128): modem-handled CRC
 *
 * Header value 0b011 is unused by linkit-v4 and reserved here for "UW Doppler"
 * (battery-only telemetry, no GPS). Self-describing across all modulations.
 *
 * @see kns_app_uw_doppler.h
 * @see mgr_sws.h
 * @see mgr_nvm.h
 */

/**
 * @addtogroup KNS_APP_UW_DOPPLER
 * @{
 */

/* UW_DOPPLER is strictly validated against the BASIC MAC profile.
 *
 * BLIND / SATDET / BLIND_POS delegate their TX scheduling to the MAC
 * stack (retx pattern, parallel messages, period). The UW_DOPPLER app
 * does its own schedule (current_interval_ms, tx_max_count, surface
 * detection) — running both layers in parallel produces conflicting
 * radio bursts AND leaves retx pulses firing after the tag has dived
 * because no path issues KNS_MAC_STOP_SEND_DATA on UW transitions.
 *
 * Lock at compile time so an accidental `make MAC_PRFL=BLIND` doesn't
 * ship a tag whose state machine is silently broken. AT+KMAC also
 * refuses runtime profile switches (see mgr_at_cmd_list_mac.c). */
#if defined(USE_MAC_PRFL_BLIND) || defined(USE_MAC_PRFL_SATDET) || \
    defined(USE_MAC_PRFL_BLIND_POS)
#  error "UW_DOPPLER is only validated with MAC_PRFL=BASIC. " \
         "Other profiles delegate TX scheduling to the MAC and conflict " \
         "with the surface-driven app scheduler."
#endif
#if !defined(USE_MAC_PRFL_BASIC)
#  error "UW_DOPPLER requires MAC_PRFL=BASIC. Check the top-level Makefile."
#endif

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "kns_app_uw_doppler.h"
#include "main.h"

/* Hardware requirements: UW_DOPPLER drives a reed switch (deploy/recovery
 * gesture), a tri-colour LED (state indication) and the PWR latch (deep LPM
 * cycling). These are present on SMD_STDALONE only; on PA / NOPA / OP the
 * board lacks the corresponding GPIO and there's no meaningful UW_DOPPLER
 * behaviour to preserve. Refuse the build loudly rather than silently
 * compile out the load-bearing logic. Checked AFTER main.h so the BSP
 * macros are available. */
#if !defined(BSP_HAS_REED_SWITCH) || !defined(BSP_HAS_LED_RGB) || \
    !defined(BSP_HAS_PWR_LATCH)
#  error "UW_DOPPLER requires a board with reed + RGB LED + PWR latch " \
         "(currently only SMD_STDALONE). Pick BOARD=SMD_STDALONE."
#endif
#include "mgr_sws.h"
#include "mgr_nvm.h"
#include "mgr_wdg.h"
#include "mgr_err.h"
#include "mgr_evtlog.h"
#include "stm32wlxx_hal.h"
#include "kns_q.h"
#include "kns_mac.h"
#include "kns_cfg.h"
#include "kineis_sw_conf.h"
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"
#include "mgr_lpm.h"
#include "lpm.h"
#include "mcu_misc.h"
#include "rtc.h"
#if defined(USE_UART_DRIVER)
#include "mgr_at_cmd.h"
#include "usart.h"
#endif

#if defined(BSP_HAS_LED_RGB)
#include "mgr_led.h"
#endif
#if defined(BSP_HAS_REED_SWITCH)
#include "mgr_reed.h"
#endif
#if defined(BSP_HAS_VBAT_ADC)
#include "mgr_bat.h"
#endif
#include "mgr_rate.h"
#include "mgr_txstats.h"
#include "mgr_pmlog.h"
#include "mgr_lpm_uw.h"
#if defined(BSP_HAS_REED_SWITCH) && defined(BSP_HAS_LED_RGB)
#include "mgr_gesture.h"
#define UW_DOPPLER_HAS_GESTURE 1
#endif

/* ---- State Machine ---- */

typedef enum {
	UW_DOPPLER_BOOT,
	UW_DOPPLER_BOOT_DEPLOY_LED,
	UW_DOPPLER_INIT_MAC,
	UW_DOPPLER_WAIT_MAC_READY,
	UW_DOPPLER_MONITORING,
	UW_DOPPLER_SURFACE_TX,
	UW_DOPPLER_WAIT_TX_DONE,
	UW_DOPPLER_SHUTDOWN_BLINK,
} UwDopplerState_t;

/* ---- Private variables ---- */

/* uw_doppler_state — NOLOAD retention, but explicitly overwritten at every
 * init via `uw_doppler_state = UW_DOPPLER_BOOT;` so the lack of a
 * self-validation check is safe. */
static __attribute__((__section__(".retentionRamNoload")))
UwDopplerState_t uw_doppler_state;

/* SWS baselines retained across ALL software-class resets (NOLOAD section
 * in SRAM2 — never wiped by Sram2_Init). Magic-checked at first read. */
#define SWS_RETAINED_MAGIC 0x53575342UL /* "SWSB" */
static __attribute__((__section__(".retentionRamNoload")))
struct {
	uint32_t magic;
	uint16_t air_baseline;
	uint16_t water_baseline;
	uint16_t observed_peak_adc;  /**< Highest ADC ever seen (dynamic cap) */
} sws_retained;

/* ---- Boot-loop protection -----------------------------------------------
 * Counts consecutive boots that never reached MONITORING. A reset between
 * boot and "MONITORING reached" is treated as a failure. Two thresholds:
 *   - BOOT_FAIL_FACTORY_RESET   : wipe NVM (defaults restored, SWS calib
 *                                 survives via sws_retained above). The tag
 *                                 keeps the same Argos ID/credentials because
 *                                 those live in the Kineis lib's own area,
 *                                 not in MGR_NVM.
 *   - BOOT_FAIL_PERMANENT_OFF   : assume hard fault, cut power via PWR_LATCH
 *                                 and wait for reed magnet to repower.
 * Stored in SRAM2 retention so it survives every reset class except
 * VBAT removal. CRC32 detects accidental corruption / first-power-on. */
#define BOOT_RETAIN_MAGIC          0x424F4F54UL  /* "BOOT" */
#define BOOT_FAIL_FACTORY_RESET    5
#define BOOT_FAIL_PERMANENT_OFF    10

static __attribute__((__section__(".retentionRamNoload")))
struct {
	uint32_t magic;
	uint16_t consecutive_failures;
	uint8_t  factory_reset_attempted;
	uint8_t  permanent_off_armed;
	uint8_t  boot_in_progress;   /**< Set at boot, cleared on MONITORING.
	                                 If a new boot sees this still set,
	                                 the previous boot failed. */
	uint8_t  _pad[3];
	uint32_t crc32;
} boot_retained;

/* Session-scoped Message Counter (Argos MC).
 *
 * Semantics: every TX inside one surface sequence ships the SAME MC. A new
 * sequence (UW→SURFACE transition, cold-boot wake at SURFACE, or the
 * tx_seq_restart_s timer firing) bumps the MC by one. The lib's auto-
 * increment after each TX still happens — we just stomp it back to
 * `s_session_mc` before the next push. The persisted NVM value at the end
 * of a sequence is therefore `s_session_mc + 1`, which is precisely the
 * value the next sequence should start at (read via KNS_CFG_getMC and
 * saved as the new `s_session_mc`). No explicit add is needed in our
 * code — the lib's increment supplies it.
 *
 * Retention NOLOAD: survives STOP1 / STOP2 / STANDBY / SHUTDOWN cold-boot
 * cycles, lost only on full VBAT removal. On power-on we re-seed from the
 * NVM-persisted MC, which preserves continuity across battery swaps.
 */
static __attribute__((__section__(".retentionRamNoload")))
struct {
	uint32_t magic;
	uint16_t session_mc;  /**< MC reused for every TX of the current sequence */
	uint8_t  initialised; /**< 1 once boot has seeded from KNS_CFG_getMC */
	uint8_t  _pad;
} mc_retained;
#define MC_RETAINED_MAGIC 0x4D4344CCUL  /* "MCD" + version 0xCC */

static uint32_t boot_retained_crc(void)
{
	/* CRC of all fields before crc32. Simple Adler-style sum is enough —
	 * we only need to catch first-boot garbage, not adversarial corruption. */
	uint32_t s = boot_retained.magic;
	s = s * 31u + boot_retained.consecutive_failures;
	s = s * 31u + boot_retained.factory_reset_attempted;
	s = s * 31u + boot_retained.permanent_off_armed;
	s = s * 31u + boot_retained.boot_in_progress;
	return s;
}

__attribute__((unused))
static bool boot_retained_valid(void)
{
	return boot_retained.magic == BOOT_RETAIN_MAGIC &&
	       boot_retained.crc32 == boot_retained_crc();
}

static void boot_retained_commit(void)
{
	boot_retained.magic = BOOT_RETAIN_MAGIC;
	boot_retained.crc32 = boot_retained_crc();
}

static bool boot_success_logged = false;  /**< Latched: ignore repeat calls. */

/* Forward decl — used by boot_loop_handle() to wipe NVM on factory reset. */
extern bool MGR_NVM_reset(void);

/**
 * @brief Boot-loop guard. Called once early in init.
 *
 * Increments consecutive_failures (boots that didn't reach MONITORING are
 * still pending so they count). Two escalations:
 *   1. ≥ BOOT_FAIL_FACTORY_RESET  →  wipe NVM and reset. Defaults will be
 *      re-applied on next boot; SWS calibration survives via sws_retained.
 *      Done ONCE only — subsequent boots keep counting up.
 *   2. ≥ BOOT_FAIL_PERMANENT_OFF →  cut PWR_LATCH (board powers off entirely
 *      until reed magnet re-arms it). Last-resort defense against a hard
 *      fault we can't software-fix.
 *
 * boot_success() resets the counter to 0 when MONITORING is first reached.
 */
/* Boot loop time-based detector.
 *
 * Mechanism: a "boot_in_progress" flag in retention RAM is SET at boot and
 * CLEARED only when MONITORING is reached (via boot_loop_mark_success()).
 * A new boot that sees the flag still set knows the previous boot died
 * before reaching MONITORING — increment the failure counter.
 *
 * This replaces the previous count-every-reset scheme, which counted
 * JLink reflashes (AIRCR.SYSRESETREQ → SFTRSTF) as failures and could
 * push the chip into permanent SHUTDOWN during normal development.
 *
 * Escalations (sealed-deployment safe):
 *   ≥ BOOT_FAIL_FACTORY_RESET (5):  wipe MGR_NVM, reset. Defaults restored
 *                                   on next boot; SWS calib preserved via
 *                                   sws_retained in SRAM2. Done ONCE.
 *   ≥ BOOT_FAIL_PERMANENT_OFF (10): SHUTDOWN with 24 h RTC auto-wake.
 *                                   The capsule may have no magnet access,
 *                                   so we never go off forever — the chip
 *                                   tries again every 24 h until either
 *                                   the fault is transient (clears) or the
 *                                   battery dies.
 *
 * On BORRSTF (cold power-on) or PINRSTF (external NRST), the counter is
 * cleared regardless — those are clearly user-initiated, not a fault.
 */
#define BOOT_PERMANENT_OFF_WAKE_S  (24u * 3600u)  /* 24 h */

static void boot_loop_handle(void)
{
	extern uint32_t g_boot_rcc_csr_raw;

	/* If the retention struct is corrupted (first power-on or VBAT loss),
	 * initialize it cleanly and exit — no fault to count. */
	if (!boot_retained_valid()) {
		boot_retained.magic = BOOT_RETAIN_MAGIC;
		boot_retained.consecutive_failures = 0;
		boot_retained.factory_reset_attempted = 0;
		boot_retained.permanent_off_armed = 0;
		boot_retained.boot_in_progress = 1;
		boot_retained_commit();
		return;
	}

	/* Cold/intentional reset — not a fault, clear counter.
	 *   BORRSTF : brown-out / true cold power-on.
	 *   PINRSTF : external NRST (debugger or operator pressing the button).
	 *   OBLRSTF : option-byte launch reset (e.g. MGR_WDG_ensureIwdgStopOptionByte
	 *             during the very first deployment boot). Without this case the
	 *             first deployment would burn one false-positive failure count
	 *             every time we touch option bytes. */
	if (g_boot_rcc_csr_raw & (RCC_CSR_BORRSTF | RCC_CSR_PINRSTF | RCC_CSR_OBLRSTF)) {
		boot_retained.consecutive_failures = 0;
		boot_retained.factory_reset_attempted = 0;
		boot_retained.permanent_off_armed = 0;
		boot_retained.boot_in_progress = 1;
		boot_retained_commit();
		return;
	}

	/* Software/IWDG/WWDG reset: only count as a failure if the previous
	 * boot did NOT reach MONITORING. The boot_in_progress flag is the
	 * tell-tale: cleared on MONITORING by boot_loop_mark_success(). */
	if (!boot_retained.boot_in_progress) {
		/* Previous boot reached MONITORING fine — the current reset
		 * happened from steady state (e.g. AT+RESET, transient fault
		 * after long uptime). Don't penalise it. */
		boot_retained.boot_in_progress = 1;
		boot_retained_commit();
		return;
	}

	/* Previous boot died before MONITORING — count it. */
	if (boot_retained.consecutive_failures < 0xFFFFu)
		boot_retained.consecutive_failures++;
	boot_retained.boot_in_progress = 1;
	boot_retained_commit();

	MGR_EVTLOG_log(EVT_BOOT_FAIL, boot_retained.consecutive_failures);
	MGR_LOG_DEBUG("[BOOT-LOOP] consecutive_failures=%u factory_done=%u\r\n",
		boot_retained.consecutive_failures,
		boot_retained.factory_reset_attempted);

	if (boot_retained.consecutive_failures >= BOOT_FAIL_PERMANENT_OFF) {
		MGR_LOG_ERR("[BOOT-LOOP] PERMANENT_OFF: SHUTDOWN with 24h auto-wake\r\n");
		MGR_EVTLOG_log(EVT_FACTORY_RESET, 0xFFFFu);
		boot_retained.permanent_off_armed = 1;
		boot_retained_commit();
#if defined(BSP_HAS_REED_SWITCH)
		MGR_REED_releasePower();
		/* RTC auto-wake every 24 h — sealed capsule never goes truly off. */
		LPM_shutdownWithAutoWake(BOOT_PERMANENT_OFF_WAKE_S);
#else
		MGR_ERR_logAndReset(ERR_BOOT_LOOP);
#endif
		return;  /* never reached */
	}

	if (boot_retained.consecutive_failures >= BOOT_FAIL_FACTORY_RESET &&
	    !boot_retained.factory_reset_attempted) {
		MGR_LOG_WARN("[BOOT-LOOP] FACTORY RESET (SWS calib preserved)\r\n");
		MGR_EVTLOG_log(EVT_FACTORY_RESET, boot_retained.consecutive_failures);
		boot_retained.factory_reset_attempted = 1;
		boot_retained_commit();
		/* Wipe app config in flash. SWS calibration survives via
		 * sws_retained (SRAM2). Kineis credentials live in FLASH_USER
		 * (MSG/WKU pages) and are not touched here. */
		(void)MGR_NVM_reset();
		MGR_ERR_logAndReset(ERR_BOOT_LOOP);
		/* never returns */
	}
}

/**
 * @brief Latch boot success when MONITORING is reached for the first time.
 *
 * Resets consecutive_failures and clears factory_reset_attempted so the next
 * fail series gets a fresh chance to escalate. Idempotent within a boot.
 */
static void boot_loop_mark_success(void)
{
	if (boot_success_logged)
		return;
	boot_success_logged = true;
	boot_retained.consecutive_failures = 0;
	boot_retained.factory_reset_attempted = 0;
	boot_retained.boot_in_progress = 0;  /* MONITORING reached: this boot is OK */
	boot_retained_commit();
	MGR_LOG_INFO("[BOOT-LOOP] Boot success latched\r\n");

	/* IWDG DISABLED on this build.
	 *
	 * Why: IWDG_STOP option byte is NOT set, so IWDG keeps counting while
	 * the MCU is in STOP. In MONITORING the LPM client requests STOP and
	 * the MAC stack may keep the chip asleep > 16s between events → IWDG
	 * fires → silent reset every ~18s.
	 *
	 * Two production-ready fixes (pick one before deployment):
	 *   1. Set IWDG_STOP option byte (FLASH_OPTR bit 17 = 1) so IWDG
	 *      freezes during STOP. Safest path, but requires an OPTR write
	 *      that has historically been risky to script on this board.
	 *   2. Arm an RTC wake-up at < 16s and refresh IWDG on every wake.
	 *
	 * Until either is in place, leave the watchdog OFF. */
	/* MGR_WDG_init();  -- intentionally disabled, see above */
}

/* Exported for fault handlers (MGR_ERR_LOG_FAULT macro in stm32wlxx_it.c) */
volatile uint32_t g_uw_doppler_state_for_err = 0;

static KNS_APP_UwDopplerTxCfg_t tx_cfg = {
	.tx_initial_interval_s = 10,
	.tx_growth_percent     = 10,
	.tx_max_interval_s     = 180,  /**< 3min cap: better Argos pass coverage for Doppler */
	.tx_max_count          = 0,    /**< unlimited */
	.tx_jitter_percent     = 10,   /**< +/-10% randomization to avoid TX collisions */
	.tx_cooldown_s         = 60,   /**< 60s quiet time between TX, survives dive/surface */
	.tx_seq_restart_s      = 0,    /**< 0 = disabled (legacy behaviour); restart sequence
	                                  *  N seconds after last TX of a capped sequence */
};

/* LB mode (low-battery) config. Defaults engage LB at 2.9V (just above the
 * existing min_tx_mV=2.8V hard floor) with 200 mV hysteresis. In LB mode TX
 * timing is slower and capped at 3 TX per surface event. */
static KNS_APP_UwDopplerLbCfg_t lb_cfg = {
	.lb_enter_mV       = 2900,
	.lb_exit_mV        = 3100,
	.lb_tx_interval_s  = 60,   /**< 6x slower than normal 10s */
	.lb_tx_max_s       = 600,  /**< 10 min cap vs normal 3 min */
	.lb_tx_max_count   = 3,
};
static bool lb_active = false; /**< Hysteretic state, updated each TX cycle. */
/* Forward decl: lb_update is defined alongside the LB getters/setters near the
 * bottom of the file, but referenced from the TX scheduling loop above. Both
 * the decl and definition are gated on BSP_HAS_VBAT_ADC since the only call
 * sites are inside that same guard. */
#if defined(BSP_HAS_VBAT_ADC)
static bool lb_update(uint16_t bat_mV);
#endif

/* Deploy mode: 1 = deployed (TX enabled), 0 = not deployed (SWS runs but no TX) */
static uint8_t deploy_mode = 1;

/* Boot window: AT command received during boot window keeps UART active */
#if defined(USE_UART_DRIVER)
static bool boot_window_at_received = false;
#endif

#define BOOT_WINDOW_MS  5000  /**< UART listen window at boot (5s) */

/* TX scheduling state */
static uint32_t tx_count = 0;              /**< Number of TXs sent in current surface event */
static uint32_t last_tx_tick = 0;          /**< Tick of last TX */
static uint32_t current_interval_ms = 0;   /**< Current interval between TXs in ms */
static bool     surface_tx_pending = false; /**< Immediate TX needed on surface detection */
/* One-shot: a fresh OPERATIONAL boot counts as the first surface event.
 * Consumed in MONITORING as soon as the SWS state is known; if the tag is
 * already at the surface a TX sequence starts without waiting for a
 * dive/resurface transition. */
static bool     boot_first_seq_pending = true;

/* Surface→TX latency instrumentation. Direct-UART trace at four
 * checkpoints — greppable on `[LAT]` for the capture campaign. Disable
 * by undefining UW_DOPPLER_LAT_TRACE in the final firmware. */
#ifndef UW_DOPPLER_LAT_TRACE
#define UW_DOPPLER_LAT_TRACE 1
#endif
#if UW_DOPPLER_LAT_TRACE
static uint32_t s_lat_t0_tick = 0u;  /**< Tick of the SURFACE detection event */
static void lat_trace(const char *tag)
{
	/* INFO-grade campaign instrumentation. Silenced once AT+LOGLVL is
	 * raised above INFO, so a production sealed unit running at WARNING
	 * doesn't waste UART time on [LAT] noise. */
	if (!MGR_LOG_passes(MGR_LOG_LVL_INFO))
		return;
	if (hlpuart1.gState == HAL_UART_STATE_RESET)
		return;
	const uint32_t now = HAL_GetTick();
	const uint32_t dt  = (s_lat_t0_tick == 0u) ? 0u : (now - s_lat_t0_tick);
	char buf[64];
	int n = snprintf(buf, sizeof(buf),
	                 "%s[LAT] %s t=%lu dt=%lu ms\r\n",
	                 MGR_LOG_levelTag(MGR_LOG_LVL_INFO),
	                 tag, (unsigned long)now, (unsigned long)dt);
	if (n > 0 && n < (int)sizeof(buf))
		(void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)buf,
		                        (uint16_t)n, 50);
}
#define LAT_MARK_T0()  do { s_lat_t0_tick = HAL_GetTick(); lat_trace("SURF"); } while (0)
#define LAT_TRACE(tag) lat_trace(tag)
#define LAT_RESET()    do { s_lat_t0_tick = 0u; } while (0)
#else
#define LAT_MARK_T0()  do {} while (0)
#define LAT_TRACE(tag) do {} while (0)
#define LAT_RESET()    do {} while (0)
#endif

/* Very-visible state-change trace. INFO-grade, distinct format with surrounding
 * separators so it's impossible to miss in a busy log. Direct-UART (bypasses
 * the ring) so the bench operator sees it the moment the transition fires. */
static void state_trace(const char *msg)
{
	if (!MGR_LOG_passes(MGR_LOG_LVL_INFO))
		return;
	if (hlpuart1.gState == HAL_UART_STATE_RESET)
		return;
	char buf[96];
	int n = snprintf(buf, sizeof(buf),
	                 "%s>>>>>>>>>> STATE: %s @ t=%lu <<<<<<<<<<\r\n",
	                 MGR_LOG_levelTag(MGR_LOG_LVL_INFO),
	                 msg, (unsigned long)HAL_GetTick());
	if (n > 0 && n < (int)sizeof(buf))
		(void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)buf,
		                        (uint16_t)n, 50);
}
#define STATE_TRACE(msg) state_trace(msg)
/* Sprint 3: anti-collision random offset added to effective_min_ms for the
 * very first TX of each surface event. Drawn fresh from the (UID-seeded)
 * PRNG at surface detection. Up to FIRST_TX_RANDOM_WINDOW_MS. */
#define FIRST_TX_RANDOM_WINDOW_MS  5000u
static uint32_t first_tx_random_offset_ms = 0;

/* Sprint 4: forced-TX counter set by KNS_APP_uw_doppler_startTestBurst().
 * Bypasses surface/deploy/rate/cooldown/backoff gates — used for AT+TEST. */
#define TEST_BURST_MAX_COUNT  10u
static uint8_t test_tx_remaining = 0;

/* ---- Event-driven LPM duty cycle for 12-month deployment ---------
 *
 * App-driven STANDBY between idle periods in MONITORING. Decides the
 * sleep duration based on the SWS state:
 *  - UNDERWATER : turtle is diving, no point checking often → long sleep.
 *  - SURFACE    : track surface session quickly → short sleep.
 *
 * Each wake = cold boot (~5 s of activity). Average current:
 *   active 5 s × 10 mA + sleep N × 2 µA ≈ (50 + N×0.002) mAh per cycle
 *   → for N=300 s (5 min surface), 0.051 mAh/cycle, ~12 cycles/h
 *      → 0.6 mAh/h × 24 × 365 = 5300 mAh/yr = 28 % of 19 Ah → 3 yr
 *   → for N=1800 s (30 min underwater), 0.054 mAh/cycle, 2 cycles/h
 *      → 0.11 mAh/h × 24 × 365 = 950 mAh/yr = 5 % of 19 Ah → 19 yr
 *
 * Gating: NEVER sleep while in CONFIG (UART session), while gesture
 * FSM is active, during MAC TX in flight, during boot stabilization,
 * or while a test burst is pending.
 *
 * Default: disabled — opt-in via AT+DUTYCFG=<uw_s>,<surf_s>,1. Once
 * validated and persisted in NVM, the sealed-deployment build can
 * ship with it on by default. */
/* duty_cfg, persistence, threshold logic and enter_standby_duty have moved
 * into the dedicated MGR_LPM_UW module (Kineis/App/Managers/MGR_LPM_UW/).
 * This file just wires the MONITORING tick into MGR_LPM_UW_tryAutoCycle. */

/* Stabilization timer + try_enter_standby_duty are owned by MGR_LPM_UW. */

/* Exponential backoff on consecutive device errors (TX_TIMEOUT / MAC_ERROR).
 * Constants defined here (rather than with the other state-machine timeouts
 * below) so the static helpers right after see them. */
#define TX_BACKOFF_BASE_MS       60000  /**< First backoff after a consecutive TX error: 1 min. Doubles each error up to TX_BACKOFF_MAX_MS. */
#define TX_BACKOFF_MAX_MS       600000  /**< Backoff cap (10 min). Prevents months-long lock-outs while still throttling a sustained RF failure mode (e.g. dead transceiver, bad antenna match). */
#define TX_BACKOFF_MAX_SHIFT         4  /**< Max left-shift on the base — 60s -> 16x = 16min, capped at TX_BACKOFF_MAX_MS = 10min anyway. */
static uint8_t  consecutive_tx_errors = 0;     /**< Reset on TX_DONE or surface event. */
static uint32_t tx_backoff_until_tick = 0;     /**< Block any TX request until this tick. */

static void tx_backoff_arm(void)
{
	uint8_t shift = consecutive_tx_errors;
	if (shift > 0) shift--;            /* first error = base, second = base*2 */
	if (shift > TX_BACKOFF_MAX_SHIFT)
		shift = TX_BACKOFF_MAX_SHIFT;
	uint32_t backoff_ms = TX_BACKOFF_BASE_MS << shift;
	if (backoff_ms > TX_BACKOFF_MAX_MS)
		backoff_ms = TX_BACKOFF_MAX_MS;
	tx_backoff_until_tick = HAL_GetTick() + backoff_ms;
	MGR_EVTLOG_log(EVT_TX_BACKOFF, (uint16_t)(backoff_ms / 1000));
	MGR_LOG_DEBUG("[UW_DPL] TX backoff armed: %lus (errors=%u)\r\n",
		(unsigned long)(backoff_ms / 1000), consecutive_tx_errors);
}

static void tx_backoff_reset(void)
{
	consecutive_tx_errors = 0;
	tx_backoff_until_tick = 0;
}

static bool tx_backoff_blocked(uint32_t *retry_in_s)
{
	if (tx_backoff_until_tick == 0)
		return false;
	uint32_t now = HAL_GetTick();
	if ((int32_t)(tx_backoff_until_tick - now) <= 0) {
		/* Expired — clear so the next blocked() call short-circuits. */
		tx_backoff_until_tick = 0;
		return false;
	}
	if (retry_in_s)
		*retry_in_s = (tx_backoff_until_tick - now) / 1000u;
	return true;
}

/* State timeout tracking */
static uint32_t state_enter_tick = 0;      /**< Tick when current state was entered */
static uint8_t  mac_init_retries = 0;      /**< MAC init retry counter */

#define TIMEOUT_BOOT_MS          10000  /**< Boot blink timeout */
#define TIMEOUT_BOOT_DEPLOY_MS   8000   /**< Deploy LED timeout (was 5000) — extra margin for MAC/RF subsystems to fully init */
#define BOOT_DEPLOY_LED_MS       5000   /**< Deploy color display duration (was 2000) — give the radio subsystem extra time to settle before MAC init */
#define TIMEOUT_MAC_READY_MS     30000  /**< Wait for MAC ready */
#define TIMEOUT_TX_DONE_MS       10000  /**< Wait for TX done — MUST be < IWDG (16s) so app times out gracefully instead of getting reset by watchdog */
#define TIMEOUT_SHUTDOWN_BLINK_MS 10000 /**< Shutdown blink timeout */
#define MAX_MAC_INIT_RETRIES     3      /**< Max MAC init retries before reset */
#define MIN_INTER_TX_INTERVAL_MS 5000   /**< Hard floor between two consecutive TX requests AND between MAC-ready and first TX — protects against MAC stack re-entry and PA back-to-back inrush */
#define PA_WDG_THRESHOLD_MS      30000  /**< Max time PA can stay ON without a TX_DONE before we force-off + reset. A normal Argos TX is < 1 s; 30 s leaves ample room for the longest BLIND retx pattern while still cutting silent 60 mA drain quickly enough to spare battery. */
#define MAX_STATE_HANG_MS       300000  /**< 5 min cap on any non-steady state. MONITORING is excluded (steady state). Anything longer means the SM is wedged — log + reset rather than silently hang. */
/* TX_BACKOFF_* macros are defined earlier in the file so the static helpers
 * (tx_backoff_arm etc.) can see them. Kept the originals below as docs:
 * #define TX_BACKOFF_BASE_MS  60000
 * #define TX_BACKOFF_MAX_MS  600000
 * #define TX_BACKOFF_MAX_SHIFT     4 */

/* Direct synchronous UART log macro for tracing crash-prone code paths.
 * Writes a fixed-tag line straight to the LPUART without going through the
 * MGR_LOG ring buffer, so the message physically reaches the host even if
 * the device freezes or power-cycles immediately after.
 * Tag should be a short literal (≤32 chars) to keep the UART burst short. */
#if defined(USE_UART_DRIVER)
#define UWDPL_TRACE(tag) do { \
	if (!MGR_LOG_passes(MGR_LOG_LVL_TRACE)) break; \
	static const char _trace_msg[] = "[T] [TRACE] " tag " t=%lu\r\n"; \
	static char _trace_buf[64]; \
	int _trace_n = snprintf(_trace_buf, sizeof(_trace_buf), \
		_trace_msg, (unsigned long)HAL_GetTick()); \
	if (_trace_n > 0 && hlpuart1.gState != HAL_UART_STATE_RESET) \
		HAL_UART_Transmit(&hlpuart1, (uint8_t *)_trace_buf, \
			(uint16_t)_trace_n, 100); \
} while (0)
#else
#define UWDPL_TRACE(tag) do {} while (0)
#endif

/* Reed switch shutdown tracking — used by the raw-reed fallback path. */
#if defined(BSP_HAS_REED_SWITCH)
#define REED_SHUTDOWN_HOLD_MS  10000
__attribute__((unused)) static uint32_t magnet_on_tick = 0;
__attribute__((unused)) static bool     shutdown_triggered = false;
#endif

/* Battery monitoring */
#if defined(BSP_HAS_VBAT_ADC)
static uint16_t last_vbat_mV = 0;          /**< Last battery voltage reading */
#endif

/* TCXO pre-warmup: started on UW->SURFACE so 1st TX skips warmup wait */
static bool     tcxo_first_tx_skip = false;   /**< Apply 0ms warmup for next TX */
static uint32_t tcxo_warmup_saved_ms = 0;     /**< Original warmup, restored after 1st TX */

/* Periodic SWS calibration save to flash (debounced) */
#define NVM_CALIB_SAVE_MIN_INTERVAL_S  300  /**< Min 5 min between flash writes */

/* Simple LCG for jitter (no need for cryptographic randomness) */
static uint32_t prng_state = 0xA5A5A5A5UL;
static uint32_t prng_next(void)
{
	/* Numerical Recipes LCG */
	prng_state = prng_state * 1664525UL + 1013904223UL;
	return prng_state;
}

/* Direct-UART trace: prints [ST] state changes and [LPM] entry/exit
 * boundaries. Bypasses the MGR_LOG ring buffer so the last line on the
 * wire tells us exactly where the chip died on crash.
 *
 * Defaults to ON in DEBUG builds, OFF in production — matches the user's
 * "OPERATIONAL silent unless DEBUG" sealed-deployment rule. Can be forced
 * either way via -DUW_DOPPLER_VERBOSE_TRACE=0/1. */
#ifndef UW_DOPPLER_VERBOSE_TRACE
#  ifdef DEBUG
#    define UW_DOPPLER_VERBOSE_TRACE 1
#  else
#    define UW_DOPPLER_VERBOSE_TRACE 0
#  endif
#endif
static void uw_trace3(const char *fmt, uint32_t a, uint32_t b, uint32_t c)
{
#if defined(USE_UART_DRIVER) && (UW_DOPPLER_VERBOSE_TRACE)
	if (!MGR_LOG_passes(MGR_LOG_LVL_TRACE))
		return;
	if (hlpuart1.gState == HAL_UART_STATE_RESET)
		return;
	static char buf[96];
	int hdr_n = snprintf(buf, sizeof(buf), "%s",
		MGR_LOG_levelTag(MGR_LOG_LVL_TRACE));
	if (hdr_n < 0) hdr_n = 0;
	int n = snprintf(buf + hdr_n, sizeof(buf) - hdr_n, fmt,
		(unsigned long)a, (unsigned long)b, (unsigned long)c);
	if (n > 0)
		HAL_UART_Transmit(&hlpuart1, (uint8_t *)buf,
		                  (uint16_t)(hdr_n + n), 50);
#else
	(void)fmt; (void)a; (void)b; (void)c;
#endif
}

/* LPM client request for the UW_DOPPLER application.
 *
 * Cap LPM to NONE while the boot state machine has not reached MONITORING.
 * Reason: in STOP the SysTick is frozen, so the BOOT / BOOT_DEPLOY_LED /
 * WAIT_MAC_READY timeouts (HAL_GetTick-based) never elapse — the chip would
 * stay asleep waiting for an event that never fires until MAC pushes one.
 *
 * Once MONITORING is reached the MAC stack manages its own wakeups via RTC
 * and the chip can drop to STOP safely. Tradeoff: ~5–10 s of busy-loop
 * during boot at higher current, then back to deep sleep in MONITORING. */
static enum MgrLpm_LPM_t uw_doppler_lpmReq(void)
{
	if (uw_doppler_state < UW_DOPPLER_MONITORING)
		return LOW_POWER_MODE_NONE;

	/* MONITORING: allow SLEEP.
	 *
	 * SLEEP mode (Cortex-M WFI with MAIN regulator on) wakes on EVERY
	 * interrupt — SysTick (1 kHz), reed EXTI, LPUART1 RX EXTI — and the
	 * HAL tick keeps incrementing through it. Safe to enable without any
	 * RTC wake-timer setup: SysTick alone bounds the sleep duration to
	 * 1 ms which is well below all our timing constraints (gesture 100 ms
	 * blink, SWS 1 s sample, etc.).
	 *
	 * Why not STOP2 yet? Memory note `lpm_stop_known_issue.md`: the
	 * Kineis aggregator's STOP path has a documented "wake works once,
	 * fails on subsequent boots" symptom. Fixing it safely needs an HW
	 * debug session — until then SLEEP gives ~75 % current reduction
	 * (10 mA → ~2.5 mA on this MAC profile) which extends the 19 Ah
	 * budget from 79 days to ~10 months.
	 *
	 * Aggregator picks the SHALLOWEST request (lowest enum). So if we
	 * say SLEEP and the Kineis MAC client says STOP, the system goes
	 * SLEEP — we never accidentally inherit a broken deeper mode.
	 *
	 * 2026-06-07 SLEEP investigation:
	 *   - First attempt failed: TIM16 sleep-clock gated → MAC corrupted.
	 *   - lpm.c now sleep-enables TIM16/SUBGHZ/GPIO/DMA/FLASH/SRAM1/2.
	 *   - Second attempt: SysTick effectively runs at ~25 % of wall time
	 *     (tick advances 7 s in 30 s wall). State machine appears slow
	 *     but functional. Cause: unknown — likely HSI/SYSCLK frequency
	 *     scaling on SLEEP exit, or LPM_SystemClockConfig setting that
	 *     reduces CPU clock when MAIN regulator is on but we're WFI.
	 *
	 * Long-run test confirmed: SLEEP causes IWDG reset loop on this
	 * config (state bounces 0↔1, tick periodically resets to ~1273 ms).
	 * Root cause needs HW scope: either PLL drops in SLEEP and CPU
	 * never executes main loop fast enough to refresh IWDG, or there's
	 * a peripheral race we haven't identified.
	 *
	 * 12-month battery target REMAINS blocked. Realistic paths forward
	 * (need HW lab session, not autonomous):
	 *   1. Scope HSE/PLL during SLEEP transition — identify clock drop.
	 *   2. Try STOP2 with thorough RTC WUTF clearing + LPTIM1 wake.
	 *   3. STANDBY-cycling: full power-down between TX cycles, RTC wake
	 *      to cold boot, re-init MAC each wake. ~2 µA between cycles.
	 *      Architecture change but proven multi-year on similar tags. */
	return LOW_POWER_MODE_NONE;
	/* return LOW_POWER_MODE_SLEEP; */
}

static bool uw_doppler_lpmNotifEnter(__attribute__((unused)) enum MgrLpm_LPM_t lpm)
{
	/* Side effects (LED off, SWS analog power down, RTC arm) only make
	 * sense for DEEP sleep modes that genuinely halt clocks and powered
	 * domains for a meaningful duration. Skip them for:
	 *   - NONE  : MGR_LPM_enter returns immediately, no sleep at all.
	 *   - SLEEP : Cortex WFI with MAIN reg on, SysTick wakes us every 1 ms
	 *             → would thrash the LED / SWS analog rail at 1 kHz.
	 *
	 * Reed EXTI and LPUART1 RX interrupts wake SLEEP directly so we lose
	 * nothing by skipping the teardown for it. STOP/STANDBY/SHUTDOWN still
	 * run the full sequence below. */
	if (lpm == LOW_POWER_MODE_NONE || lpm == LOW_POWER_MODE_SLEEP)
		return true;

	uw_trace3("[LPM] enter mode=%lu st=%lu t=%lu\r\n",
		(uint32_t)lpm, (uint32_t)uw_doppler_state, HAL_GetTick());

	/* Arm a 1 Hz RTC wake-up so the chip exits STOP every ~1 s to poll
	 * SWS / gesture / MAC. Safe in BASIC profile: the only MAC consumers
	 * of MCU_TIM_HDLR_TX_PERIOD (kns_mac_prfl_blind.c, aks_l1.c) are not
	 * linked in this build. WL55 HAL: CK_SPRE 16-bit, counter=0 → 1 s.
	 *
	 * Critical: clear PWR_SR1.WUFI (sticky from previous wake). The MGR_LPM
	 * STOP path doesn't clear it (only STANDBY/SHUTDOWN do — mgr_lpm.c:287-289,
	 * lpm.c:243). Leaving it set makes WFI return immediately on subsequent
	 * STOP entries, so only the first wake works. */
	if (lpm == LOW_POWER_MODE_STOP) {
		/* Disable + clear stale RTC + WUFI state, re-enable internal
		 * wake-up line, arm fresh 1 Hz timer. The full sequence is
		 * needed because the MGR_LPM STOP path doesn't call
		 * LPM_configWakeUpRtc() (only STANDBY/SHUTDOWN do — lpm.c:459). */
		HAL_PWREx_DisableInternalWakeUpLine();
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
		HAL_PWREx_EnableInternalWakeUpLine();
		(void)HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0,
			RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
	}

#if defined(BSP_HAS_LED_RGB)
	MGR_LED_off();
#endif
	MGR_SWS_enterLowPower();
	return true;
}

static bool uw_doppler_lpmNotifExit(__attribute__((unused)) enum MgrLpm_LPM_t lpm)
{
	/* Mirror lpmNotifEnter: NONE + SLEEP don't run the heavy
	 * SWS-analog-rail / RTC-disarm sequence, so don't run the inverse
	 * on exit either. */
	if (lpm == LOW_POWER_MODE_NONE || lpm == LOW_POWER_MODE_SLEEP)
		return true;
	MGR_SWS_exitLowPower();
	/* Disarm our wake-up so MAC can safely reuse the same timer for its own
	 * scheduling on its next request. Costs one register write per wake. */
	if (lpm == LOW_POWER_MODE_STOP)
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	uw_trace3("[LPM] exit mode=%lu st=%lu t=%lu\r\n",
		(uint32_t)lpm, (uint32_t)uw_doppler_state, HAL_GetTick());
	return true;
}

static struct MgrLpmClientCb_t uw_doppler_lpm_client = {
	.fpMGR_LPM_LpmReqCb        = uw_doppler_lpmReq,
	.fpMGR_LPM_LpmNotifEnterCb = uw_doppler_lpmNotifEnter,
	.fpMGR_LPM_LpmNotifExitCb  = uw_doppler_lpmNotifExit,
};

/* Public entry points routed through MGR_LPM_UW for the AT command layer. */
void KNS_APP_uw_doppler_enterStandbyTest(uint32_t seconds)
{
	MGR_LPM_UW_enterStandbyTimed(seconds);
}

void KNS_APP_uw_doppler_setDutyCfg(uint16_t uw_s, uint16_t surf_s, uint8_t enabled)
{
	MGR_LPM_UW_setDutyCfg(uw_s, surf_s, enabled);
}

void KNS_APP_uw_doppler_getDutyCfg(uint16_t *uw_s, uint16_t *surf_s, uint8_t *enabled)
{
	MGR_LPM_UW_getDutyCfg(uw_s, surf_s, enabled);
}

/* ---- State transition helper ---- */

static void transition_to(UwDopplerState_t new_state)
{
	uw_trace3("[ST] %lu->%lu t=%lu\r\n",
		(uint32_t)uw_doppler_state, (uint32_t)new_state, HAL_GetTick());
	uw_doppler_state = new_state;
	state_enter_tick = HAL_GetTick();
	g_uw_doppler_state_for_err = (uint32_t)new_state;
	MGR_EVTLOG_log(EVT_STATE_CHANGE, (uint16_t)new_state);

	/* First time we reach steady-state operation — disarm the boot-loop
	 * guard. Putting the hook in transition_to() (rather than the loop)
	 * guarantees one call per real transition, not per loop tick. */
	if (new_state == UW_DOPPLER_MONITORING) {
		boot_loop_mark_success();
		MGR_LPM_UW_markMonitoringEntered();
	}
}

static uint32_t state_elapsed_ms(void)
{
	return HAL_GetTick() - state_enter_tick;
}

/* ---- Shutdown ---- */

#if defined(BSP_HAS_REED_SWITCH)
/**
 * @brief Power off the board (gesture / end-of-mission path)
 *
 * Saves config to NVM then hands over to MGR_LPM_UW_enterShutdownReed:
 * PWR_LATCH LOW (true power-off on battery, magnet re-latch = POR), with
 * a STOP2 + reed-EXTI fallback when VDD is externally maintained (bench).
 * Never returns; next boot is always OPERATIONAL.
 */
static void enter_shutdown(void)
{
	MGR_LOG_INFO("[UW_DPL] Entering POWER_OFF...\r\n");
	MGR_EVTLOG_log(EVT_SHUTDOWN, 0);
	MGR_WDG_refresh();  /* Refresh before NVM save (flash write takes time) */
	MGR_NVM_save();

	MGR_LPM_UW_enterShutdownReed();
}
#endif

/* ---- Helpers ---- */

/**
 * @brief Milliseconds until the NEXT scheduled action — deadline-based
 *        sleep source for the LPM scheduler.
 *
 * The scheduler sleeps exactly until the nearest pending deadline instead
 * of a fixed period. Candidates:
 *  - next SWS sample (dive/surface detection cadence),
 *  - next TX of the running sequence (last_tx_tick + current_interval_ms),
 *  - sequence-restart deadline once the TX cap is reached.
 * Returns 0 when an action is due now (caller skips the sleep and lets the
 * state machine run). Requires the STOP-mode tick compensation: all these
 * timers are HAL_GetTick-based and must count wall-clock time.
 */
static uint32_t uw_ms_until_next_action(void)
{
	uint32_t delta = MGR_SWS_msUntilNextSample();
	const uint8_t cap = lb_active ? lb_cfg.lb_tx_max_count
	                              : tx_cfg.tx_max_count;
	const uint32_t since = HAL_GetTick() - last_tx_tick;

	/* Next TX of an in-flight sequence (cap not reached yet). */
	if (tx_count > 0 && current_interval_ms > 0 &&
	    (cap == 0u || tx_count < cap)) {
		uint32_t r = (since >= current_interval_ms)
			? 0u : (current_interval_ms - since);
		if (r < delta)
			delta = r;
	}

	/* Sequence-restart deadline (cap reached, timer armed). */
	if (tx_cfg.tx_seq_restart_s > 0u && cap > 0u && tx_count >= cap &&
	    last_tx_tick > 0u) {
		const uint32_t rst_ms = (uint32_t)tx_cfg.tx_seq_restart_s * 1000u;
		uint32_t r = (since >= rst_ms) ? 0u : (rst_ms - since);
		if (r < delta)
			delta = r;
	}

	return delta;
}

static uint32_t compute_next_interval_ms(uint32_t n)
{
	/* T_n = T_initial * (1 + growth/100)^n
	 * Computed iteratively to avoid floating point.
	 * Each step: interval = interval * (100 + growth) / 100
	 *
	 * LB mode override: when low-battery mode is active, swap the initial/max
	 * for the lb_* variants (no growth applied — just fixed slower cadence).
	 */
	uint32_t interval_ms = lb_active
		? (uint32_t)lb_cfg.lb_tx_interval_s * 1000
		: (uint32_t)tx_cfg.tx_initial_interval_s * 1000;
	uint32_t max_ms = lb_active
		? (uint32_t)lb_cfg.lb_tx_max_s * 1000
		: (uint32_t)tx_cfg.tx_max_interval_s * 1000;
	for (uint32_t i = 0; i < n; i++) {
		uint64_t next = (uint64_t)interval_ms * (100 + tx_cfg.tx_growth_percent) / 100;
		if (next >= max_ms) {
			interval_ms = max_ms;
			break;
		}
		interval_ms = (uint32_t)next;
	}

	/* Apply jitter: random +/- jitter_percent of base interval.
	 * Avoids RF collisions when multiple tags surface together.
	 */
	if (tx_cfg.tx_jitter_percent > 0 && interval_ms > 0) {
		uint32_t span_ms = (interval_ms * tx_cfg.tx_jitter_percent) / 100;
		if (span_ms > 0) {
			/* Map prng to [-span_ms, +span_ms] */
			uint32_t r = prng_next();
			int32_t delta = (int32_t)(r % (2 * span_ms + 1)) - (int32_t)span_ms;
			int64_t jittered = (int64_t)interval_ms + delta;
			if (jittered < 1000) jittered = 1000;  /* min 1s safety */
			interval_ms = (uint32_t)jittered;
		}
	}
	return interval_ms;
}

static void reset_tx_scheduling(void)
{
	tx_count = 0;
	/* IMPORTANT: don't reset last_tx_tick. Preserving it makes the inter-TX
	 * minimum interval check honor the configured tx_initial_interval_s even
	 * across dive/surface cycles — a re-surface within X seconds after the
	 * previous TX won't trigger a new one until X has elapsed. */
	current_interval_ms = 0;
	surface_tx_pending = false;
}

/* ---- Linkit-v4 compatible Argos packet helpers ---- */

/* Battery encoding (Argos/linkit-v4): (mV - 2700) / 20, clamped 0..127 (7 bits).
 * Reference: linkit-v4-core/core/services/argos_packet_builder.cpp:38
 */
#define ARGOS_BATT_REF_MV    2700U
#define ARGOS_BATT_MV_PER_UNIT  20U
#define ARGOS_BATT_MAX_VAL    127U

/* Low-battery margin above hard inhibit threshold. The flag warns receivers
 * that the device is approaching cutoff so they can deprioritize the fix.
 */
#define LOW_BATT_MARGIN_MV    200U

/* UW Doppler packet header — 3-bit type identifier so receivers can demux
 * this packet from linkit's Short(000)/Sensor(001)/Fastloc(010)/RSPB(100-110)/
 * CloudLocate(111) types. Value 0b011 is unused by linkit and reserved here
 * for "UW Doppler" (battery-only, no GPS).
 */
#define UW_DOPPLER_PACKET_HEADER  0b011U

/* LDA2 frame: data ends at bit 184, CRC8 at byte 23 (bits 184..191).
 * Reference: linkit-v4-core/core/services/argos_packet_builder.hpp
 */
#define LDA2_FRAME_BYTES   24U
#define LDA2_DATA_BITS    184U

static uint8_t argos_convert_battery_voltage(uint16_t mV)
{
	if (mV <= ARGOS_BATT_REF_MV)
		return 0;
	uint32_t v = ((uint32_t)mV - ARGOS_BATT_REF_MV) / ARGOS_BATT_MV_PER_UNIT;
	if (v > ARGOS_BATT_MAX_VAL) v = ARGOS_BATT_MAX_VAL;
	return (uint8_t)v;
}

/* Pack `nbits` bits of `value` into `buf` at bit offset `*bit_pos`,
 * MSB-first big-endian (same semantics as linkit's PACK_BITS macro).
 * Updates *bit_pos by nbits.
 */
static void argos_pack_bits(uint8_t *buf, uint32_t *bit_pos,
                            uint32_t value, uint8_t nbits)
{
	for (int i = (int)nbits - 1; i >= 0; i--) {
		uint8_t bit = (uint8_t)((value >> i) & 1U);
		uint32_t byte_idx = *bit_pos >> 3;
		uint8_t  bit_in_byte = 7U - (uint8_t)(*bit_pos & 7U);
		buf[byte_idx] = (uint8_t)((buf[byte_idx] & ~(1U << bit_in_byte)) |
		                          (bit << bit_in_byte));
		(*bit_pos)++;
	}
}

/* CRC8 used by Argos LDA2 frames (linkit-v4 core/util/crc8.hpp).
 * Polynomial 0x8380 in 16-bit accumulator (NOT the standard 0x07 CRC8-CCITT
 * used by the SPI protocol layer). Final CRC = (acc >> 8) & 0xFF.
 *
 * Computes CRC over `nbits` bits of `data`, prepending zero pad to byte-align.
 * Result is the CRC8 byte to be stored at byte 23 of the LDA2 frame.
 */
static uint8_t argos_crc8(const uint8_t *data, uint32_t nbits)
{
	/* Linkit's algorithm zero-pads at the START so total bit count becomes a
	 * multiple of 8. We replicate by computing remainder up front, then
	 * iterating bytes from the padded start.
	 */
	uint16_t crc = 0;
	uint32_t remainder = (8U - (nbits % 8U)) % 8U;
	uint32_t total_bits = nbits + remainder;
	uint32_t total_bytes = total_bits / 8U;

	/* Re-pack data with leading zero-pad of `remainder` bits */
	uint8_t buffer[LDA2_FRAME_BYTES] = {0};
	uint32_t op_pos = remainder;
	uint32_t ip_pos = 0;
	uint32_t left = nbits;
	while (left) {
		uint32_t bits = (left > 8U) ? 8U : left;
		uint8_t byte = 0;
		/* Extract `bits` from data starting at ip_pos (MSB-first) */
		for (uint32_t k = 0; k < bits; k++) {
			uint32_t b_idx = (ip_pos + k) >> 3;
			uint8_t  b_off = 7U - (uint8_t)((ip_pos + k) & 7U);
			byte = (uint8_t)((byte << 1) | ((data[b_idx] >> b_off) & 1U));
		}
		/* Pack `bits` into buffer at op_pos (MSB-first) */
		for (int k = (int)bits - 1; k >= 0; k--) {
			uint32_t b_idx = op_pos >> 3;
			uint8_t  b_off = 7U - (uint8_t)(op_pos & 7U);
			uint8_t bit = (uint8_t)((byte >> k) & 1U);
			buffer[b_idx] = (uint8_t)((buffer[b_idx] & ~(1U << b_off)) | (bit << b_off));
			op_pos++;
		}
		ip_pos += bits;
		left -= bits;
	}

	for (uint32_t idx = 0; idx < total_bytes; idx++) {
		crc ^= (uint16_t)((uint16_t)buffer[idx] << 8);
		for (int i = 0; i < 8; i++) {
			if (crc & 0x8000U)
				crc ^= 0x8380U;
			crc = (uint16_t)(crc << 1);
		}
	}
	return (uint8_t)(crc >> 8);
}

/* Build the UW Doppler payload — battery telemetry only, with explicit
 * 3-bit type header so receivers can identify it on any modulation.
 *
 * Bit layout (MSB-first):
 *   [0..2]    header = 0b011  (UW Doppler type)
 *   [3..10]   last_known_pos = 0  (8 bits, reserved for future GPS hint)
 *   [11..17]  battery_encoded     (7 bits, (mV-2700)/20 capped to 127)
 *   [18]      is_low_battery flag (1 bit)
 *   [19..]    zero pad
 *   For LDA2/LDA2L: CRC8 at byte 23 (computed over first 184 bits).
 *
 * Frame sizes per modulation:
 *   VLDA4 → 24 bits  (3 bytes), modem handles CRC
 *   LDK   → 128 bits (16 bytes), modem handles CRC
 *   LDA2  → 192 bits (24 bytes), firmware embeds CRC8 at byte 23
 *   LDA2L → 196 bits (24.5 bytes), firmware embeds CRC8 at byte 23
 */
static bool build_tx_payload(struct KNS_MAC_appEvt_t *appEvt)
{
	struct KNS_CFG_radio_t device_radio_cfg;
	if (KNS_CFG_getRadioInfo(&device_radio_cfg) != KNS_STATUS_OK) {
		MGR_LOG_ERR("[UW_DPL] Radio config read failed, skipping TX\r\n");
		return false;
	}

	appEvt->id = KNS_MAC_SEND_DATA;
	appEvt->data_ctxt.sf = KNS_SF_NO_SERVICE;

	/* Zero the entire payload buffer (zero pad is the linkit convention) */
	memset(appEvt->data_ctxt.usrdata, 0, KNS_MAC_USRDATA_MAXLEN);

	/* Resolve battery + low-battery flag */
	uint16_t batt_mV = 0;
	bool is_low_battery = false;
#if defined(BSP_HAS_VBAT_ADC)
	batt_mV = last_vbat_mV;
	uint16_t min_tx_mV = MGR_BAT_getMinTxVoltage_mV();
	if (min_tx_mV > 0)
		is_low_battery = (batt_mV < (uint16_t)(min_tx_mV + LOW_BATT_MARGIN_MV));
#endif

	uint8_t batt_encoded = argos_convert_battery_voltage(batt_mV);

	/* Pack UW Doppler layout (19 useful bits, MSB-first):
	 *   header(3) + last_known_pos(8) + batt(7) + low_batt(1) = 19 bits
	 */
	uint32_t bit_pos = 0;
	const uint32_t last_known_pos = 0;  /* placeholder, no GPS */
	argos_pack_bits(appEvt->data_ctxt.usrdata, &bit_pos, UW_DOPPLER_PACKET_HEADER, 3);
	argos_pack_bits(appEvt->data_ctxt.usrdata, &bit_pos, last_known_pos, 8);
	argos_pack_bits(appEvt->data_ctxt.usrdata, &bit_pos, batt_encoded, 7);
	argos_pack_bits(appEvt->data_ctxt.usrdata, &bit_pos, is_low_battery ? 1U : 0U, 1);

	/* Set bitlen based on modulation. LDA2/LDA2L need a CRC8 byte at the end
	 * because the modem doesn't add one for LDA2; LDK/VLDA4 modem handles CRC.
	 */
	bool needs_lda2_crc = false;
	switch (device_radio_cfg.modulation) {
	case KNS_TX_MOD_LDA2:
		appEvt->data_ctxt.usrdata_bitlen = 192;
		needs_lda2_crc = true;
		break;
	case KNS_TX_MOD_LDA2L:
		appEvt->data_ctxt.usrdata_bitlen = 196;
		needs_lda2_crc = true;
		break;
	case KNS_TX_MOD_VLDA4:
		/* 24-bit frame: 19 useful + 5 zero pad. Modem handles CRC. */
		appEvt->data_ctxt.usrdata_bitlen = 24;
		break;
	case KNS_TX_MOD_LDK:
		/* 128-bit frame: 19 useful + 109 zero pad. Modem handles CRC. */
		appEvt->data_ctxt.usrdata_bitlen = 128;
		break;
	default:
		MGR_LOG_WARN("[UW_DPL] Unknown modulation %d, fallback to LDA2\r\n",
			device_radio_cfg.modulation);
		appEvt->data_ctxt.usrdata_bitlen = 192;
		needs_lda2_crc = true;
		break;
	}

	/* LDA2/LDA2L: compute CRC8 over the first 184 bits, store at byte 23 */
	if (needs_lda2_crc) {
		uint8_t crc = argos_crc8(appEvt->data_ctxt.usrdata, LDA2_DATA_BITS);
		appEvt->data_ctxt.usrdata[LDA2_FRAME_BYTES - 1] = crc;
	}

	MGR_LOG_DEBUG("[UW_DPL] TX payload: hdr=0x%X batt=%umV(enc=%u) low=%u "
		"mod=%d bits=%u data=%02X%02X%02X...%02X\r\n",
		UW_DOPPLER_PACKET_HEADER, batt_mV, batt_encoded,
		is_low_battery ? 1 : 0, device_radio_cfg.modulation,
		appEvt->data_ctxt.usrdata_bitlen,
		appEvt->data_ctxt.usrdata[0], appEvt->data_ctxt.usrdata[1],
		appEvt->data_ctxt.usrdata[2],
		needs_lda2_crc ? appEvt->data_ctxt.usrdata[LDA2_FRAME_BYTES - 1] : 0);

	return true;
}

static void start_mac_profile(void)
{
	enum KNS_status_t status;
	struct KNS_MAC_appEvt_t appEvt;

	appEvt.id = KNS_MAC_INIT;
	/* UW_DOPPLER is locked to BASIC at compile time — see the guard at
	 * the top of this file. The other profile branches were removed
	 * intentionally; do NOT re-introduce a USE_MAC_PRFL_BLIND fallback
	 * without first fixing the app-side / MAC-side TX scheduling clash. */
	appEvt.init_prfl_ctxt.id = KNS_MAC_PRFL_BASIC;
	MGR_LOG_INFO("[UW_DPL] MAC profile: BASIC\r\n");

	status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
	if (status != KNS_STATUS_OK) {
		MGR_LOG_ERR("[UW_DPL] MAC init push failed: 0x%x\r\n", status);
		/* Don't assert — let timeout handle retry */
	}
}

static bool process_mac_events(void)
{
	enum KNS_status_t status;
	struct KNS_MAC_srvcEvt_t srvcEvt;
	bool got_event = false;

	for (status = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt);
	     status != KNS_STATUS_QEMPTY;
	     status = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt)) {
		if (status != KNS_STATUS_OK)
			continue;

		got_event = true;

		switch (srvcEvt.id) {
		case KNS_MAC_OK:
			if (uw_doppler_state == UW_DOPPLER_WAIT_MAC_READY) {
				MGR_LOG_INFO("[UW_DPL] MAC ready\r\n");
				MGR_EVTLOG_log(EVT_MAC_READY, 0);
				mac_init_retries = 0;
				/* Set last_tx_tick so the MIN_INTER_TX_INTERVAL_MS check
				 * makes the first TX wait POST_MAC_READY_SETTLE_MS after
				 * MAC ready — gives radio/TCXO time to fully come up. */
				last_tx_tick = HAL_GetTick();
				transition_to(UW_DOPPLER_MONITORING);
			} else if (srvcEvt.app_evt == KNS_MAC_SEND_DATA) {
				MGR_LOG_INFO("[UW_DPL] TX accepted\r\n");
			}
			break;

		case KNS_MAC_TX_DONE:
			LAT_TRACE("TX_DONE");  /* T3: MAC reports frame on the wire */
			LAT_RESET();           /* arm for the next surface event */
			MGR_LOG_INFO("[UW_DPL] TX done (#%lu)\r\n", tx_count);
			MGR_EVTLOG_log(EVT_TX_DONE, 0);
			MGR_TXSTATS_recordDone();  /* Sprint 4 */
			/* Sprint 4: consume one test-burst slot if any. The next
			 * MONITORING loop iteration will fire the following TX. */
			if (test_tx_remaining > 0)
				test_tx_remaining--;
			/* Rate limiter: count successful TX only. TX_TIMEOUT/TX_ERROR
			 * are NOT counted — they don't consume air-time / battery the
			 * same way, and the backoff path handles their throttling. */
			MGR_RATE_recordTx();
			/* Successful TX → clear the consecutive-error counter so the
			 * next failure starts fresh at the base backoff. */
			tx_backoff_reset();
			/* CRITICAL: turn off external PA — MAC stack enables it via
			 * MCU_MISC_turn_on_pa() at TX start but does NOT auto-disable it.
			 * If we don't disable it here, the PA stays drawing ~60 mA forever.
			 * GUI mode calls turn_off_pa from AT TX handler; UW_DOPPLER must
			 * also do it on every TX exit path. */
			MCU_MISC_turn_off_pa();
			/* Restore TCXO warmup after first TX completes */
			if (tcxo_first_tx_skip) {
				MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = false;
				MGR_LOG_DEBUG("[UW_DPL] TCXO warmup restored to %lums\r\n",
					(unsigned long)tcxo_warmup_saved_ms);
			}
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				transition_to(UW_DOPPLER_MONITORING);
			break;

		case KNS_MAC_TX_TIMEOUT:
		/* v11.1.0 introduces KNS_MAC_TX_ABORT — emitted when a TX is
		 * cancelled mid-flight (FIFO flush, stop_send_data). Treat
		 * identically to TX_TIMEOUT: it represents a TX that didn't
		 * land, so the app must release back to MONITORING and apply
		 * the same backoff to avoid radio thrashing. */
		case KNS_MAC_TX_ABORT:
			MGR_LOG_ERR("[UW_DPL] TX %s\r\n",
				(srvcEvt.id == KNS_MAC_TX_ABORT) ? "abort" : "timeout");
			MGR_EVTLOG_log(EVT_TX_TIMEOUT, 0);
			MGR_TXSTATS_recordTimeout();  /* Sprint 4 */
			/* Sprint 4: decrement test burst on failure too, otherwise a
			 * broken radio could keep the burst spinning forever. */
			if (test_tx_remaining > 0)
				test_tx_remaining--;
			MCU_MISC_turn_off_pa();
			if (tcxo_first_tx_skip) {
				MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = false;
			}
			/* Device-error backoff: a TX timeout is the canonical "device
			 * misbehaving" signal — could be a stuck radio, bad antenna
			 * match, supply droop. Backoff prevents thrashing the MAC
			 * stack and the PA in a tight loop. */
			if (consecutive_tx_errors < 0xFFu)
				consecutive_tx_errors++;
			tx_backoff_arm();
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				transition_to(UW_DOPPLER_MONITORING);
			break;

		case KNS_MAC_ERROR:
			MGR_LOG_ERR("[UW_DPL] MAC error: %d\r\n", srvcEvt.status);
			MGR_EVTLOG_log(EVT_MAC_ERROR, (uint16_t)srvcEvt.status);
			/* Always cleanup PA on error — MAC may have already turned it on */
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE ||
			    srvcEvt.app_evt == KNS_MAC_SEND_DATA)
				MCU_MISC_turn_off_pa();
			if (tcxo_first_tx_skip) {
				MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = false;
			}
			/* Count this as a TX error only if it was actually tied to a
			 * SEND_DATA request (others may be benign init-time errors). */
			if (srvcEvt.app_evt == KNS_MAC_SEND_DATA) {
				if (consecutive_tx_errors < 0xFFu)
					consecutive_tx_errors++;
				tx_backoff_arm();
				MGR_TXSTATS_recordError();  /* Sprint 4 */
				if (test_tx_remaining > 0)
					test_tx_remaining--;  /* don't loop forever */
			}
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				transition_to(UW_DOPPLER_MONITORING);
			else if (uw_doppler_state == UW_DOPPLER_WAIT_MAC_READY)
				transition_to(UW_DOPPLER_INIT_MAC); /* retry */
			break;

		default:
			/* Unhandled MAC event during WAIT_TX_DONE — safer to assume the
			 * TX cycle is over and force PA off to prevent stuck 60 mA drain. */
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE) {
				MGR_LOG_WARN("[UW_DPL] Unknown MAC evt %d during WAIT_TX_DONE, cleanup PA\r\n",
					srvcEvt.id);
				MCU_MISC_turn_off_pa();
			}
			break;
		}
	}

	return got_event;
}

/* ---- Public API ---- */

void KNS_APP_uw_doppler_init(void)
{
	/* Force uw_doppler_state to BOOT BEFORE anything reads it.
	 * The variable lives in .retentionRamData (SRAM2). Sram2_Init only runs
	 * on default-boot and SHUTDOWN-wake paths in main.c; on STOP/SLEEP/STANDBY
	 * wake paths the variable keeps its previous value (or stale garbage on
	 * the very first power-up before retention has been initialised). Setting
	 * it explicitly here makes the state machine deterministic regardless of
	 * the wake path. */
	uw_doppler_state = UW_DOPPLER_BOOT;

	reset_tx_scheduling();
	mac_init_retries = 0;
	state_enter_tick = HAL_GetTick();

	/* Seed PRNG so each device/boot picks a different jitter sequence —
	 * important to avoid clustered TX patterns across a multi-tag deployment.
	 * Sprint 3: pull the STM32WL55 96-bit factory unique ID and fold it in.
	 * Without this, two tags freshly powered at the same moment would have
	 * very similar seeds (HAL_GetTick() identical + same .bss address). */
	{
		uint32_t uid = *(volatile uint32_t *)(UID_BASE)
		             ^ *(volatile uint32_t *)(UID_BASE + 4u)
		             ^ *(volatile uint32_t *)(UID_BASE + 8u);
		prng_state ^= uid;
	}
	prng_state ^= HAL_GetTick();
	prng_state ^= (uint32_t)(uintptr_t)&prng_state;
	/* Mix once more so the first prng_next() output already depends on UID. */
	prng_state = prng_state * 1664525UL + 1013904223UL;
	(void)prng_next();  /* discard first value */

	/* Ensure IWDG_STOP option byte is set (bit 17 of FLASH_OPTR). If not,
	 * this triggers a SYSTEM RESET via HAL_FLASH_OB_Launch and does not
	 * return — the next boot will see the bit set and proceed normally.
	 * Idempotent on subsequent boots. Must run BEFORE MGR_WDG_init() so we
	 * don't arm an IWDG that would fire during MAC STOP phases. */
	(void)MGR_WDG_ensureIwdgStopOptionByte();

	/* IWDG ~16s window. Safe with IWDG_STOP=1: watchdog freezes during
	 * STOP mode so MAC stack sleeps don't accumulate. Refreshed from the
	 * main loop and from MGR_WDG_delayWithKick blocking paths.
	 *
	 * Boot-budget contract: between MGR_WDG_init() here and the first main
	 * loop iteration (which refreshes at the top) the init code MUST not
	 * spend more than ~16 s blocked. Every step that performs blocking
	 * I/O (flash erase/read, UART transmit) is followed by an explicit
	 * MGR_WDG_refresh() below so a single slow operation can't bust the
	 * window. */
	MGR_WDG_init();

	/* Init event log (SRAM2 retention - survives resets) */
	MGR_EVTLOG_init();
	MGR_EVTLOG_log(EVT_BOOT, 0);

	/* Boot-loop guard: increments per-boot failure counter, optionally
	 * triggers factory reset or permanent off. Must run BEFORE MGR_NVM_load()
	 * so a wiped flash leads to default config on this very boot. May call
	 * MGR_NVM_reset() which erases 2 flash pages (~100 ms). */
	boot_loop_handle();
	MGR_WDG_refresh();

	/* HardFault forensics: if a fault occurred in the previous boot, the
	 * crash_info struct in retention RAM still has the context. Emit a
	 * direct-UART trace (visible even with DEBUG=0) + log to EVTLOG, then
	 * clear so we only report once. */
	{
		MGR_ERR_CrashInfo_t crash;
		if (MGR_ERR_takeRetainedCrash(&crash)) {
#if defined(USE_UART_DRIVER)
			if (MGR_LOG_passes(MGR_LOG_LVL_ERROR)) {
				extern UART_HandleTypeDef hlpuart1;
				static char cb[208];
				int cn = snprintf(cb, sizeof(cb),
					"\r\n%s[CRASH-REPLAY] type=%u state=%u tick=%lu\r\n"
					"  PC=%08lx LR=%08lx XPSR=%08lx\r\n"
					"  HFSR=%08lx CFSR=%08lx BFAR=%08lx MMFAR=%08lx\r\n",
					MGR_LOG_levelTag(MGR_LOG_LVL_ERROR),
					(unsigned)crash.fault_type, (unsigned)crash.app_state,
					(unsigned long)crash.tick,
					(unsigned long)crash.pc, (unsigned long)crash.lr,
					(unsigned long)crash.xpsr,
					(unsigned long)crash.hfsr, (unsigned long)crash.cfsr,
					(unsigned long)crash.bfar, (unsigned long)crash.mmfar);
				if (cn > 0 && hlpuart1.gState != HAL_UART_STATE_RESET)
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)cb,
						(uint16_t)cn, 200);
			}
#endif
			MGR_EVTLOG_log(EVT_ERROR, (uint16_t)crash.fault_type);
		}
	}
	MGR_WDG_refresh();  /* crash replay UART transmit can take up to 200 ms. */

	/* Rate limiter: init retention ring BEFORE NVM_load so apply_config can
	 * push the persisted config (window_s / max_tx) into it. */
	MGR_RATE_init();

	/* Sprint 4: persistent TX stats + post-mortem flash log. Init order
	 * doesn't matter relative to NVM_load — neither uses NVM config. */
	MGR_TXSTATS_init();
	MGR_PMLOG_init();   /* iterates the flash log ring — ~100 ms worst case */
	MGR_WDG_refresh();

#if defined(BSP_HAS_VBAT_ADC)
	/* BAT init MUST run BEFORE MGR_NVM_load: the loader applies the LB
	 * config via KNS_APP_uw_doppler_setLbCfg() which reads VBAT and
	 * evaluates the hysteresis. Without an initialised BAT manager
	 * (incl. the warm-up read in MGR_BAT_init that primes the external
	 * BJT cascade) the first VBAT read returns ~30 mV and the device
	 * spuriously latches LB mode on a healthy battery — observed every
	 * cold-boot in the long-run STANDBY test. */
	MGR_BAT_init();
#endif

	/* Load saved config from flash (if valid) */
	MGR_NVM_load();
	MGR_WDG_refresh();

	/* MGR_LPM_UW owns the retention-NOLOAD duty config + defaults. */
	MGR_LPM_UW_init();

	/* Seed the session-MC retention block. On full power-on the retention
	 * RAM contains garbage and the magic mismatches → re-seed from NVM via
	 * KNS_CFG_getMC. On STOP2/STANDBY/SHUTDOWN cold-boot the retention is
	 * preserved and we keep the in-flight value. */
	if (mc_retained.magic != MC_RETAINED_MAGIC ||
	    mc_retained.initialised != 1u) {
		uint16_t mc = 0;
		if (KNS_CFG_getMC(&mc) == KNS_STATUS_OK) {
			mc_retained.session_mc = mc;
		} else {
			mc_retained.session_mc = 0;
		}
		mc_retained.magic = MC_RETAINED_MAGIC;
		mc_retained.initialised = 1u;
		MGR_LOG_INFO("[UW_DPL] Session MC seeded from NVM = %u\r\n",
			(unsigned)mc_retained.session_mc);
	}

	MGR_LOG_INFO("[UW_DPL] Init: interval=%us growth=%u%% max=%us deploy=%u\r\n",
		tx_cfg.tx_initial_interval_s, tx_cfg.tx_growth_percent,
		tx_cfg.tx_max_interval_s, deploy_mode);

	/* SWS baselines will be restored after MGR_SWS_init() via
	 * KNS_APP_uw_doppler_restoreSwsBaselines() called from main.c */

	/* Register LPM client to prevent SHUTDOWN during operation */
	MGR_LPM_registerClient(uw_doppler_lpm_client);

#if defined(BSP_HAS_REED_SWITCH)
	MGR_REED_latchPower();
	MGR_LOG_DEBUG("[REED] Init: magnet=%u\r\n", (unsigned)MGR_REED_isMagnetPresent());
#endif
#if defined(BSP_HAS_VBAT_ADC)
	/* MGR_BAT_init already ran above (before NVM_load). Just take a
	 * fresh reading for last_vbat_mV — the BJT cascade is warmed up
	 * so this read is reliable. */
	last_vbat_mV = MGR_BAT_readVoltage_mV();
	MGR_LOG_DEBUG("[BAT] Init: %umV\r\n", last_vbat_mV);
#endif
#if defined(UW_DOPPLER_HAS_GESTURE)
	/* Magnet 2-gesture FSM. Restores persisted OPERATIONAL/CONFIG mode and
	 * plays the 5-blink GREEN wake indicator (slow cadence). The gesture
	 * init owns the boot LED — no separate BLUE-10x blink (would overwrite
	 * the WAKE_BLINK pattern). */
	MGR_GESTURE_init();
#elif defined(BSP_HAS_LED_RGB)
	/* Boot sequence (legacy, non-gesture builds): blink 10x */
	MGR_LED_blink(MGR_LED_BLUE, 10, 200, 200);
#endif
	/* Always go through BOOT state for boot window (UART listen period) */
	transition_to(UW_DOPPLER_BOOT);
}

void KNS_APP_uw_doppler_loop(void)
{
	/* Refresh watchdog every loop iteration */
	MGR_WDG_refresh();

	/* PA watchdog: catch the case where the MAC stack enabled the PA but
	 * never fired TX_DONE/TIMEOUT/ERROR. Without this the GR5504 bias keeps
	 * drawing ~60 mA until IWDG eventually resets (16 s) — and on STDALONE
	 * we've seen freezes where IWDG does fire but the same hang recurs on
	 * the next TX. Forcing PA off + logging + resetting gives a clean recovery
	 * with full diagnostic context. */
	if (MCU_MISC_PA_isStuck(PA_WDG_THRESHOLD_MS)) {
		uint32_t on_ms = MCU_MISC_PA_onDuration_ms();
		MGR_LOG_ERR("[UW_DPL] PA watchdog tripped (on=%lums)\r\n",
			(unsigned long)on_ms);
		MGR_EVTLOG_log(EVT_PA_STUCK, (uint16_t)(on_ms / 100));
		MCU_MISC_turn_off_pa();
		MGR_ERR_logAndReset(ERR_PA_STUCK);
		/* Never returns */
	}

	/* Generic state-hang detector: any non-MONITORING state lingering past
	 * MAX_STATE_HANG_MS is interpreted as a wedged state machine. Per-state
	 * timeouts (TIMEOUT_BOOT_MS, TIMEOUT_MAC_READY_MS, TIMEOUT_TX_DONE_MS,
	 * etc.) catch the common cases sooner; this is the long-tail safety net. */
	if (uw_doppler_state != UW_DOPPLER_MONITORING &&
	    state_elapsed_ms() > MAX_STATE_HANG_MS) {
		MGR_LOG_ERR("[UW_DPL] State hang: state=%u elapsed=%lums\r\n",
			(unsigned)uw_doppler_state, (unsigned long)state_elapsed_ms());
		MGR_EVTLOG_log(EVT_STATE_HANG, (uint16_t)uw_doppler_state);
		MCU_MISC_turn_off_pa();  /* defensive: PA may be stuck on */
		MGR_ERR_logAndReset(ERR_STATE_HANG);
		/* Never returns */
	}

#if defined(USE_UART_DRIVER) && defined(DEBUG)
	/* Heartbeat: direct synchronous UART, bypasses MGR_LOG ring buffer.
	 * Survives silent power resets — last "hb" line tells us the exact tick
	 * at which the device died. 1s cadence to limit noise.
	 *
	 * Compiled OUT in production (DEBUG=0): an OPERATIONAL tag must run
	 * silent unless the build was explicitly debug-instrumented. AT command
	 * responses and crash-replay still go out regardless of DEBUG.
	 *
	 * Hardening:
	 *  - Gated through APP_UART_isEnabled() (ground-truth HAL state, not the
	 *    cached flag) so an unexpected gState=RESET doesn't fault HAL_UART_Transmit.
	 *  - TX timeout 20 ms (115200 baud → ~11 ms for the worst-case 128 B line).
	 *  - Disabled silently after 3 consecutive TX errors — broken cable
	 *    shouldn't keep stalling the loop. */
	{
		extern volatile uint32_t g_reed_isr_count;
		extern volatile uint32_t g_reed_isr_last_state;
		extern volatile uint32_t g_at_isr_bytes;
		extern volatile uint32_t g_at_parse_calls;
		extern volatile uint32_t g_at_cb_null;
		static uint32_t hb_last_tick = 0;
		static uint8_t  hb_tx_errors = 0;
		const uint32_t now = HAL_GetTick();
		/* hb is a TRACE-level forensic — silenced once the operator
		 * raises AT+LOGLVL above TRACE. Still bypasses MGR_LOG ring on
		 * purpose so a stuck ring doesn't lose the last-tick crumb. */
		if ((now - hb_last_tick) >= 1000u && APP_UART_isEnabled() &&
		    hb_tx_errors < 3u &&
		    MGR_LOG_passes(MGR_LOG_LVL_TRACE)) {
			hb_last_tick = now;
			static char hb_buf[160];
#if defined(BSP_HAS_REED_SWITCH)
			const uint32_t reed_now = (uint32_t)MGR_REED_isMagnetPresent();
#else
			const uint32_t reed_now = 0u;  /* no reed on this board */
#endif
			const int hb_n = snprintf(hb_buf, sizeof(hb_buf),
				"%shb t=%lu s=%lu reed=%lu reedISR=%lu pin=%lu | atRX=%lu parse=%lu cbN=%lu\r\n",
				MGR_LOG_levelTag(MGR_LOG_LVL_TRACE),
				(unsigned long)now,
				(unsigned long)uw_doppler_state,
				(unsigned long)reed_now,
				(unsigned long)g_reed_isr_count,
				(unsigned long)g_reed_isr_last_state,
				(unsigned long)g_at_isr_bytes,
				(unsigned long)g_at_parse_calls,
				(unsigned long)g_at_cb_null);
			if (hb_n > 0) {
				/* At 115200 baud, 120 bytes transmit in ~10 ms.
				 * 20 ms gives comfortable margin without starving
				 * the main loop. */
				HAL_StatusTypeDef tx_st = HAL_UART_Transmit(
				    &hlpuart1, (uint8_t *)hb_buf,
				    (uint16_t)hb_n, 20);
				if (tx_st != HAL_OK)
					hb_tx_errors++;
				else
					hb_tx_errors = 0;
			}
		}
	}
#endif

	/* Process pending AT commands */
#if defined(USE_UART_DRIVER)
	{
		uint8_t *pu8_atcmd = MGR_AT_CMD_popNextAt();
		if (pu8_atcmd != NULL) {
			MGR_AT_CMD_decodeAt(pu8_atcmd);
			/* Track AT activity during boot window */
			if (uw_doppler_state <= UW_DOPPLER_BOOT_DEPLOY_LED)
				boot_window_at_received = true;
		}
	}
#endif

#if defined(BSP_HAS_LED_RGB)
	MGR_LED_task();
#endif

	/* Magnet 2-gesture FSM. Drives mode transitions
	 * OPERATIONAL ↔ CONFIG ↔ SHUTDOWN with confirmation window.
	 *
	 * UART gating policy:
	 *   - Boot grace period (5 s after MONITORING) → UART stays ON regardless
	 *     of persisted mode so the operator can see the boot trace.
	 *   - After grace: drive UART based on current mode.
	 *     OPERATIONAL → UART OFF (saves ~1 mA + silences false RX wake).
	 *     CONFIG      → UART ON  (AT command surface available).
	 *   - Override: build with -DUW_DOPPLER_KEEP_UART_ALIVE=1 to keep UART
	 *     on forever (debug builds, factory test). */
#if defined(UW_DOPPLER_HAS_GESTURE)
	MGR_GESTURE_task();

	/* Apply UART state from persisted mode after a one-shot grace period. */
#ifndef UW_DOPPLER_KEEP_UART_ALIVE
#define UW_DOPPLER_KEEP_UART_ALIVE 1
#endif
#if !(UW_DOPPLER_KEEP_UART_ALIVE)
	{
		static bool s_uart_init_done = false;
		static uint32_t s_monitoring_start_tick = 0;
		if (uw_doppler_state == UW_DOPPLER_MONITORING) {
			if (s_monitoring_start_tick == 0)
				s_monitoring_start_tick = HAL_GetTick();
			uint32_t elapsed = HAL_GetTick() - s_monitoring_start_tick;
			if (!s_uart_init_done && elapsed >= 5000u) {
				MGR_GESTURE_Mode_t m = MGR_GESTURE_getMode();
				APP_UART_setEnabled(m == MGR_GESTURE_MODE_CONFIG);
				s_uart_init_done = true;
			}
		}
	}
#endif

	{
		MGR_GESTURE_Event_t gevt = MGR_GESTURE_getEvent();
		if (gevt == MGR_GESTURE_EVT_ENTER_CONFIG) {
			MGR_LOG_INFO("[UW_DPL] Magnet → CONFIG mode\r\n");
			/* Power down the SWS analog rail: in CONFIG we don't
			 * sample anymore (see main-loop pause above). */
			MGR_SWS_enterLowPower();
#if !(UW_DOPPLER_KEEP_UART_ALIVE)
			APP_UART_setEnabled(true);
#endif
		} else if (gevt == MGR_GESTURE_EVT_ENTER_OPERATIONAL) {
			MGR_LOG_INFO("[UW_DPL] Magnet → OPERATIONAL mode\r\n");
			/* Bring the SWS analog rail back up so MGR_SWS_task()
			 * sees stable readings on its next sample. */
			MGR_SWS_exitLowPower();
#if !(UW_DOPPLER_KEEP_UART_ALIVE)
			APP_UART_setEnabled(false);
#endif
		} else if (gevt == MGR_GESTURE_EVT_REQUEST_SHUTDOWN) {
			MGR_LOG_INFO("[UW_DPL] Magnet → SHUTDOWN request\r\n");
			transition_to(UW_DOPPLER_SHUTDOWN_BLINK);
			return;
		}
	}

	/* ----- Steady-state LED indicator --------------------------------
	 * Background colour while gesture FSM is idle in MONITORING. Priority:
	 *   1. Low-battery   → YELLOW slow blink (250 ms / 1750 ms) urgent
	 *   2. CONFIG + UART → solid BLUE        ("session live", <5 s RX)
	 *   3. CONFIG idle   → BLUE slow blink   (500 / 500)
	 *   4. otherwise     → yield (don't touch the LED)
	 *
	 * Yields to: gesture FSM busy, non-MONITORING states, AT+DIAG
	 * (blocking — main loop doesn't run during HAL_Delay).
	 *
	 * Idempotency comes from MGR_LED_blink's own no-restart-if-same-params
	 * check, so we can call the indicator each iteration. A blink killed
	 * by an external MGR_LED_set/off (e.g. AT+DIAG end) gets revived on
	 * the very next iteration since `blinking` is then false. */
	{
		const bool eligible =
		    (uw_doppler_state == UW_DOPPLER_MONITORING) &&
		    !MGR_GESTURE_isInteracting();

		if (eligible) {
			if (lb_active) {
				MGR_LED_blink(MGR_LED_YELLOW, 0u,
				              250u, 1750u);
			} else if (MGR_GESTURE_getMode() ==
			           MGR_GESTURE_MODE_CONFIG) {
				const uint32_t since_rx_ms =
				    APP_UART_msSinceRx();
				if (since_rx_ms < 5000u)
					MGR_LED_set(MGR_LED_BLUE);
				else
					MGR_LED_blink(MGR_LED_BLUE, 0u,
					              500u, 500u);
			}
		}
	}
#elif defined(BSP_HAS_REED_SWITCH)
	/* Fallback raw reed handling for builds without LED RGB (e.g. board
	 * variants where MGR_GESTURE is not applicable). Only handles the
	 * long-hold → shutdown path. */
	{
		MGR_REED_Event_t evt = MGR_REED_getEvent();
		if (evt == MGR_REED_EVT_MAGNET_ON) {
			MGR_EVTLOG_log(EVT_REED_ON, 0);
			magnet_on_tick = HAL_GetTick();
			shutdown_triggered = false;
		} else if (evt == MGR_REED_EVT_MAGNET_OFF) {
			MGR_EVTLOG_log(EVT_REED_OFF,
			    (uint16_t)(MGR_REED_getLastHoldDuration_ms() / 100));
			magnet_on_tick = 0;
			shutdown_triggered = false;
		}
		if (MGR_REED_isMagnetPresent() && !shutdown_triggered && magnet_on_tick > 0) {
			uint32_t hold_ms = HAL_GetTick() - magnet_on_tick;
			if (hold_ms >= REED_SHUTDOWN_HOLD_MS) {
				shutdown_triggered = true;
				transition_to(UW_DOPPLER_SHUTDOWN_BLINK);
				return;
			}
		}
	}
#endif

	/* Run SWS measurement after boot.
	 * Paused in CONFIG mode: the user is doing AT-command work, the tag is
	 * almost certainly out of the water for inspection, and SWS sampling
	 * spams the UART (one [SWS] line per second), drains the battery
	 * uselessly, and keeps the analog rail powered. Resumes automatically
	 * when the user goes back to OPERATIONAL. */
#if defined(UW_DOPPLER_HAS_GESTURE)
	const bool sws_paused_config =
	    (MGR_GESTURE_getMode() == MGR_GESTURE_MODE_CONFIG);
#else
	const bool sws_paused_config = false;
#endif
	if (uw_doppler_state >= UW_DOPPLER_MONITORING && !sws_paused_config) {
		MGR_SWS_task();
		/* Save adapted baselines to retention RAM for warm reset recovery */
		sws_retained.magic = SWS_RETAINED_MAGIC;
		sws_retained.air_baseline = MGR_SWS_getAirBaseline();
		sws_retained.water_baseline = MGR_SWS_getWaterBaseline();
		sws_retained.observed_peak_adc = MGR_SWS_getObservedPeak();
	}

	switch (uw_doppler_state) {
	case UW_DOPPLER_BOOT:
	{
		/* Boot window: UART listens for AT commands.
		 * With LED: wait for blink to finish (with timeout).
		 * Without LED: wait BOOT_WINDOW_MS for AT commands. */
		bool boot_done = false;
#if defined(BSP_HAS_LED_RGB)
		boot_done = (MGR_LED_isBlinkDone() || state_elapsed_ms() > TIMEOUT_BOOT_MS);
#else
		boot_done = (state_elapsed_ms() > BOOT_WINDOW_MS);
#endif
		if (boot_done) {
#if defined(BSP_HAS_LED_RGB)
			/* Show deploy status: green=deployed, blue=not deployed */
			if (deploy_mode)
				MGR_LED_set(MGR_LED_GREEN);
			else
				MGR_LED_set(MGR_LED_BLUE);
#endif
			transition_to(UW_DOPPLER_BOOT_DEPLOY_LED);
		}
		return;
	}

	case UW_DOPPLER_BOOT_DEPLOY_LED:
	{
		/* Show deploy color for 2s then proceed (with timeout) */
		bool deploy_led_done = false;
#if defined(BSP_HAS_LED_RGB)
		deploy_led_done = (state_elapsed_ms() >= BOOT_DEPLOY_LED_MS ||
		                   state_elapsed_ms() > TIMEOUT_BOOT_DEPLOY_MS);
#else
		deploy_led_done = true;  /* No LED: skip immediately */
#endif
		if (deploy_led_done) {
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
#if defined(USE_UART_DRIVER)
			/* DURING DEV: keep UART alive in deploy_mode unconditionally.
			 *
			 * BUG (2026-06-05): production build (DEBUG=0) was hitting the
			 * #else branch which did HAL_UART_DeInit + HAL_GPIO_DeInit on
			 * the LPUART pins for "power savings in deployed multi-year tag".
			 * That killed every subsequent UART log → looked like the chip
			 * hung at BOOT_DEPLOY_LED → INIT_MAC. State machine was actually
			 * fine and reached MONITORING (s=4), just silent.
			 *
			 * Real fix: gate UART deinit on the magnet 2-gesture "operational"
			 * vs "config" mode (Phase 4), not on DEBUG flag. For now keep UART
			 * alive in all cases so dev/test can observe MAC, TX, SWS, REED. */
			(void)boot_window_at_received;
#endif
			transition_to(UW_DOPPLER_INIT_MAC);
		}
		return;
	}

	case UW_DOPPLER_INIT_MAC:
		start_mac_profile();
		transition_to(UW_DOPPLER_WAIT_MAC_READY);
		return;

	case UW_DOPPLER_WAIT_MAC_READY:
		process_mac_events();
		/* Timeout protection */
		if (uw_doppler_state == UW_DOPPLER_WAIT_MAC_READY &&
		    state_elapsed_ms() > TIMEOUT_MAC_READY_MS) {
			MGR_LOG_WARN("[UW_DPL] MAC ready timeout (retry %u/%u)\r\n",
				mac_init_retries + 1, MAX_MAC_INIT_RETRIES);
			MGR_EVTLOG_log(EVT_TIMEOUT, (uint16_t)UW_DOPPLER_WAIT_MAC_READY);
			MGR_ERR_log(ERR_MAC_TIMEOUT);
			if (++mac_init_retries >= MAX_MAC_INIT_RETRIES) {
				MGR_LOG_ERR("[UW_DPL] MAC init failed after %u retries, resetting\r\n",
					MAX_MAC_INIT_RETRIES);
				MGR_EVTLOG_log(EVT_ERROR, (uint16_t)ERR_MAC_INIT_FAIL);
				MGR_ERR_logAndReset(ERR_MAC_INIT_FAIL);
				/* Never reaches here */
			}
			transition_to(UW_DOPPLER_INIT_MAC);
		}
		return;

	case UW_DOPPLER_MONITORING:
	{
		/* Process any pending MAC events */
		process_mac_events();

		MGR_SWS_State_t sws_state = MGR_SWS_getState();

		/* Cold-boot UW→SURFACE detection: compares the freshly-sampled
		 * SWS state against the one persisted by the previous duty cycle
		 * (.retentionRamNoload). If the previous cycle saw UNDERWATER and
		 * we now read SURFACE we force an immediate TX even if the SWS
		 * task didn't report a transition (it may have stabilized on
		 * SURFACE before stateChanged() was polled). Per-cycle no-op
		 * once the persisted state matches the current one. */
		if (MGR_LPM_UW_detectSurfaceWake((int)sws_state)) {
			LAT_MARK_T0();  /* T0 cold-boot SURF detect (post-STOP2 wake) */
			STATE_TRACE("UW -> SURFACE (post-STOP2 wake)");
			MGR_LOG_INFO("[UW_DPL] Cold-boot UW->SURF detected, TX ASAP\r\n");
			MGR_EVTLOG_log(EVT_SWS_SURFACE, MGR_SWS_getLastADC());
			reset_tx_scheduling();
			surface_tx_pending = true;
			first_tx_random_offset_ms =
				prng_next() % FIRST_TX_RANDOM_WINDOW_MS;
			MGR_LPM_UW_clearWakeShouldTx();
			/* HAL_GetTick is paused inside STOP2 → since_last_tx
			 * resolves to "active-time since last TX" instead of
			 * wall-clock seconds. A surface event after a long
			 * STOP2 sleep would then sit waiting for the initial
			 * interval to "elapse" in active time, which can take
			 * dozens of sleep cycles. Clear last_tx_tick so the
			 * first TX of the new burst is gated only by the
			 * 5 s hardcoded safety floor + anti-collision random
			 * offset. The rate limiter / backoff still protect
			 * against true spam. */
			last_tx_tick = 0;
			/* Arm the MAC-side TCXO warmup bypass for the 1st surface
			 * TX. The configured AT+TCXO_WU value still applies for
			 * subsequent TXs in the same session — only the 1st gets
			 * the fast path. Restored on TX_DONE / TIMEOUT / ERROR. */
			MCU_MISC_TCXO_get_warmup(&tcxo_warmup_saved_ms);
			tcxo_first_tx_skip = true;
		}

		/* Boot-as-first-surface-event: a fresh OPERATIONAL boot
		 * (power-on, magnet wake, NRST) with the tag already at the
		 * surface must start a TX sequence — there is no dive/resurface
		 * transition to detect. One-shot, consumed at the first known
		 * SWS state. Skipped on STANDBY duty-cycle wakes: sequencing
		 * there belongs to the UW→SURF detection + seq-restart timer. */
		if (boot_first_seq_pending &&
		    sws_state != MGR_SWS_STATE_UNKNOWN) {
			boot_first_seq_pending = false;
			if (sws_state == MGR_SWS_STATE_SURFACE &&
			    !surface_tx_pending &&
			    !MGR_LPM_UW_isWakeFromStandby()) {
				LAT_MARK_T0();  /* T0 boot-at-SURFACE */
				STATE_TRACE("BOOT at SURFACE -> first sequence");
				MGR_LOG_INFO("[UW_DPL] Boot at SURFACE, starting first sequence\r\n");
				MGR_EVTLOG_log(EVT_SWS_SURFACE, MGR_SWS_getLastADC());
				reset_tx_scheduling();
				surface_tx_pending = true;
				first_tx_random_offset_ms =
					prng_next() % FIRST_TX_RANDOM_WINDOW_MS;
				last_tx_tick = 0;
				/* Same TCXO warmup bypass as the other sequence starts. */
				MCU_MISC_TCXO_get_warmup(&tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = true;
			}
		}

		/* Check for surface detection */
		if (MGR_SWS_stateChanged()) {
			if (sws_state == MGR_SWS_STATE_SURFACE) {
				LAT_MARK_T0();  /* T0 live SURF transition */
				STATE_TRACE("UW -> SURFACE");  /* very visible */
				UWDPL_TRACE("SURF detected");
				/* Surface detected! Schedule immediate TX */
				MGR_LOG_INFO("[UW_DPL] Surface detected, starting TX\r\n");
				/* Same reset as the cold-boot path: HAL tick paused
				 * during STOP2 breaks the active-time interval
				 * arithmetic, so we force the gate open by clearing
				 * last_tx_tick. Live transitions are reached on the
				 * very next loop after STOP2 wake when SWS samples
				 * fresh, which is the same scenario. */
				last_tx_tick = 0;
				/* Arm MAC-side TCXO warmup bypass for the 1st TX of
				 * this surface burst (see cold-boot path above). */
				MCU_MISC_TCXO_get_warmup(&tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = true;
				MGR_EVTLOG_log(EVT_SWS_SURFACE, MGR_SWS_getLastADC());
				reset_tx_scheduling();
				surface_tx_pending = true;
				/* Draw a fresh anti-collision delay for the first TX of
				 * this surface burst. UID-seeded PRNG gives per-tag spread. */
				first_tx_random_offset_ms =
					prng_next() % FIRST_TX_RANDOM_WINDOW_MS;

				/* Pre-warmup TCXO: start it now so it's stable by the time
				 * the first TX fires. Save current warmup setting so we can
				 * temporarily zero it for the first TX (avoid double wait).
				 * Re-entrance guard: don't overwrite saved value if a previous
				 * skip is still pending (would store the already-zeroed value).
				 */
				/* TCXO pre-warmup DISABLED on this app/board combination.
				 *
				 * Why: on STM32WL5x the TCXO power rail (VDDTCXO) is gated by
				 * the SubGHz radio peripheral via SUBGHZ_Set_TcxoMode, NOT by
				 * a GPIO the MCU can drive. Between TX bursts the radio is in
				 * sleep mode, so VDDTCXO is LOW and the TCXO does NOT oscillate.
				 * Calling MCU_MISC_TCXO_Force_State(true) at that moment makes
				 * HAL_RCC_OscConfig wait for HSERDY which never asserts and
				 * times out — and even more critically, when MAC later processes
				 * the queued TX request, its own TCXO management may assert
				 * because of the partially-configured clock state, resetting
				 * the chip.
				 *
				 * GUI mode (mgr_at_cmd_list_user_data.c:150) calls this
				 * successfully because TX happens immediately after MAC init,
				 * before the radio goes to sleep.
				 *
				 * Fix: let the Kineis MAC stack wake the radio and manage TCXO
				 * itself when it processes the TX request. We do NOT set
				 * tcxo_first_tx_skip so SURFACE_TX leaves the warmup at its
				 * configured value (no `set_warmup(0)` optimisation). */
				(void)tcxo_warmup_saved_ms;  /* unused now */
#if defined(BSP_HAS_LED_RGB)
				MGR_LED_blink(MGR_LED_GREEN, 1, 500, 0);
#endif
			} else {
				/* Went underwater, stop TX scheduling */
				STATE_TRACE("SURFACE -> UW");
				MGR_LOG_INFO("[UW_DPL] Underwater, stopping TX\r\n");
				MGR_EVTLOG_log(EVT_SWS_UNDERWATER, MGR_SWS_getLastADC());
				reset_tx_scheduling();
				/* Reset latency T0 so a stale tick from a brief SURF
				 * bounce doesn't pollute the next legit surface→TX
				 * measurement. The campaign only cares about full
				 * UW→SURF→TX cycles. */
				LAT_RESET();
				/* TCXO pre-warmup is disabled (see surface branch note above)
				 * so there is nothing to release here. The MAC stack manages
				 * TCXO power-down itself when the radio re-enters sleep. */
#if defined(BSP_HAS_LED_RGB)
				MGR_LED_blink(MGR_LED_BLUE, 1, 500, 0);
#endif
			}

			/* Persist updated baselines/peak to flash (debounced).
			 * State changes are the natural moment to save: calibration just
			 * finished applying for the previous state.
			 */
			(void)MGR_NVM_saveCalibDebounced(NVM_CALIB_SAVE_MIN_INTERVAL_S);
		}

		/* Sprint 4: forced test burst. If queued, fire immediately bypassing
		 * deploy_mode / surface / rate / cooldown / backoff. Still respects
		 * MIN_INTER_TX_INTERVAL_MS so the PA isn't hammered back-to-back. */
		if (test_tx_remaining > 0) {
			uint32_t since = HAL_GetTick() - last_tx_tick;
			if (since >= MIN_INTER_TX_INTERVAL_MS) {
				MGR_LOG_INFO("[UW_DPL] Test TX %u remaining\r\n",
					test_tx_remaining);
				MGR_EVTLOG_log(EVT_TX_START, (uint16_t)tx_count);
				transition_to(UW_DOPPLER_SURFACE_TX);
				return;  /* loop again next tick to enter SURFACE_TX */
			}
			return;  /* still waiting on safety floor */
		}

		/* TX scheduling logic - only TX if deployed.
		 * Gated on CONFIG mode: a tag being configured is on the bench,
		 * not in the water. Auto TX would burn the daily Argos quota
		 * for a session that's not generating real positions. AT+TEST
		 * remains available for radio validation (handled above). */
#if defined(UW_DOPPLER_HAS_GESTURE)
		const bool gesture_in_config =
		    (MGR_GESTURE_getMode() == MGR_GESTURE_MODE_CONFIG);
#else
		const bool gesture_in_config = false;
#endif
		if (deploy_mode && sws_state == MGR_SWS_STATE_SURFACE &&
		    !gesture_in_config) {
			bool should_tx = false;
			uint32_t since_last_tx = HAL_GetTick() - last_tx_tick;

			/* Effective minimum interval between any two TX requests.
			 * Uses the largest of:
			 *   - MIN_INTER_TX_INTERVAL_MS  (safety floor, hardcoded)
			 *   - tx_initial_interval_s     (user-configured initial gap)
			 *   - tx_cooldown_s             (global post-TX quiet time, also
			 *                                applies across surface/dive cycles)
			 * The cooldown is what stops a turtle that oscillates UW/SURF
			 * every few seconds from spamming TX bursts each time it surfaces. */
			uint32_t effective_min_ms = MIN_INTER_TX_INTERVAL_MS;
			/* In LB mode the initial interval is replaced by lb_tx_interval_s. */
			uint32_t configured_min_ms = lb_active
				? (uint32_t)lb_cfg.lb_tx_interval_s * 1000
				: (uint32_t)tx_cfg.tx_initial_interval_s * 1000;
			uint32_t cooldown_ms = (uint32_t)tx_cfg.tx_cooldown_s * 1000;
			if (configured_min_ms > effective_min_ms)
				effective_min_ms = configured_min_ms;
			if (cooldown_ms > effective_min_ms)
				effective_min_ms = cooldown_ms;

			if (surface_tx_pending) {
				/* Surface detected (first ever, or re-surface): fire the
				 * first TX as soon as the anti-collision random offset
				 * has elapsed.
				 *
				 * Why we no longer apply `effective_min_ms` here: it
				 * mixes `tx_initial_interval_s` (intended as the gap
				 * BETWEEN consecutive TXs of a burst) and `tx_cooldown_s`
				 * (intended as a post-burst quiet window) with the
				 * 5 s safety floor. None of those should delay the
				 * *first* TX of a surface event — the operator wants to
				 * minimize latency from "head out of water" to "Argos
				 * frame on the wire". Inter-TX cadence is handled below
				 * by `current_interval_ms` once tx_count > 0, and global
				 * spam is bounded by the rate limiter + backoff (rate
				 * limiter uses RTC-seconds so it remains accurate even
				 * across STOP2 cycles where HAL_GetTick is paused). */
				if (since_last_tx >= first_tx_random_offset_ms) {
					should_tx = true;
					surface_tx_pending = false;
				}
			} else if (tx_count > 0 && current_interval_ms > 0) {
				/* Periodic TX while still SURFACE — use the schedule's
				 * current_interval_ms (which grows with tx_count). */
				if (since_last_tx >= current_interval_ms)
					should_tx = true;
			}

			/* Check max TX count — LB mode swaps in lb_tx_max_count. */
			{
				uint8_t cap = lb_active ? lb_cfg.lb_tx_max_count : tx_cfg.tx_max_count;
				if (should_tx && cap > 0 && tx_count >= cap)
					should_tx = false;
			}

			/* Sequence-restart timer (tx_seq_restart_s). If the cap was
			 * reached and the tag has been idle at the surface for
			 * tx_seq_restart_s seconds since the last TX, start a fresh
			 * sequence — without needing a UW→SURFACE transition or
			 * cold-boot wake. reset_tx_scheduling() zeroes tx_count, so
			 * the next loop pass sees `tx_count == 0` and rolls over to
			 * the "first TX" branch which bumps the Message Counter
			 * automatically via mc_retained.session_mc update. Only
			 * meaningful when tx_max_count > 0 (otherwise the cap is
			 * never reached). */
			{
				const uint8_t cap =
					lb_active ? lb_cfg.lb_tx_max_count : tx_cfg.tx_max_count;
				if (tx_cfg.tx_seq_restart_s > 0u &&
				    cap > 0u && tx_count >= cap &&
				    last_tx_tick > 0u &&
				    (HAL_GetTick() - last_tx_tick) >=
				        ((uint32_t)tx_cfg.tx_seq_restart_s * 1000u)) {
					MGR_LOG_INFO("[UW_DPL] Seq restart timer fired "
						"(%us since last TX, MC will bump)\r\n",
						(unsigned)tx_cfg.tx_seq_restart_s);
					reset_tx_scheduling();
					/* Trigger the first TX of the new sequence on the
					 * next loop pass (do not set should_tx here so the
					 * normal first-TX guards — battery, rate limiter,
					 * backoff — still run). */
					surface_tx_pending = true;
				}
			}

			if (should_tx) {
#if defined(BSP_HAS_VBAT_ADC)
				last_vbat_mV = MGR_BAT_readVoltage_mV();
				MGR_EVTLOG_log(EVT_BAT, last_vbat_mV);
				/* Update LB hysteretic state on every fresh reading. */
				lb_update(last_vbat_mV);
				if (!MGR_BAT_isTxAllowedAt(last_vbat_mV)) {
					MGR_LOG_WARN("[UW_DPL] Battery low (%umV < %umV), TX inhibited\r\n",
						last_vbat_mV, MGR_BAT_getMinTxVoltage_mV());
					should_tx = false;
				}
#endif
			}
			if (should_tx) {
				/* Rate limiter: hard cap on TX bursts over the rolling window.
				 * Survives resets, so a crash-loop can't bypass it. */
				uint32_t retry_in_s = 0;
				if (MGR_RATE_isBlocked(&retry_in_s)) {
					MGR_LOG_WARN("[UW_DPL] Rate limit hit, retry in %lus\r\n",
						(unsigned long)retry_in_s);
					MGR_EVTLOG_log(EVT_RATE_BLOCKED,
						(uint16_t)(retry_in_s > 0xFFFFu ? 0xFFFFu : retry_in_s));
					should_tx = false;
				}
			}
			if (should_tx) {
				/* Device-error backoff: skip TX while we're inside a
				 * post-error quiet window (exponential, capped). */
				uint32_t retry_in_s = 0;
				if (tx_backoff_blocked(&retry_in_s)) {
					MGR_LOG_WARN("[UW_DPL] Backoff active, retry in %lus\r\n",
						(unsigned long)retry_in_s);
					should_tx = false;
				}
			}
			if (should_tx) {
				UWDPL_TRACE("TX should=yes");
				LAT_TRACE("SURF_TX");  /* T1: enter SURFACE_TX state */
				MGR_EVTLOG_log(EVT_TX_START, (uint16_t)tx_count);
				transition_to(UW_DOPPLER_SURFACE_TX);
				/* Fall through to SURFACE_TX */
			} else {
				/* Idle surface or no TX due — let MGR_LPM_UW decide
				 * whether to drop to STANDBY / SHUTDOWN+RTC. Gates
				 * (gesture, CONFIG, test burst, surface_tx_pending,
				 * stabilization window) are all inside the module. */
				if (test_tx_remaining == 0u && !surface_tx_pending) {
#if defined(UW_DOPPLER_HAS_GESTURE)
					const bool g_busy   = MGR_GESTURE_isInteracting();
					const bool g_config = (MGR_GESTURE_getMode() ==
					                       MGR_GESTURE_MODE_CONFIG);
#else
					const bool g_busy = false, g_config = false;
#endif
					/* Event-driven LPM: sleep exactly until the next
					 * scheduled action (next SWS sample, next TX of
					 * the sequence, or the seq-restart deadline —
					 * whichever comes first). spin / SLEEP / STOP2 is
					 * picked from AT+LPMTHR thresholds. delta==0 means
					 * an action is due now: skip the sleep, the state
					 * machine handles it on the next pass. */
					const uint32_t delta_ms = uw_ms_until_next_action();
					if (delta_ms > 0u)
						MGR_LPM_UW_idleTick((int)sws_state, delta_ms,
						                    g_busy, g_config);
				}
				return;
			}
		} else {
			/* Underwater or deploy_mode=0 idle — deepest sleep path.
			 * No TX can be scheduled here; the only deadline is the
			 * next SWS sample (fast cadence = fast surface detection). */
			if (test_tx_remaining == 0u && !surface_tx_pending) {
#if defined(UW_DOPPLER_HAS_GESTURE)
				const bool g_busy   = MGR_GESTURE_isInteracting();
				const bool g_config = (MGR_GESTURE_getMode() ==
				                       MGR_GESTURE_MODE_CONFIG);
#else
				const bool g_busy = false, g_config = false;
#endif
				const uint32_t delta_ms = MGR_SWS_msUntilNextSample();
				if (delta_ms > 0u)
					MGR_LPM_UW_idleTick((int)sws_state, delta_ms,
					                    g_busy, g_config);
			}
			return;
		}
	}
	/* Fall through */

	case UW_DOPPLER_SURFACE_TX:
	{
		UWDPL_TRACE("SURF_TX enter");
#if defined(BSP_HAS_LED_RGB)
		/* VIOLET (R+B soft-PWM) for TX-in-flight. The common-anode RGB with a
		 * single anode resistor can't drive R+B simultaneously, so a solid
		 * VIOLET would clamp to RED only. We use the soft-PWM rotator in
		 * MGR_LED (333 Hz channel switching) which time-multiplexes the
		 * cathodes — the eye integrates a clean magenta. */
		MGR_LED_set(MGR_LED_VIOLET);
#endif
		UWDPL_TRACE("SURF_TX LED set");

		/* TCXO pre-warmup: SKIPPED on UW_DOPPLER.
		 *
		 * Why: MCU_MISC_TCXO_Force_State(true) consistently returns
		 * HAL_TIMEOUT (= 3) on this board because VDDTCXO is gated by the
		 * SubGHz radio peripheral (not a GPIO). Between TX bursts the radio
		 * is in sleep mode → VDDTCXO is LOW → TCXO doesn't oscillate →
		 * HSERDY never asserts within the HAL timeout. The subsequent
		 * MGR_WDG_delayWithKick(warmup_ms) call then sits doing nothing
		 * useful for `warmup_ms` (2 s default) — pure dead time on the
		 * surface→TX critical path.
		 *
		 * The Kineis MAC stack manages TCXO power itself when it wakes the
		 * radio for the TX request: it sets up VDDTCXO via SubGHz commands
		 * BEFORE clocking the PLL. So the warmup happens naturally inside
		 * KNS_Q_push → MAC processing, costing the same time but no longer
		 * sequential to our app loop.
		 *
		 * Override at compile time with `-DUW_DOPPLER_PREWARM_TCXO=1` if
		 * you want to re-enable (e.g. testing a board with GPIO-controlled
		 * VDDTCXO). Default: disabled. Removes ~2 s from the latency. */
#ifndef UW_DOPPLER_PREWARM_TCXO
#define UW_DOPPLER_PREWARM_TCXO 0
#endif
#if UW_DOPPLER_PREWARM_TCXO
		{
			uint32_t warmup_ms = 0;
			MCU_MISC_TCXO_Force_State(true);
			MCU_MISC_TCXO_get_warmup(&warmup_ms);
			if (warmup_ms > 30000)
				warmup_ms = 30000;
			if (MGR_LOG_passes(MGR_LOG_LVL_INFO)) {
				static char _tcxo_buf[48];
				int _n = snprintf(_tcxo_buf, sizeof(_tcxo_buf),
					"%s[TCXO] warmup %lums\r\n",
					MGR_LOG_levelTag(MGR_LOG_LVL_INFO),
					(unsigned long)warmup_ms);
				if (_n > 0 && hlpuart1.gState != HAL_UART_STATE_RESET)
					HAL_UART_Transmit(&hlpuart1,
						(uint8_t *)_tcxo_buf,
						(uint16_t)_n, 100);
			}
			if (warmup_ms > 0)
				MGR_WDG_delayWithKick(warmup_ms);
			UWDPL_TRACE("TCXO ready");
		}
#else
		UWDPL_TRACE("TCXO skip (MAC manages)");
#endif

		struct KNS_MAC_appEvt_t appEvt;
		if (!build_tx_payload(&appEvt)) {
			/* Radio config error - skip TX, go back to monitoring */
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			transition_to(UW_DOPPLER_MONITORING);
			return;
		}

		/* For the very first TX after surface detection, the TCXO has been
		 * pre-warmed since UW->SURFACE; tell the MAC to skip its warmup wait
		 * by setting it to 0 just before the push. Restore on TX done.
		 */
		if (tcxo_first_tx_skip) {
			MCU_MISC_TCXO_set_warmup(0);
			MGR_LOG_DEBUG("[UW_DPL] First TX: TCXO warmup bypassed (was prewarmed)\r\n");
		}

		/* Session-MC handling. On the very first TX of a sequence
		 * (tx_count == 0), snapshot the current persisted MC and adopt
		 * it as the session value. The lib's auto-increment after each
		 * TX has already supplied the +1 from the previous sequence, so
		 * we don't bump explicitly — just read what the lib persisted.
		 * For every TX of the sequence (including the first), stomp MC
		 * back to s_session_mc so retransmissions ship the same value
		 * the GUI / ground segment expects for the sequence. */
		if (tx_count == 0u) {
			uint16_t mc_now = 0;
			if (KNS_CFG_getMC(&mc_now) == KNS_STATUS_OK) {
				mc_retained.session_mc = mc_now;
				MGR_LOG_INFO("[UW_DPL] New sequence, MC=%u\r\n",
					(unsigned)mc_retained.session_mc);
			}
		}
		/* Override the auto-incremented MC so every TX in this sequence
		 * uses the same Message Counter. */
		(void)KNS_CFG_setMC(mc_retained.session_mc);

		UWDPL_TRACE("KNS_Q_push start");
		enum KNS_status_t status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
		UWDPL_TRACE("KNS_Q_push done");
		if (status == KNS_STATUS_OK) {
			LAT_TRACE("MAC_PUSH");  /* T2: frame submitted to MAC */
			MGR_LOG_INFO("[UW_DPL] TX #%lu sent (interval=%lums)\r\n",
				tx_count, current_interval_ms);

			MGR_TXSTATS_recordAttempt();  /* Sprint 4 — count the request */
			last_tx_tick = HAL_GetTick();
			current_interval_ms = compute_next_interval_ms(tx_count);
			tx_count++;

			UWDPL_TRACE("→ WAIT_TX_DONE");
			transition_to(UW_DOPPLER_WAIT_TX_DONE);
		} else {
			MGR_LOG_ERR("[UW_DPL] TX push failed: 0x%x\r\n", status);
			/* Restore TCXO warmup since the MAC never picked up the request */
			if (tcxo_first_tx_skip) {
				MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = false;
			}
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			transition_to(UW_DOPPLER_MONITORING);
		}
		return;
	}

	case UW_DOPPLER_WAIT_TX_DONE:
		process_mac_events();
		if (uw_doppler_state == UW_DOPPLER_MONITORING) {
			/* TX complete, turn off TX LED */
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
		}
		/* Timeout protection */
		if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE &&
		    state_elapsed_ms() > TIMEOUT_TX_DONE_MS) {
			MGR_LOG_ERR("[UW_DPL] TX done timeout\r\n");
			MGR_EVTLOG_log(EVT_TIMEOUT, (uint16_t)UW_DOPPLER_WAIT_TX_DONE);
			MGR_ERR_log(ERR_TX_TIMEOUT);
			/* MAC stack never fired TX_DONE/TIMEOUT/ERROR. PA may have been
			 * left enabled (drains ~60 mA continuously). Force cleanup. */
			MCU_MISC_turn_off_pa();
			/* Restore TCXO warmup: TX_DONE never fired, MAC may be stuck */
			if (tcxo_first_tx_skip) {
				MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = false;
			}
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			transition_to(UW_DOPPLER_MONITORING);
		}
		return;

#if defined(BSP_HAS_REED_SWITCH)
	case UW_DOPPLER_SHUTDOWN_BLINK:
#if !defined(UW_DOPPLER_HAS_GESTURE)
		/* Legacy reed-hold-only path: cancel if magnet is released
		 * during the blink (deadman switch — no upstream confirmation
		 * exists, so the blink itself is the user's last chance to
		 * abort). */
		if (!MGR_REED_isMagnetPresent()) {
			MGR_LOG_INFO("[UW_DPL] Shutdown cancelled (magnet removed)\r\n");
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			shutdown_triggered = false;
			transition_to(UW_DOPPLER_MONITORING);
			return;
		}
#else
		/* Gesture FSM path: the magnet touch inside the 2 s
		 * WAIT_CONFIRM window IS the confirmation. Once we land here
		 * the user has already committed (hold ≥6 s + release + touch
		 * within 2 s) and the blink is purely visual feedback while
		 * the LPM teardown unwinds. Don't add a second deadman — the
		 * operator was confused by being asked to keep the magnet
		 * applied through the slow red blink. */
#endif
		/* Wait for blink to finish or timeout */
#if defined(BSP_HAS_LED_RGB)
		if (MGR_LED_isBlinkDone() || state_elapsed_ms() > TIMEOUT_SHUTDOWN_BLINK_MS)
#else
		if (state_elapsed_ms() > TIMEOUT_SHUTDOWN_BLINK_MS)
#endif
		{
			enter_shutdown();
			/* Never reaches here - board loses power */
		}
		return;
#endif

	default:
		MGR_LOG_ERR("[UW_DPL] Invalid state %u, resetting to INIT_MAC\r\n",
			uw_doppler_state);
		transition_to(UW_DOPPLER_INIT_MAC);
		break;
	}
}

KNS_APP_UwDopplerTxCfg_t KNS_APP_uw_doppler_getTxCfg(void)
{
	return tx_cfg;
}

void KNS_APP_uw_doppler_setTxCfg(const KNS_APP_UwDopplerTxCfg_t *cfg)
{
	if (cfg) {
		tx_cfg = *cfg;
		MGR_LOG_DEBUG("[UW_DPL] TX config: interval=%us growth=%u%% max=%us count=%u\r\n",
			tx_cfg.tx_initial_interval_s, tx_cfg.tx_growth_percent,
			tx_cfg.tx_max_interval_s, tx_cfg.tx_max_count);
	}
}

uint8_t KNS_APP_uw_doppler_getDeployMode(void)
{
	return deploy_mode;
}

void KNS_APP_uw_doppler_setDeployMode(uint8_t mode)
{
	deploy_mode = mode ? 1 : 0;
	MGR_LOG_INFO("[UW_DPL] Deploy mode: %u\r\n", deploy_mode);
}

void KNS_APP_uw_doppler_setSessionMC(uint16_t mc)
{
	mc_retained.session_mc  = mc;
	mc_retained.magic       = MC_RETAINED_MAGIC;
	mc_retained.initialised = 1u;
	MGR_LOG_INFO("[UW_DPL] Session MC overridden by AT+MC = %u\r\n",
		(unsigned)mc);
}

uint8_t KNS_APP_uw_doppler_getStateRaw(void)
{
	return (uint8_t)uw_doppler_state;
}

uint32_t KNS_APP_uw_doppler_getTxCountSession(void)
{
	return tx_count;
}

KNS_APP_UwDopplerLbCfg_t KNS_APP_uw_doppler_getLbCfg(void)
{
	return lb_cfg;
}

void KNS_APP_uw_doppler_setLbCfg(const KNS_APP_UwDopplerLbCfg_t *cfg)
{
	if (!cfg) return;
	lb_cfg = *cfg;
	/* Force-reevaluate against the current battery level NOW so the
	 * YELLOW LB indicator fires immediately if applicable, not only on
	 * the next TX cycle. Without this, raising `enter_mV` above current
	 * battery via AT+LBCFG would leave lb_active=0 until next TX runs
	 * lb_update — confusing UX during live tuning. */
	lb_active = false;
#if defined(BSP_HAS_VBAT_ADC)
	{
		const uint16_t bat_mV = MGR_BAT_readVoltage_mV();
		(void)lb_update(bat_mV);
	}
#endif
	MGR_LOG_DEBUG("[UW_DPL] LB cfg: enter=%umV exit=%umV intvl=%us max=%us cnt=%u\r\n",
		lb_cfg.lb_enter_mV, lb_cfg.lb_exit_mV,
		lb_cfg.lb_tx_interval_s, lb_cfg.lb_tx_max_s, lb_cfg.lb_tx_max_count);
}

bool KNS_APP_uw_doppler_isLbActive(void)
{
	return lb_active;
}

uint8_t KNS_APP_uw_doppler_startTestBurst(uint8_t count)
{
	if (count == 0) count = 1;
	if (count > TEST_BURST_MAX_COUNT) count = TEST_BURST_MAX_COUNT;
	test_tx_remaining = count;
	MGR_LOG_INFO("[UW_DPL] Test burst started: %u TX queued\r\n", count);
	return count;
}

uint8_t KNS_APP_uw_doppler_getTestBurstRemaining(void)
{
	return test_tx_remaining;
}

#if defined(BSP_HAS_VBAT_ADC)
/* Update LB hysteretic state from a fresh battery reading. Returns the new
 * active state for convenience. lb_enter_mV=0 disables LB entirely. */
static bool lb_update(uint16_t bat_mV)
{
	if (lb_cfg.lb_enter_mV == 0) {
		lb_active = false;
		return false;
	}
	if (!lb_active && bat_mV > 0 && bat_mV < lb_cfg.lb_enter_mV) {
		lb_active = true;
		MGR_LOG_INFO("[UW_DPL] LB mode ENTER (%umV < %umV)\r\n",
			bat_mV, lb_cfg.lb_enter_mV);
		MGR_EVTLOG_log(EVT_LB_ENTER, bat_mV);
	} else if (lb_active && bat_mV > lb_cfg.lb_exit_mV) {
		lb_active = false;
		MGR_LOG_INFO("[UW_DPL] LB mode EXIT (%umV > %umV)\r\n",
			bat_mV, lb_cfg.lb_exit_mV);
		MGR_EVTLOG_log(EVT_LB_EXIT, bat_mV);
	}
	return lb_active;
}
#endif /* BSP_HAS_VBAT_ADC */

void KNS_APP_uw_doppler_restoreSwsBaselines(void)
{
	if (sws_retained.magic == SWS_RETAINED_MAGIC &&
	    sws_retained.air_baseline > 0 &&
	    sws_retained.water_baseline > sws_retained.air_baseline) {
		MGR_SWS_restoreBaselines(sws_retained.air_baseline,
		                         sws_retained.water_baseline);
		if (sws_retained.observed_peak_adc > 0)
			MGR_SWS_restoreObservedPeak(sws_retained.observed_peak_adc);
		MGR_LOG_DEBUG("[UW_DPL] Restored SWS: air=%u water=%u peak=%u\r\n",
			sws_retained.air_baseline, sws_retained.water_baseline,
			sws_retained.observed_peak_adc);
	}

	/* Instant first-TX path is enforced by MONITORING's standard
	 * `MGR_SWS_stateChanged()` hook + the wake_should_tx flag in
	 * MGR_LPM_UW retention: if the previous duty cycle persisted
	 * UNDERWATER and the SWS task now reports SURFACE on its first
	 * sample, the existing surface-detection branch primes
	 * surface_tx_pending naturally. No extra synchronous sample is
	 * needed here (would block init + risk an unstable analog read). */
}

/**
 * @}
 */
