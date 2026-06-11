/**
 * @file    mgr_lpm_uw.c
 * @brief   UW_DOPPLER-dedicated LPM manager — implementation.
 *
 * See mgr_lpm_uw.h for the API contract and rationale.
 *
 * Power-optimization principles encoded here:
 *  1. Deep STANDBY between meaningful events. Cold-boot on wake; the
 *     retention NOLOAD region (boot counter, SWS calib, crash forensics,
 *     this module's duty_cfg + last_sws_state) survives.
 *  2. Wake-context detection. On cold-boot, MGR_LPM_UW_init compares the
 *     last persisted SWS state against the freshly-sampled one. If the
 *     transition is UW → SURFACE we set `wake_should_tx_now = true` so
 *     the app fires the first TX on this wake instead of cycling through
 *     a standard MONITORING window.
 *  3. Configurable per-state sleep duration. Underwater turtles dive
 *     for tens of minutes; sleep aggressively (default 30 min) without
 *     compromising surface-session tracking (default 1 min).
 *  4. SHUTDOWN_REED for end-of-mission. Board powers off entirely.
 *     Only the HW reed magnet re-energises the regulator.
 */

#include "mgr_lpm_uw.h"

#include <stddef.h>
#include "stm32wlxx_hal.h"
#include "main.h"          /* PA_PSU_SEL_Pin, BSP defines */
#include "mgr_log.h"
#include "mgr_nvm.h"
#include "mgr_sws.h"       /* MGR_SWS_STATE_{SURFACE,UNDERWATER,UNKNOWN} */
#include "mcu_misc.h"      /* MCU_MISC_VSEL_set */
#include "lpm.h"           /* LPM_shutdownNow / LPM_shutdownWithAutoWake */
#include "rtc.h"
#include "adc.h"           /* MX_ADC_Init/DeInit — STOP2 entry/exit */
#if defined(BSP_HAS_LED_RGB)
#include "mgr_led.h"       /* MGR_LED_off before STOP2 */
#endif
#include "mgr_reed.h"      /* MGR_REED_releasePower + REED_MCU_Pin (stubs if no reed) */
#include "mgr_at_cmd.h"    /* MGR_AT_CMD_getLastActivityTick — console grace window */

/* Deep sleep is held off this long after the last decoded AT command so a
 * bench/commissioning session stays interactive. */
#define LPM_UW_AT_GRACE_MS  30000u
/* GPIO disable to analog for minimum STOP2 leakage. */
extern void GPIO_DisableAllToAnalogInput(void);
extern void MX_GPIO_Init(void);

/* Marker telling the next boot it was triggered by a magnet during soft-off,
 * so the wake feedback (green blink) plays even though RCC_CSR shows SFTRST
 * instead of a POR. Lives in TAMP backup: cleared on first consume and on
 * any backup-domain (VBAT) loss. */
#define SOFTOFF_WAKE_MAGIC  0x534F4657u  /* "SOFW" */

/* SubGHz radio teardown for minimum STOP2 leakage — the peripheral
 * itself draws several hundred µA when left armed. */
#include "subghz.h"
extern void MX_SUBGHZ_Init(void);

/* LPTIM1 is the wake source for the SLEEP tier — autonomous LSI-clocked
 * timer that keeps running through STOP1 and fires an IRQ at the
 * configured timeout, giving us sub-second precision (LSI ≈ 32 kHz ⇒
 * ~31 µs resolution, capped to ~2 s per pass by the 16-bit ARR). Only
 * compiled when the board has the PWR latch (real LPM path). */
#if defined(BSP_HAS_PWR_LATCH)
static LPTIM_HandleTypeDef s_lptim;
static volatile bool       s_lptim_fired = false;
#define LPTIM_LSI_HZ              32000u
#define LPTIM_MAX_MS_PER_PASS     2000u  /* 16-bit ARR / 32 ticks/ms */
static void enter_sleep_for_ms(uint32_t ms);
#endif
/* MGR_SWS enter/exitLowPower drop the SWS analog rail so it doesn't
 * burn current during STOP2. */
extern void MGR_SWS_enterLowPower(void);
extern void MGR_SWS_exitLowPower(void);

/* Drop VSEL to 1.8V right before STANDBY entry on STDALONE. BOR_LEV is
 * at level 0 (~1.7V) in production option bytes so VDD=1.8V keeps a
 * ~100 mV brownout margin. The TPS63901 takes a few ms to switch the
 * output rail; we add a settle delay before arming the WUF. On wake
 * gpio.c MX_GPIO_Init re-drives PC1 HIGH bringing VDD back to 3V3
 * before any radio/MAC work. Currently DISABLED while we diagnose
 * the immediate-wake bug from STANDBY (cycle observed ~22 ms regardless
 * of configured sleep_s — needs root cause before re-enabling). */
/* #define LPM_UW_STANDBY_LOW_VOLTAGE 1 */

#if defined(BSP_HAS_PWR_LATCH)

/* ---- Persisted retention-NOLOAD config block ---- */

/* Magic includes a struct-layout version byte: bump on any field add/move
 * so old retained state from prior firmware doesn't get reinterpreted as
 * garbage in newly-added fields (saw this: shutdown_threshold_s read as
 * 20308 after upgrading from a build without that field). */
#define DUTY_CFG_MAGIC 0x44555402UL  /* "DUTY" + version 02 */

typedef struct {
	uint32_t magic;
	uint16_t uw_sleep_s;
	uint16_t surf_sleep_s;
	uint8_t  enabled;
	uint8_t  last_sws_state;   /**< persisted SWS state across STANDBY */
	uint8_t  wake_should_tx;   /**< set if cold-boot detected UW→SURF */
	uint8_t  _pad;
	uint16_t shutdown_threshold_s;
} UwLpmDutyCfg_t;

/* NOLOAD: Sram2_Init does not touch this section, so it survives every
 * software-class reset (IWDG / SFT / OBL / BOR / PIN) and the STANDBY
 * cold-boot used by the auto-cycle. Cleared only by VBAT loss. */
static __attribute__((__section__(".retentionRamNoload")))
UwLpmDutyCfg_t s_duty;

/* Event-driven LPM thresholds. Replaces the old "always sleep for N seconds"
 * model with a "sleep until next scheduled task" approach. The thresholds
 * decide which depth of LPM is worth entering for the requested delay:
 *
 *   delta_ms < spin_ms              → just spin (continue main loop)
 *   spin_ms ≤ delta_ms < sleep_ms   → SLEEP (light, ~100 µA, wake instant)
 *   delta_ms ≥ sleep_ms             → STOP2 (deep, ~3 µA, ~50 ms wake cost)
 *
 * Set enabled=0 to disable LPM entirely (bench / commissioning at ~5 mA).
 * Persisted via the regular NVM save path. */
#define LPM_THR_MAGIC 0x4C504D01UL  /* "LPM" + version 01 */

typedef struct {
	uint32_t magic;
	uint16_t spin_ms;
	uint16_t sleep_ms;
	uint8_t  enabled;
	uint8_t  _pad[3];
} LpmThrCfg_t;

static __attribute__((__section__(".retentionRamNoload")))
LpmThrCfg_t s_lpm_thr;

static void s_apply_lpm_thr_defaults_if_needed(void)
{
	if (s_lpm_thr.magic != LPM_THR_MAGIC) {
		s_lpm_thr.magic    = LPM_THR_MAGIC;
		s_lpm_thr.spin_ms  = 10u;     /* below 10 ms LPM entry overhead wins */
		s_lpm_thr.sleep_ms = 500u;    /* 500 ms is the SLEEP↔STOP2 cross-over */
		s_lpm_thr.enabled  = 1u;      /* power-efficient by default */
	}
}

void MGR_LPM_UW_setLpmThr(uint16_t spin_ms, uint16_t sleep_ms, uint8_t enabled)
{
	s_apply_lpm_thr_defaults_if_needed();
	/* Clamp into sensible ranges; spin must be ≤ sleep. */
	if (sleep_ms < spin_ms) sleep_ms = spin_ms;
	s_lpm_thr.magic    = LPM_THR_MAGIC;
	s_lpm_thr.spin_ms  = spin_ms;
	s_lpm_thr.sleep_ms = sleep_ms;
	s_lpm_thr.enabled  = enabled ? 1u : 0u;
}

void MGR_LPM_UW_getLpmThr(uint16_t *spin_ms, uint16_t *sleep_ms,
                          uint8_t *enabled)
{
	s_apply_lpm_thr_defaults_if_needed();
	if (spin_ms)  *spin_ms  = s_lpm_thr.spin_ms;
	if (sleep_ms) *sleep_ms = s_lpm_thr.sleep_ms;
	if (enabled)  *enabled  = s_lpm_thr.enabled;
}

/* Interaction window on true cold-boot.
 *
 * PB6 reed is NOT a WL55 WKUP pin (only PA0/PC13/PB3 are) and EXTI is
 * gated during STANDBY, so a reed magnet event is invisible to the chip
 * once we are in STANDBY — magnet detection latency in steady state is
 * up to the current sleep duration (surf_sleep_s / uw_sleep_s).
 *
 * The exception is right after a true cold-boot (POR / deploy / recovery):
 * the operator is typically the one who powered the tag on and may want
 * to interact with a magnet during the first seconds. Hold the auto-cycle
 * for 30 s on this very first MONITORING entry so EXTI on PB6 stays armed.
 * STANDBY re-wakes skip this window (the device is sealed underwater,
 * nobody is around to apply a magnet). */
#define DUTY_STABILIZE_MS  30000u
static uint32_t s_first_monitoring_tick = 0u;
static bool     s_monitoring_ever_entered = false;

static void s_apply_defaults_if_needed(void)
{
	if (s_duty.magic != DUTY_CFG_MAGIC) {
		s_duty.magic                = DUTY_CFG_MAGIC;
		/* uw_sleep_s / surf_sleep_s are now LEGACY: kept in the struct
		 * so the AT+DUTYCFG echo doesn't regress the GUI parser, but
		 * tryAutoCycle no longer reads them — sleep duration is derived
		 * from the active SWS interval (the natural sampling cadence). */
		s_duty.uw_sleep_s           = 0u;
		s_duty.surf_sleep_s         = 0u;
		s_duty.enabled              = 1u;      /* on by default — LPM is the
		                                        * point of UW_DOPPLER */
		s_duty.last_sws_state       = 0xFFu;   /* "unknown" */
		s_duty.wake_should_tx       = 0u;
		s_duty.shutdown_threshold_s = 300u;    /* default: ≥5min → SHUTDOWN */
	}
}

/* ---- Init ---- */

void MGR_LPM_UW_init(void)
{
	s_apply_defaults_if_needed();
}

/* ---- Auto-cycle policy ---- */

void MGR_LPM_UW_markMonitoringEntered(void)
{
	if (!s_monitoring_ever_entered) {
		s_first_monitoring_tick = HAL_GetTick();
		s_monitoring_ever_entered = true;
	}
}

void MGR_LPM_UW_setDutyCfg(uint16_t uw_s, uint16_t surf_s, uint8_t enabled)
{
	if (uw_s < 1u)   uw_s   = 1u;
	if (surf_s < 1u) surf_s = 1u;
	s_duty.magic        = DUTY_CFG_MAGIC;
	s_duty.uw_sleep_s   = uw_s;
	s_duty.surf_sleep_s = surf_s;
	s_duty.enabled      = enabled ? 1u : 0u;
	MGR_LOG_INFO("[LPM_UW] DUTYCFG uw=%us surf=%us en=%u\r\n",
		uw_s, surf_s, s_duty.enabled);
}

void MGR_LPM_UW_getDutyCfg(uint16_t *uw_s, uint16_t *surf_s, uint8_t *enabled)
{
	if (uw_s)    *uw_s    = s_duty.uw_sleep_s;
	if (surf_s)  *surf_s  = s_duty.surf_sleep_s;
	if (enabled) *enabled = s_duty.enabled;
}

void MGR_LPM_UW_setShutdownThreshold(uint16_t seconds)
{
	s_duty.magic               = DUTY_CFG_MAGIC;
	s_duty.shutdown_threshold_s = seconds;
	MGR_LOG_INFO("[LPM_UW] SHUTDOWN threshold=%us\r\n", (unsigned)seconds);
}

uint16_t MGR_LPM_UW_getShutdownThreshold(void)
{
	return s_duty.shutdown_threshold_s;
}

bool MGR_LPM_UW_detectSurfaceWake(int current_sws_state)
{
	const uint8_t prev = s_duty.last_sws_state;
	const uint8_t curr = (uint8_t)current_sws_state;
	bool transition_uw_to_surf = false;

	if (prev == (uint8_t)MGR_SWS_STATE_UNDERWATER &&
	    curr == (uint8_t)MGR_SWS_STATE_SURFACE) {
		/* Previous cycle persisted UNDERWATER, current sample says
		 * SURFACE → first surface event since last wake. Fire TX. */
		transition_uw_to_surf = true;
		s_duty.wake_should_tx = 1u;
	}
	/* Update persisted state only on known transitions, so an UNKNOWN
	 * sample on cold-boot (SWS task not yet ready) doesn't wipe the
	 * UW/SURF baseline from the previous cycle. */
	if (curr == (uint8_t)MGR_SWS_STATE_SURFACE ||
	    curr == (uint8_t)MGR_SWS_STATE_UNDERWATER) {
		s_duty.last_sws_state = curr;
	}
	return transition_uw_to_surf;
}

bool MGR_LPM_UW_isWakeShouldTx(void)
{
	return s_duty.wake_should_tx != 0u;
}

void MGR_LPM_UW_clearWakeShouldTx(void)
{
	s_duty.wake_should_tx = 0u;
}

extern uint32_t g_boot_pwr_extscr_raw;

bool MGR_LPM_UW_isWakeFromStandby(void)
{
	/* PWR_EXTSCR.C1SBF (bit 8) — STANDBY flag for CPU1 on WL55.
	 * Sticky across the cold-boot, cleared by HAL in the wake-up switch;
	 * we snapshot EXTSCR before that happens. */
	return (g_boot_pwr_extscr_raw & PWR_EXTSCR_C1SBF) != 0u;
}

/* Event-driven LPM scheduler. Called from the main app loop with the time
 * until the next scheduled task (typically the next SWS sample, or the
 * next TX in a burst). Decides whether to spin, SLEEP or STOP2 based on
 * the LpmThr thresholds.
 *
 *   delta_ms < spin_ms              → spin (continue main loop)
 *   spin_ms ≤ delta_ms < sleep_ms   → SLEEP (light, instant wake)
 *   delta_ms ≥ sleep_ms             → STOP2 (deep, ~50 ms wake cost)
 *
 * SLEEP mode is implemented as a STOP2 call with a short RTC window —
 * the WL55 RTC CK_SPRE supports 1 Hz minimum so anything below 1 s is
 * rounded up. Until a proper LPTIM-based SLEEP path is wired, the
 * sleep-tier and stop-tier are functionally the same; only the threshold
 * differs. The spin-tier IS distinct and saves the STOP2 entry/exit cost
 * on very short waits. */
void MGR_LPM_UW_idleTick(int sws_state, uint32_t delta_ms,
                         bool gesture_busy, bool config_mode)
{
	s_apply_lpm_thr_defaults_if_needed();

	if (!s_lpm_thr.enabled)
		return;
	if (!s_duty.enabled)            /* legacy DUTYCFG master kill switch */
		return;
	if (gesture_busy || config_mode)
		return;
	if (!s_monitoring_ever_entered)
		return;
	if (MGR_REED_isDebouncing())
		return;

	/* Console grace window: someone is actively sending AT commands —
	 * hold off deep sleep so the dialogue stays interactive. In STOP2 the
	 * LPUART has no kernel clock (115200 needs HSI16) and the board is
	 * deaf except for a ~50 ms window per wake; measured on the bench at
	 * ~1 command landed per 40 attempts. Sealed deployments have no AT
	 * traffic, so this gate never delays sleep in the field. */
	{
		const uint32_t last_at = MGR_AT_CMD_getLastActivityTick();
		if (last_at != 0u &&
		    (HAL_GetTick() - last_at) < LPM_UW_AT_GRACE_MS)
			return;
	}

	/* Stabilization gate. Full 30 s on true cold-boot to give the
	 * operator a UART window for AT commands. On wake-from-STANDBY
	 * the previous cycle has proven the device is healthy, the box
	 * is sealed and no operator is interacting — skip the wait. */
	const uint32_t stabilize_ms = MGR_LPM_UW_isWakeFromStandby()
	                              ? 0u
	                              : DUTY_STABILIZE_MS;
	if ((HAL_GetTick() - s_first_monitoring_tick) < stabilize_ms)
		return;

	/* Persist the SWS state so the next wake's init can detect a
	 * UW→SURFACE transition and fire TX immediately on cold-boot. */
	if (sws_state == (int)MGR_SWS_STATE_SURFACE ||
	    sws_state == (int)MGR_SWS_STATE_UNDERWATER) {
		s_duty.last_sws_state = (uint8_t)sws_state;
	}
	s_duty.wake_should_tx = 0u;

	(void)s_duty.shutdown_threshold_s;  /* unused by event-driven path */

	if (delta_ms < s_lpm_thr.spin_ms) {
		/* Too short for LPM entry overhead. Spin. */
		return;
	}

	if (delta_ms < s_lpm_thr.sleep_ms) {
		/* SLEEP tier: STOP1 + LPTIM with sub-second precision. Cap to
		 * the LPTIM 16-bit ARR limit; longer SLEEP requests downgrade
		 * automatically to STOP2 below. */
		if (delta_ms <= LPTIM_MAX_MS_PER_PASS) {
			enter_sleep_for_ms(delta_ms);
			return;
		}
		/* Fall through to STOP2 for the rare case where sleep_ms is
		 * configured above the LPTIM range. */
	}

	/* STOP2 tier: deadline-exact wake (RTCCLK/16 below 29 s, floored
	 * CK_SPRE seconds above). The chip sleeps until the next scheduled
	 * action — never past it; a sub-tier remainder is handled by the
	 * next idleTick pass after the wake. */
	MGR_LPM_UW_enterStop2TimedMs(delta_ms);
	/* Returns here on wake. State machine continues normally. */
}

/* Legacy entry point. Kept so older call sites still compile during the
 * migration; auto-derives the sleep duration from the active SWS interval
 * and forwards to the event-driven scheduler. */
void MGR_LPM_UW_tryAutoCycle(int sws_state, bool gesture_busy, bool config_mode)
{
	uint32_t delta_ms;
	if (sws_state == (int)MGR_SWS_STATE_SURFACE)
		delta_ms = MGR_SWS_getSurfIntervalMs();
	else
		delta_ms = MGR_SWS_getUWIntervalMs();
	MGR_LPM_UW_idleTick(sws_state, delta_ms, gesture_busy, config_mode);
}

/* ---- STANDBY timed wake ---- */

void MGR_LPM_UW_enterStandbyTimed(uint32_t seconds)
{
	MGR_LOG_INFO("[LPM_UW] STANDBY %lus\r\n", (unsigned long)seconds);

	/* NO MGR_NVM_save() here — calling it every cycle would burn the
	 * flash within weeks at the deployment cadence and the 50 ms write
	 * stalls the loop on each iteration. duty_cfg + sws calibration +
	 * boot counter all live in .retentionRamNoload which survives the
	 * STANDBY cold-boot without any flash interaction. NVM is committed
	 * on AT+SAVE / boot-loop-factory-reset only. */

	if (seconds < 1u)     seconds = 1u;
	if (seconds > 0xFFFFu) seconds = 0xFFFFu;

	/* Arm RTC alarm. CK_SPRE 16-bit: 1..65 536 s per cycle. */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
	(void)HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
	    (uint16_t)(seconds - 1u),
	    RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);

	/* Anchor PWR_LATCH (PB7) HIGH via PWR controller so the STDALONE
	 * regulator stays alive through STANDBY. Without this the GPIO
	 * loses drive on STANDBY entry, the board powers off, and only
	 * the HW reed circuit can wake. */
	HAL_PWREx_EnablePullUpPullDownConfig();
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, PWR_GPIO_BIT_7);

#if defined(LPM_UW_STANDBY_LOW_VOLTAGE)
	/* Drop VSEL LOW so TPS63901 outputs 1.8V during STANDBY (~half the
	 * leakage of the 3V3 rail). VDD must stay above the ~1.7V BOR
	 * threshold; with a 1.8V regulator setpoint we have ~100 mV margin.
	 * Drive the pin LOW first so the rail transitions while the chip is
	 * still actively powered, then hand off to the PWR controller's
	 * pull-down to hold it during STANDBY. */
	MCU_MISC_VSEL_set(false);
	/* Settle: TPS63901 takes ~1-2 ms to switch its internal feedback
	 * divider. Add margin so we are firmly at 1.8V before STANDBY. */
	HAL_Delay(3);
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PA_PSU_SEL_Pin);
#else
	/* Hold VSEL HIGH so TPS63901 stays in 3V3 mode on wake. */
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_C, PA_PSU_SEL_Pin);
#endif

	HAL_PWREx_EnableSRAMRetention();

	HAL_PWREx_DisableInternalWakeUpLine();
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);
	HAL_PWREx_EnableInternalWakeUpLine();

	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	__HAL_RCC_CLEAR_RESET_FLAGS();

	HAL_PWR_EnterSTANDBYMode();
	for (;;) { /* unreachable */ }
}

/* ---- Power-off with reed-magnet wake ---- */

void MGR_LPM_UW_enterShutdownReed(void)
{
	MGR_LOG_INFO("[LPM_UW] POWER_OFF: latch release, magnet-only wake\r\n");
	HAL_Delay(20);  /* drain the UART TX FIFO before teardown */

	/* This mode ends ONLY on a magnet event — disarm every scheduled
	 * wake the RTC could deliver. */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
	(void)HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
	(void)HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);
	HAL_PWREx_DisableInternalWakeUpLine();
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);

	/* Same peripheral-teardown floor as enterStop2Timed. */
#if defined(BSP_HAS_LED_RGB)
	MGR_LED_off();
#endif
	MGR_SWS_enterLowPower();
	MX_ADC_DeInit();
#if defined(BSP_HAS_VBAT_ADC)
	HAL_GPIO_WritePin(VBAT_EN_GPIO_Port, VBAT_EN_Pin, GPIO_PIN_RESET);
#endif
	GPIO_DisableAllToAnalogInput();
	(void)HAL_SUBGHZ_DeInit(&hsubghz);

	/* UART RX must not wake the soft-off: magnet only. */
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
	HAL_NVIC_DisableIRQ(LPUART1_IRQn);

	/* Cut our own power: PB7 LOW, held through phase 1 (STOP2 retains
	 * GPIO output drive). While the confirm magnet is still on the reed,
	 * the external reed/PWR_LATCH OR-gate keeps the regulator alive no
	 * matter what PB7 says — the actual collapse happens at magnet
	 * removal. PB6 EXTI is wake-capable in STOP2 (the EXTI block stays
	 * powered), unlike in SHUTDOWN/STANDBY where only the dedicated
	 * WKUP pins PA0/PC13/PB3 have wake circuitry. */
	MGR_REED_releasePower();
	HAL_SuspendTick();

	/* Phase 1 — wait in STOP2 for the confirm magnet to be removed.
	 * Skipped instantly when no magnet is present (AT+SHUTDOWN path). */
	while (HAL_GPIO_ReadPin(REED_MCU_GPIO_Port, REED_MCU_Pin)
	       == GPIO_PIN_SET) {
		__HAL_GPIO_EXTI_CLEAR_IT(REED_MCU_Pin);
		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
	}

	/* Magnet gone + PB7 LOW: on battery the regulator collapses during
	 * this window → true power-off at the HW quiescent floor; the next
	 * boot is the reed re-latch POR. */
	HAL_ResumeTick();
	HAL_Delay(150);

	/* Still alive → VDD is fed from elsewhere (bench USB/JLink backfeed)
	 * or the regulator stayed up. Stop fighting the external latch
	 * pull-up (VDD-through-Rpull into a LOW pin is a permanent ~25 µA
	 * burn): give PB7 back to analog and purge any PWR-domain pull-down
	 * a previous firmware left armed across resets. */
	HAL_GPIO_DeInit(PWR_LATCH_GPIO_Port, PWR_LATCH_Pin);
	(void)HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_7);

#if defined(BSP_REED_ON_WKUP3)
	/* Reed wired to PB3 = WKUP3: true SHUTDOWN, wakeable by the magnet
	 * through the dedicated WKUP circuitry (sub-µA MCU floor — LSE/RTC
	 * and all clocks die with the core). Wake = cold-boot reset, the
	 * gesture init forces OPERATIONAL. PWR pull-down keeps PB3 from
	 * floating once GPIOs power off, so the magnet's rising edge is
	 * detected cleanly. */
	HAL_PWREx_EnablePullUpPullDownConfig();
	(void)HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_3);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN3);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_HIGH);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	__HAL_RCC_CLEAR_RESET_FLAGS();
	/* Marker set BEFORE entry: SHUTDOWN exit does not reliably raise a
	 * cold-boot RCC flag, and any exit from this state (magnet or NRST)
	 * legitimately deserves the wake feedback. */
	TAMP->BKP11R = SOFTOFF_WAKE_MAGIC;
	HAL_PWREx_EnterSHUTDOWNMode();
	for (;;) { /* unreachable — SHUTDOWN exit is a reset */ }
#else
	/* Soft-off in STOP2, magnet-only wake. */
	HAL_SuspendTick();

	/* Phase 2 — any NEW magnet application restarts the board. */
	for (;;) {
		if (HAL_GPIO_ReadPin(REED_MCU_GPIO_Port, REED_MCU_Pin)
		    == GPIO_PIN_SET) {
			/* Magnet present: restart as if power-cycled. Gesture
			 * boot path always forces OPERATIONAL mode. */
			TAMP->BKP11R = SOFTOFF_WAKE_MAGIC;
			NVIC_SystemReset();
		}
		__HAL_GPIO_EXTI_CLEAR_IT(REED_MCU_Pin);
		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
	}
#endif
}

void MGR_LPM_UW_enterShutdownAutoWake(uint32_t wakeup_seconds)
{
	MGR_LOG_INFO("[LPM_UW] SHUTDOWN auto-wake %lus\r\n",
		(unsigned long)wakeup_seconds);
	LPM_shutdownWithAutoWake(wakeup_seconds);
	for (;;) { /* unreachable */ }
}

/* ---- SLEEP tier: STOP1 + LPTIM sub-second wake ---- */

/* LSI nominal frequency. WL55 datasheet quotes ±50 % tolerance; that
 * jitter is fine for the 10–500 ms range — the operator's perception
 * threshold and the SWS state-change cadence are both orders of
 * magnitude larger than the worst-case error. */

static bool s_lptim_initialised = false;

static void lptim_init_once(void)
{
	if (s_lptim_initialised)
		return;

	/* Route LSI to LPTIM1. LSI is already enabled by the IWDG path and
	 * stays on across STOP modes, so we only need to select it as the
	 * peripheral clock source. */
	RCC_PeriphCLKInitTypeDef clk = {0};
	clk.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
	clk.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
	(void)HAL_RCCEx_PeriphCLKConfig(&clk);
	__HAL_RCC_LPTIM1_CLK_ENABLE();

	s_lptim.Instance                  = LPTIM1;
	s_lptim.Init.Clock.Source         = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	s_lptim.Init.Clock.Prescaler      = LPTIM_PRESCALER_DIV1;
	s_lptim.Init.Trigger.Source       = LPTIM_TRIGSOURCE_SOFTWARE;
	s_lptim.Init.OutputPolarity       = LPTIM_OUTPUTPOLARITY_HIGH;
	s_lptim.Init.UpdateMode           = LPTIM_UPDATE_IMMEDIATE;
	s_lptim.Init.CounterSource        = LPTIM_COUNTERSOURCE_INTERNAL;
	s_lptim.Init.Input1Source         = LPTIM_INPUT1SOURCE_GPIO;
	s_lptim.Init.Input2Source         = LPTIM_INPUT2SOURCE_GPIO;
	(void)HAL_LPTIM_Init(&s_lptim);

	HAL_NVIC_SetPriority(LPTIM1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(LPTIM1_IRQn);

	s_lptim_initialised = true;
}

/* Match callback — overrides HAL's __weak default. We don't actually
 * need to do anything except clear the sentinel because the IRQ itself
 * is what wakes the chip from STOP1. */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	(void)hlptim;
	s_lptim_fired = true;
}

void LPTIM1_IRQHandler(void)
{
	HAL_LPTIM_IRQHandler(&s_lptim);
}

/* SLEEP tier entry. STOP1 (lighter than STOP2 — peripherals partly
 * alive, ~5 µs wake) gated by LPTIM IRQ. Returns after the timeout
 * fires or any other EXTI / NVIC source wakes the chip. Used for the
 * spin_ms..sleep_ms tier where sub-second precision matters. */
static void enter_sleep_for_ms(uint32_t ms)
{
	if (ms == 0u)
		return;
	if (ms > LPTIM_MAX_MS_PER_PASS)
		ms = LPTIM_MAX_MS_PER_PASS;

	lptim_init_once();

	/* LSI ≈ 32 kHz ⇒ 32 ticks per millisecond. The 16-bit ARR caps the
	 * pass at ~2 s; any caller asking for more should use STOP2 instead. */
	uint32_t ticks = ms * (LPTIM_LSI_HZ / 1000u);
	if (ticks == 0u) ticks = 1u;
	if (ticks > 0xFFFFu) ticks = 0xFFFFu;

	s_lptim_fired = false;
	(void)HAL_LPTIM_TimeOut_Start_IT(&s_lptim, 0xFFFFu, (uint32_t)ticks);

#if defined(BSP_HAS_LED_RGB)
	MGR_LED_off();
#endif
	MGR_SWS_enterLowPower();

	HAL_SuspendTick();
	LPM_saveRtcTime();
	HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
	HAL_ResumeTick();
	/* Credit the real time slept to HAL_GetTick (SysTick was dead). */
	LPM_compensateTick();

	(void)HAL_LPTIM_TimeOut_Stop_IT(&s_lptim);
	MGR_SWS_exitLowPower();
	MGR_SWS_forceMeasurement();
}

/* ---- STOP2 timed wake (production duty-cycle path) ---- */

/* Duplicate of lpm.c:LPM_SystemClock_Config_RestoreFromStop (static there).
 * After STOP2 the SoC clock is MSI; we need to restore HSI + PLL to keep
 * peripheral baud rates / TX timeouts consistent with production. */
static void uw_restore_clock_from_stop(void)
{
	RCC_OscInitTypeDef osc = {0};
	RCC_ClkInitTypeDef clk = {0};
	uint32_t flat = 0;

	HAL_PWR_EnableBkUpAccess();

	HAL_RCC_GetOscConfig(&osc);
	osc.OscillatorType       = RCC_OSCILLATORTYPE_HSI;
	osc.HSICalibrationValue  = RCC_HSICALIBRATION_DEFAULT;
	osc.HSIState             = RCC_HSI_ON;
	osc.PLL.PLLState         = RCC_PLL_ON;
	osc.PLL.PLLSource        = RCC_PLLSOURCE_HSI;
	(void)HAL_RCC_OscConfig(&osc);

	HAL_RCC_GetClockConfig(&clk, &flat);
	clk.ClockType    = RCC_CLOCKTYPE_SYSCLK;
	clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	(void)HAL_RCC_ClockConfig(&clk, flat);
}

void MGR_LPM_UW_enterStop2TimedMs(uint32_t ms)
{
	MGR_LOG_INFO("[LPM_UW] STOP2 %lums\r\n", (unsigned long)ms);

	if (ms > 65535000u) ms = 65535000u;

	/* Arm RTC wake-up timer if a non-zero interval was requested. With
	 * ms=0 only EXTI sources (reed, gesture) can wake.
	 *
	 * Two clockings — both chosen so the scheduler NEVER oversleeps a
	 * deadline (any remainder is handled by the next idleTick pass):
	 *  - < 29 s: RTCCLK/16 = 2048 Hz, ~0.49 ms steps. Exact-ms wake so a
	 *    TX slot mid-sequence fires on time instead of "next multiple of
	 *    the SWS period" (the old fixed-period behaviour).
	 *  - >= 29 s: CK_SPRE 1 Hz, duration FLOORED to whole seconds. */
	if (ms > 0u) {
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
		__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
		if (ms < 29000u) {
			uint32_t ticks = (ms * 2048u) / 1000u;
			if (ticks == 0u)
				ticks = 1u;
			(void)HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
			    (uint16_t)(ticks - 1u),
			    RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0);
		} else {
			uint32_t seconds = ms / 1000u;
			(void)HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
			    (uint16_t)(seconds - 1u),
			    RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
		}
	}

	/* Internal wake-up line routes RTC events to the PWR wake-up logic.
	 * Without this RTC fires its IRQ but the chip doesn't exit STOP2. */
	HAL_PWREx_DisableInternalWakeUpLine();
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);
	HAL_PWREx_EnableInternalWakeUpLine();

	/* Peripheral teardown — drops the SWS analog rail + LED + ADC before
	 * STOP2. Without these the chip stays at several mA from always-on
	 * peripheral biases. Originally suspected of crashing the STOP2
	 * path but the real cause was IWDG running in STOP (now fixed via
	 * OPTR.IWDG_STOP = 0 = FREEZE). */
#if defined(BSP_HAS_LED_RGB)
	MGR_LED_off();
#endif
	MGR_SWS_enterLowPower();
	/* De-init ADC peripheral — leaving it armed costs ~µA. Re-init on
	 * wake restores it cleanly. */
	MX_ADC_DeInit();
	/* Force VBAT measurement enable LOW — when HIGH the external divider
	 * pulls ~30 µA continuously through the 120k+300k resistor chain. */
#if defined(BSP_HAS_VBAT_ADC)
	HAL_GPIO_WritePin(VBAT_EN_GPIO_Port, VBAT_EN_Pin, GPIO_PIN_RESET);
#endif
	/* Set all unused GPIOs to analog (no pull) to kill the long-tail
	 * leakage that kept STOP2 at ~630 µA on SMD_STDALONE. The Kineis
	 * helper carefully preserves UART (PA2/3), SWD (PA13/14), SWS
	 * (PA11/12), reed (PB6), PWR_LATCH (PB7), VBAT_EN (PB9), LEDs
	 * (PB4/5/PA1), and PA control (PC0/1). After wake we re-run
	 * MX_GPIO_Init to put everything back into its production state. */
	GPIO_DisableAllToAnalogInput();

	/* De-init SubGHz radio peripheral. The radio's internal circuitry
	 * keeps ~500 µA flowing even when idle if the peripheral is left
	 * armed. HAL_SUBGHZ_DeInit gates the peripheral clock and puts the
	 * radio in its reset state. The MAC stack picks up the radio on
	 * its next TX request, which won't happen until the chip is back
	 * in MONITORING and our wake path has re-run MX_SUBGHZ_Init. */
	(void)HAL_SUBGHZ_DeInit(&hsubghz);

	/* PB6 EXTI is intentionally LEFT ARMED through STOP2 so a magnet edge
	 * during sleep wakes the chip and the operator gets prompt gesture
	 * feedback (the LED has to come on within the human-perception
	 * window, ~100 ms). Earlier rev tried masking it when the Hall was
	 * stuck HIGH at entry to dodge ~28 000 ISR/s on the bench — but that
	 * also killed all magnet wakes, so a new tap during STOP2 produced no
	 * visible response until the next RTC wake. Bench operators with the
	 * magnet nearby will see the chip stuck at MONITORING current
	 * (~5 mA) because the ISR keeps re-waking it; this is a commissioning
	 * scenario only. In deployment there is no magnet → no ISR storm →
	 * STOP2 reaches its µA-floor normally. */

	/* SysTick gets disabled during STOP (no HCLK), then re-enabled on
	 * wake. HAL_SuspendTick avoids spurious tick interrupts wedging WFI. */
	HAL_SuspendTick();
	LPM_saveRtcTime();

	/* Enter STOP2. WFI returns here once a configured wake source fires
	 * (RTC alarm, or any pending EXTI — including reed PB6 which the
	 * MGR_REED driver sets up at boot with rising+falling edge IT). */
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

	/* === Awake again ===
	 *
	 * Order matters: HAL_RCC_* calls and HAL_ADC_Init use HAL_GetTick
	 * for their internal timeout polling. If SysTick is suspended at
	 * this point, HAL_GetTick stays frozen → every wait loop falls
	 * through immediately → Error_Handler() infinite loop → IWDG fires
	 * at ~16 s → cold-boot. Observed exactly that pattern when a magnet
	 * woke STOP2 (chip cold-booted 18 s after the AT command).
	 *
	 * Resume tick FIRST so the rest of the restore can use timed HAL. */
	HAL_ResumeTick();

	/* Add the RTC-measured sleep duration to HAL_GetTick. Without this
	 * every tick-based timer (TX schedule, seq-restart, SWS dive
	 * watchdog, AT grace) counts CPU-active time only and stretches by
	 * the total time slept — a 10 s TX interval interleaved with STOP2
	 * would take hundreds of wall-clock seconds to "elapse". */
	LPM_compensateTick();

	/* Clear the PB6 EXTI pending flag so a stale magnet edge captured
	 * while we were still in the wake-up critical path doesn't fire a
	 * phantom IRQ now that the NVIC is back to normal priority. The line
	 * stays unmasked at all times (see entry-side comment). */
	__HAL_GPIO_EXTI_CLEAR_IT(REED_MCU_Pin);

	/* Disarm the RTC wake timer so it doesn't fire again mid-MONITORING. */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

	/* On STOP exit the chip is on MSI; restore the production PLL config
	 * so peripheral baud rates / TX timeouts behave the same as before. */
	uw_restore_clock_from_stop();

	/* Re-init GPIOs (we set most to analog before STOP2 entry). */
	MX_GPIO_Init();

	/* Re-init SubGHz radio so the MAC can use it again. */
	MX_SUBGHZ_Init();

	/* Re-init ADC: STOP2 deinitialises the peripheral, and without this
	 * SWS reads return 0 for the whole post-wake cycle (observed). */
	MX_ADC_Init();

	/* SWS exit-LP mirrors the enterLowPower call, re-arms the analog rail. */
	MGR_SWS_exitLowPower();

	/* Force a fresh SWS sample. STOP2 is so short that the periodic
	 * MGR_SWS_task interval (1 Hz at surface, 2 Hz UW) never fires
	 * between successive wakes — the cached state would stay stale
	 * for the whole deployment. Force one sample per wake so the
	 * auto-cycle decision uses fresh data and surface transitions
	 * are caught at the next wake instead of after a full sleep_s. */
	MGR_SWS_forceMeasurement();

	/* No HAL_UART_TX in this function — log only on caller side once we're
	 * back in the main loop, otherwise the BAUD lock-up from waking still
	 * partially initialized UART hardware can stall the print. */
}

void MGR_LPM_UW_enterStop2Timed(uint32_t seconds)
{
	if (seconds > 65535u)
		seconds = 65535u;
	MGR_LPM_UW_enterStop2TimedMs(seconds * 1000u);
}

#else /* !BSP_HAS_PWR_LATCH */

/* Without PWR_LATCH we can't keep the regulator alive through STANDBY,
 * so the timed-wake path is unavailable. The SHUTDOWN path still works
 * but with magnet-only wake. */

void MGR_LPM_UW_init(void) {}
void MGR_LPM_UW_markMonitoringEntered(void) {}
void MGR_LPM_UW_setDutyCfg(uint16_t a, uint16_t b, uint8_t c)
    { (void)a; (void)b; (void)c; }
void MGR_LPM_UW_getDutyCfg(uint16_t *a, uint16_t *b, uint8_t *c)
{
	if (a) *a = 0;
	if (b) *b = 0;
	if (c) *c = 0;
}
void MGR_LPM_UW_setShutdownThreshold(uint16_t s) { (void)s; }
uint16_t MGR_LPM_UW_getShutdownThreshold(void) { return 0; }
void MGR_LPM_UW_tryAutoCycle(int s, bool g, bool cm)
    { (void)s; (void)g; (void)cm; }
bool MGR_LPM_UW_detectSurfaceWake(int s) { (void)s; return false; }
bool MGR_LPM_UW_isWakeShouldTx(void) { return false; }
void MGR_LPM_UW_clearWakeShouldTx(void) {}
bool MGR_LPM_UW_isWakeFromStandby(void) { return false; }

__attribute__((noreturn))
void MGR_LPM_UW_enterStandbyTimed(uint32_t s) { (void)s; for (;;) {} }

__attribute__((noreturn))
void MGR_LPM_UW_enterShutdownReed(void) { LPM_shutdownNow(); for (;;) {} }

__attribute__((noreturn))
void MGR_LPM_UW_enterShutdownAutoWake(uint32_t s)
    { LPM_shutdownWithAutoWake(s); for (;;) {} }

void MGR_LPM_UW_enterStop2Timed(uint32_t s) { (void)s; }
void MGR_LPM_UW_enterStop2TimedMs(uint32_t ms) { (void)ms; }

#endif /* BSP_HAS_PWR_LATCH */

bool MGR_LPM_UW_consumeSoftOffWake(void)
{
	if (TAMP->BKP11R != SOFTOFF_WAKE_MAGIC)
		return false;
	TAMP->BKP11R = 0u;
	return true;
}
