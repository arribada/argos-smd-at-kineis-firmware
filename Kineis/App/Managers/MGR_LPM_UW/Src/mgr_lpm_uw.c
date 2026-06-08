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
		s_duty.uw_sleep_s           = 1800u;   /* 30 min underwater */
		s_duty.surf_sleep_s         = 60u;     /* 1  min surface idle */
		s_duty.enabled              = 0u;      /* opt-in via AT+DUTYCFG */
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
	MGR_LOG_DEBUG("[LPM_UW] DUTYCFG uw=%us surf=%us en=%u\r\n",
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
	MGR_LOG_DEBUG("[LPM_UW] SHUTDOWN threshold=%us\r\n", (unsigned)seconds);
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

/** Threshold below which dropping to STANDBY is not worth it (cold-boot
 *  overhead + NVM save costs more than just staying awake). */
#define LPM_UW_SHORT_SLEEP_THRESHOLD_S  5u

void MGR_LPM_UW_tryAutoCycle(int sws_state, bool gesture_busy, bool config_mode)
{
	if (!s_duty.enabled)
		return;
	if (gesture_busy || config_mode)
		return;
	if (!s_monitoring_ever_entered)
		return;
	/* Stabilization gate. Full 5s on true cold-boot to give the user a
	 * UART window for AT commands. On wake-from-STANDBY the previous
	 * cycle has proven the device is healthy, the box is sealed and no
	 * operator is interacting — skip the wait and re-enter sleep ASAP. */
	const uint32_t stabilize_ms = MGR_LPM_UW_isWakeFromStandby()
	                              ? 0u
	                              : DUTY_STABILIZE_MS;
	if ((HAL_GetTick() - s_first_monitoring_tick) < stabilize_ms)
		return;

	/* Persist the SWS state so the next wake's init can detect
	 * a UW→SURFACE transition and fire TX immediately on cold-boot. */
	if (sws_state == (int)MGR_SWS_STATE_SURFACE ||
	    sws_state == (int)MGR_SWS_STATE_UNDERWATER) {
		s_duty.last_sws_state = (uint8_t)sws_state;
	}
	s_duty.wake_should_tx = 0u;

	/* MGR_SWS_State_t: UNKNOWN=0, SURFACE=1, UNDERWATER=2.
	 *
	 * Sleep selection for sealed 12-month deployment: any non-SURFACE
	 * state (UNDERWATER or UNKNOWN/sensor-fault) MUST pick the long
	 * underwater interval. The previous behaviour treated UNKNOWN as
	 * SURFACE, which on a corroded / broken sensor would put the tag
	 * into a permanent short-cycle and exhaust the battery in weeks
	 * instead of months. We'd rather miss a TX window than burn the
	 * battery — the device will still wake periodically and re-attempt
	 * SWS calibration each cycle (EMA recalibration in MGR_SWS). */
	const uint32_t sleep_s = (sws_state == (int)MGR_SWS_STATE_SURFACE)
	                          ? s_duty.surf_sleep_s
	                          : s_duty.uw_sleep_s;

	/* Below threshold: cold-boot overhead > sleep saving — skip. */
	if (sleep_s < LPM_UW_SHORT_SLEEP_THRESHOLD_S)
		return;

	/* Production duty-cycle path: STOP2 (RAM + MAC + peripherals retained).
	 *
	 * Why not STANDBY: STANDBY cold-boots the chip on wake and re-inits the
	 * MAC stack (~6 s × ~10 mA = 0.017 mAh per wake). For a 30 min cycle
	 * that's already ~50% of the duty-cycle current. Worse, STANDBY can't
	 * wake on the reed switch (PB6 isn't a WL55 WKUP pin) so a magnet
	 * event during deep sleep is invisible for up to sleep_s seconds.
	 *
	 * STOP2 keeps the same ~2 µA sleep floor, returns from this function
	 * after wake (no re-init), and wakes on RTC OR reed EXTI — both
	 * paths matter for the deployment.
	 *
	 * The deeper SHUTDOWN mode (chip fully off, magnet-only wake) stays
	 * available for operator-controlled end-of-mission via AT+SHUTDOWN
	 * and for the boot-loop guard's PERMANENT_OFF 24 h fallback. */
	(void)s_duty.shutdown_threshold_s;  /* no longer used by auto-cycle */
	MGR_LPM_UW_enterStop2Timed(sleep_s);
	/* Returns here on wake. State machine continues normally. */
}

/* ---- STANDBY timed wake ---- */

void MGR_LPM_UW_enterStandbyTimed(uint32_t seconds)
{
	MGR_LOG_DEBUG("[LPM_UW] STANDBY %lus\r\n", (unsigned long)seconds);

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

/* ---- SHUTDOWN with reed-magnet wake ---- */

void MGR_LPM_UW_enterShutdownReed(void)
{
	MGR_LOG_DEBUG("[LPM_UW] SHUTDOWN+reed\r\n");
	/* Use the existing LPM_shutdownNow path. On SMD_STDALONE it pulls
	 * PWR_LATCH LOW which lets the regulator collapse — the HW reed
	 * circuit then re-energises VBUS when the magnet is applied. */
	LPM_shutdownNow();
	for (;;) { /* unreachable */ }
}

void MGR_LPM_UW_enterShutdownAutoWake(uint32_t wakeup_seconds)
{
	MGR_LOG_DEBUG("[LPM_UW] SHUTDOWN auto-wake %lus\r\n",
		(unsigned long)wakeup_seconds);
	LPM_shutdownWithAutoWake(wakeup_seconds);
	for (;;) { /* unreachable */ }
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

void MGR_LPM_UW_enterStop2Timed(uint32_t seconds)
{
	MGR_LOG_DEBUG("[LPM_UW] STOP2 %lus\r\n", (unsigned long)seconds);

	if (seconds > 0xFFFFu) seconds = 0xFFFFu;

	/* Arm RTC wake-up timer if a non-zero interval was requested. With
	 * seconds=0 only EXTI sources (reed, gesture) can wake. */
	if (seconds > 0u) {
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
		__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
		(void)HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
		    (uint16_t)(seconds - 1u),
		    RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
	}

	/* Internal wake-up line routes RTC events to the PWR wake-up logic.
	 * Without this RTC fires its IRQ but the chip doesn't exit STOP2. */
	HAL_PWREx_DisableInternalWakeUpLine();
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);
	HAL_PWREx_EnableInternalWakeUpLine();

	/* Peripheral teardown — without this the chip enters STOP2 but the
	 * SWS analog rail / LED state keep drawing several mA. The
	 * MGR_LPM_aggregator's lpmNotifEnter callback would normally do
	 * this, but our direct-from-MONITORING path bypasses the aggregator
	 * for finer control of the cycle. Replicate the necessary tear-down
	 * inline. */
#if defined(BSP_HAS_LED_RGB)
	MGR_LED_off();
#endif
	MGR_SWS_enterLowPower();

	/* SysTick gets disabled during STOP (no HCLK), then re-enabled on
	 * wake. HAL_SuspendTick avoids spurious tick interrupts wedging WFI. */
	HAL_SuspendTick();

	/* Enter STOP2. WFI returns here once a configured wake source fires
	 * (RTC alarm, or any pending EXTI — including reed PB6 which the
	 * MGR_REED driver sets up at boot with rising+falling edge IT). */
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

	/* === Awake again === */

	/* Disarm the RTC wake timer so it doesn't fire again mid-MONITORING. */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

	/* On STOP exit the chip is on MSI; restore the production PLL config
	 * so peripheral baud rates / TX timeouts behave the same as before. */
	uw_restore_clock_from_stop();

	/* Re-enable SysTick + compensate the elapsed wall time so HAL_GetTick()
	 * stays monotonic across the sleep window. The existing lpm.c machinery
	 * already does both via the stop_exit callback; we replicate the
	 * essential bits here so we don't need to hook into MGR_LPM. */
	HAL_ResumeTick();

	/* Re-init ADC: STOP2 deinitialises the peripheral, and without this
	 * SWS reads return 0 for the whole post-wake cycle (observed). */
	MX_ADC_Init();

	/* SWS exit-LP mirrors the enterLowPower call, re-arms the analog rail. */
	MGR_SWS_exitLowPower();

	/* No HAL_UART_TX in this function — log only on caller side once we're
	 * back in the main loop, otherwise the BAUD lock-up from waking still
	 * partially initialized UART hardware can stall the print. */
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
	if (a) *a = 0; if (b) *b = 0; if (c) *c = 0;
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

#endif /* BSP_HAS_PWR_LATCH */
