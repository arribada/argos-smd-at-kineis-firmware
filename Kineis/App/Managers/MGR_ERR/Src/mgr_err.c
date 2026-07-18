/**
 * @file    mgr_err.c
 * @brief   Error tracker using TAMP backup registers
 *
 * Uses STM32WL TAMP backup registers BKP12R-BKP17R to persist
 * error context across system resets. These registers survive
 * all resets (NRST, software, watchdog, brown-out) and are only
 * cleared on VBAT removal.
 *
 * Register map (this module):
 *   BKP12R = reset counter (incremented each boot)
 *   BKP13R = last RCC_CSR flags (reset cause snapshot)
 *   BKP14R = last error code (MGR_ERR_Code_t)
 *   BKP15R = last UW_DOPPLER state at time of error
 *   BKP16R = last HAL tick at time of error
 *   BKP17R = consecutive crash counter
 *
 * Init sequence: read previous session data, log it, then clear
 * error code for this session and increment boot counter.
 */

/**
 * @addtogroup MGR_ERR
 * @{
 */

#include <stddef.h>
#include "mgr_err.h"
#include "stm32wlxx.h"
#include "stm32wlxx_hal.h"
#include "mgr_log.h"

/* Use TAMP peripheral struct for backup register access.
 *
 * REMAPPED BKP2R-BKP7R -> BKP12R-BKP17R (fix 2026-07): the old block
 * collided with the LINKER-placed backup-domain data at 0x4000B100
 * (.lpmSection / .msgCntSectionData in STM32WL55XX_FLASH_APP.ld):
 * mcu_tim's timer[] sits in BKP0R-BKP3R, lpm_ctxt in BKP4R and the
 * Kineis Argos message_counter in BKP5R. Every MGR_ERR_log therefore
 * overwrote the LIVE message counter (on-air MC corruption + spurious
 * destructive MC page rewrites) and MCU_TIM_resetState zeroed the boot
 * counter each boot ("Boot #1" forever).
 *
 * Full TAMP BKPxR allocation on this firmware:
 *   BKP0R-BKP3R   mcu_tim timer[] (linker .lpmSection). BKP0/1 are ALSO
 *                 the DFU request flag (main.c / bootloader) — benign by
 *                 ordering: consumed at boot before any timer is armed.
 *   BKP4R         lpm_ctxt (linker)
 *   BKP5R         Kineis Argos message_counter (linker .msgCntSectionData)
 *   BKP6R         MGR_ERR fault-streak counter (this module)
 *   BKP7R-BKP9R   free
 *   BKP10R        MGR_GESTURE mode-persistence magic (0x47535400|mode) —
 *                 written every boot by MGR_GESTURE_init/persist_mode
 *   BKP11R        MGR_LPM_UW soft-off wake magic (SOFTOFF_WAKE_MAGIC)
 *   BKP12R-BKP17R MGR_ERR (this module)
 *   BKP18R        flash-ECC NMI breadcrumb (stm32wlxx_it.c)
 *   BKP19R        MGR_CRED provisioning-intent flag (mgr_cred.c, "CINT")
 * Keep this table in sync when allocating a new backup register. */
#define ERR_BKP_COUNT   (TAMP->BKP12R)  /* Reset counter */
#define ERR_BKP_CSR     (TAMP->BKP13R)  /* Last RCC_CSR */
#define ERR_BKP_CODE    (TAMP->BKP14R)  /* Last error code */
#define ERR_BKP_STATE   (TAMP->BKP15R)  /* Last state */
#define ERR_BKP_TICK    (TAMP->BKP16R)  /* Last tick */
#define ERR_BKP_CRASH   (TAMP->BKP17R)  /* Consecutive crash counter (<30 s boots) */
/* Fault-streak counter moved BKP10R -> BKP6R (fix 2026-07, audit #7): BKP10R
 * is ALSO owned by MGR_GESTURE, which writes 0x47535400|mode into it on EVERY
 * boot (MGR_GESTURE_init -> persist_mode). That clobbered the streak with a
 * constant 0x47535400 (not < FAULT_STREAK_MAX), so a SINGLE transient fault
 * tripped the 1 h crash-loop safe-sleep designed to require 20. BKP6R was
 * free. Consecutive fault-terminated boots, ANY uptime. */
#define ERR_BKP_FSTREAK (TAMP->BKP6R)

__attribute__((unused)) static const char *err_code_str(MGR_ERR_Code_t code)
{
	switch (code) {
	case ERR_NONE:           return "NONE";
	case ERR_HARDFAULT:      return "HARDFAULT";
	case ERR_BUSFAULT:       return "BUSFAULT";
	case ERR_MEMMANAGE:      return "MEMMANAGE";
	case ERR_USAGEFAULT:     return "USAGEFAULT";
	case ERR_NMI:            return "NMI";
	case ERR_ASSERT:         return "ASSERT";
	case ERR_MAC_TIMEOUT:    return "MAC_TIMEOUT";
	case ERR_MAC_INIT_FAIL:  return "MAC_INIT_FAIL";
	case ERR_TX_TIMEOUT:     return "TX_TIMEOUT";
	case ERR_STACK_OVERFLOW: return "STACK_OVERFLOW";
	case ERR_WDG_RESET:      return "WDG_RESET";
	case ERR_PA_STUCK:       return "PA_STUCK";
	case ERR_BOOT_LOOP:      return "BOOT_LOOP";
	case ERR_STATE_HANG:     return "STATE_HANG";
	case ERR_CREDS_BLANK:    return "CREDS_BLANK";
	case ERR_RTC_DEAD:       return "RTC_DEAD";
	default:                 return "UNKNOWN";
	}
}

__attribute__((unused)) static const char *reset_cause_str(uint32_t csr)
{
	if (csr & RCC_CSR_IWDGRSTF) return "IWDG";
	if (csr & RCC_CSR_SFTRSTF)  return "SOFTWARE";
	if (csr & RCC_CSR_PINRSTF)  return "PIN";
	if (csr & RCC_CSR_BORRSTF)  return "BROWNOUT";
	return "POWER_ON";
}

void MGR_ERR_init(void)
{
	/* Backup access must already be enabled (enable_backup_access() in main.c) */

	/* Read previous session info before modifying. The `unused` attribute
	 * silences -Werror=unused-variable in DEBUG=0 builds where all
	 * MGR_LOG_DEBUG calls below collapse to do{}while(0). */
	uint32_t prev_count = ERR_BKP_COUNT;
	uint32_t prev_csr   __attribute__((unused)) = ERR_BKP_CSR;
	MGR_ERR_Code_t prev_err = (MGR_ERR_Code_t)ERR_BKP_CODE;
	uint32_t prev_state __attribute__((unused)) = ERR_BKP_STATE;
	uint32_t prev_tick  = ERR_BKP_TICK;

	/* Read current reset cause from RCC_CSR. NOTE: main.c clears RCC_CSR (RMVF) on
	 * every reset where PINRSTF co-asserts — internal IWDG/SW resets do, with the
	 * default bidirectional NRST — so `csr` reads NONE for those. Crash counting
	 * and the Sram2 gate intentionally stay on this (conservative) value until the
	 * snapshot-based behaviour is bench-validated. The pre-RMVF snapshot is used
	 * for OBSERVABILITY only, so a silent IWDG hang is visible at the bench. */
	uint32_t csr = RCC->CSR;
	extern uint32_t g_boot_rcc_csr_raw;       /* captured in main() before RMVF */
	uint32_t true_csr = g_boot_rcc_csr_raw;

	/* If THIS boot was caused by IWDG but the previous session never logged
	 * an error code, the previous boot hung silently — surface that as
	 * WDG_RESET so the user sees it instead of last_err=NONE. tick stays 0
	 * because the previous boot never got far enough to record one.
	 * Fix 2026-07: use the PRE-RMVF snapshot — the post-RMVF `csr` reads 0
	 * on every internal reset (bidirectional NRST co-asserts PINRSTF and
	 * main.c clears the flags), so this synthesis was dead in production
	 * and silent IWDG hangs were invisible to the fault-streak ladder
	 * below. IWDGRSTF in the true snapshot is unambiguous (a debugger
	 * SYSRESETREQ sets SFT, not IWDG). */
	if ((true_csr & RCC_CSR_IWDGRSTF) && prev_err == ERR_NONE) {
		prev_err = ERR_WDG_RESET;
	}

	/* Log the reset cause for THIS boot (most useful diagnostic).
	 * The previous-session summary follows for context. */
	MGR_LOG_INFO("[ERR] Boot #%lu reset_cause=%s csr=0x%08lx\r\n",
		prev_count + 1, reset_cause_str(csr), (unsigned long)csr);

	/* Log previous session info */
	MGR_LOG_INFO("[ERR] Boot #%lu: prev_reset=%s last_err=%s last_state=%lu tick=%lu\r\n",
		prev_count + 1,
		reset_cause_str(prev_csr),
		err_code_str(prev_err),
		prev_state,
		prev_tick);

	/* OBSERVABILITY (audit 2026-06-20, no behaviour change): if the true pre-RMVF
	 * cause shows an internal reset that the cleared `csr` masks, surface it. This
	 * is the silent-IWDG / SW-reset the crash counter currently does NOT count —
	 * read this WARN at the bench to decide whether to move counting onto the
	 * snapshot (see task). Functional path above stays on `csr`. */
	if ((true_csr & (RCC_CSR_IWDGRSTF | RCC_CSR_SFTRSTF)) &&
	    !(csr & (RCC_CSR_IWDGRSTF | RCC_CSR_SFTRSTF)))
		MGR_LOG_WARN("[ERR] true reset cause masked by RMVF: %s (not counted)\r\n",
			reset_cause_str(true_csr));

	/* Crash loop detection: check if previous boot was short-lived.
	 * ERR_CREDS_BLANK is excluded (fix 2026-07): the CRED_FAIL park logs it
	 * early in every wake cycle by DESIGN (deliberate fail-loud state, not a
	 * crash), so counting it made a blank-cred unit accrue a false crash
	 * loop and take a spurious 1 h safe sleep every ~10 park cycles —
	 * narrowing the operator's already-narrow recovery windows. */
	uint32_t prev_crash = ERR_BKP_CRASH;
	if (prev_tick > 0 && prev_tick < MGR_ERR_CRASH_LOOP_MIN_UP_MS &&
	    prev_err != ERR_NONE && prev_err != ERR_CREDS_BLANK) {
		prev_crash++;
	} else {
		prev_crash = 0;  /* Previous boot was stable, reset counter */
	}

	/* Fault-streak ladder (fix 2026-07, sim-proven structural gap): the
	 * <30 s crash counter misses any reproducible fault at uptime >= 30 s
	 * and the boot-loop ladder only counts pre-MONITORING deaths — a tag
	 * faulting 60 s after every boot ran 100% awake FOREVER (sim: 1439
	 * boots/day, no remedy, pack dead in weeks). Count CONSECUTIVE
	 * fault-terminated boots regardless of uptime; any clean boot (no
	 * error logged, incl. AT+RESET and park wakes) resets the streak.
	 * ERR_CREDS_BLANK is excluded (deliberate fail-loud park, not a
	 * crash). checkCrashLoop() throttles the streak with the same 1 h
	 * safe sleep, bounding the awake duty to ~streak_max*period per hour. */
	uint32_t prev_streak = ERR_BKP_FSTREAK;
	/* Migration guard (audit #8): the fault-streak counter moved to BKP6R,
	 * which on a field unit updated over DFU may hold uncontrolled retained data
	 * (older firmware used BKP6R as ERR_BKP_TICK — a large HAL-tick value). A
	 * stale seed far above the trip threshold would fire a spurious 1 h safe-
	 * sleep on the first post-update boot (only when the last session logged an
	 * error, so prev_err != ERR_NONE below). Treat any implausible value as
	 * garbage. A real streak can never legitimately exceed FAULT_STREAK_MAX by
	 * much (checkCrashLoop zeroes it at the cap), so 2x is a safe ceiling. */
	if (prev_streak > (uint32_t)(MGR_ERR_FAULT_STREAK_MAX * 2u))
		prev_streak = 0;
	if (prev_err != ERR_NONE && prev_err != ERR_CREDS_BLANK) {
		if (prev_streak < 0xFFFFu)
			prev_streak++;
	} else {
		prev_streak = 0;
	}
	if (prev_streak > 0)
		MGR_LOG_WARN("[ERR] Fault streak: %lu consecutive\r\n",
			(unsigned long)prev_streak);

	/* Update registers for this session */
	ERR_BKP_COUNT = prev_count + 1;  /* Increment reset counter */
	ERR_BKP_CSR   = csr;             /* Store current reset cause */
	ERR_BKP_CODE  = ERR_NONE;        /* Clear error code */
	ERR_BKP_STATE = 0;
	ERR_BKP_TICK  = 0;
	ERR_BKP_CRASH = prev_crash;
	ERR_BKP_FSTREAK = prev_streak;

	if (prev_crash > 0) {
		MGR_LOG_WARN("[ERR] Consecutive crashes: %lu\r\n", prev_crash);
	}

	/* Clear RCC reset flags for next reset detection */
	RCC->CSR |= RCC_CSR_RMVF;
}

void MGR_ERR_log(MGR_ERR_Code_t code)
{
	extern volatile uint32_t g_uw_doppler_state_for_err;
	ERR_BKP_CODE  = (uint32_t)code;
	ERR_BKP_STATE = g_uw_doppler_state_for_err;
	ERR_BKP_TICK  = HAL_GetTick();

	MGR_LOG_ERR("[ERR] Error logged: %s (tick=%lu)\r\n",
		err_code_str(code), HAL_GetTick());
}

void MGR_ERR_logAndReset(MGR_ERR_Code_t code)
{
	MGR_ERR_log(code);

	/* Drain the log ring buffer synchronously so the assert message reaches
	 * the host UART before reset. Without this the user only sees the next
	 * boot's banner with no clue what failed. Bounded loop: cap iterations
	 * to avoid wedging here if the UART itself is the failure cause. */
	for (int i = 0; i < 32 && MGR_LOG_has_pending(); i++)
		(void)MGR_LOG_flush();

	__disable_irq();
	NVIC_SystemReset();
	/* Never reaches here */
	while (1) {}
}

uint32_t MGR_ERR_getResetCount(void)
{
	return ERR_BKP_COUNT;
}

MGR_ERR_Code_t MGR_ERR_getLastError(void)
{
	return (MGR_ERR_Code_t)ERR_BKP_CODE;
}

uint32_t MGR_ERR_getCrashCount(void)
{
	return ERR_BKP_CRASH;
}

bool MGR_ERR_checkCrashLoop(void)
{
	/* Two triggers (fix 2026-07): the fast <30 s crash loop (counter at
	 * MGR_ERR_CRASH_LOOP_MAX) and the slow ANY-uptime fault streak
	 * (MGR_ERR_FAULT_STREAK_MAX consecutive fault-terminated boots) — the
	 * latter closes the sim-proven post-MONITORING >=30 s blind spot. */
	if (ERR_BKP_CRASH < MGR_ERR_CRASH_LOOP_MAX &&
	    ERR_BKP_FSTREAK < MGR_ERR_FAULT_STREAK_MAX)
		return false;

	MGR_LOG_ERR("[ERR] CRASH/FAULT LOOP detected (crash=%lu streak=%lu), safe sleep %us\r\n",
		ERR_BKP_CRASH, ERR_BKP_FSTREAK, MGR_ERR_CRASH_LOOP_SLEEP_S);

	/* Configure RTC wakeup timer to wake after CRASH_LOOP_SLEEP_S seconds.
	 * RTC clock = LSE (32768 Hz), wakeup clock = CK_SPRE (1 Hz).
	 */
	__HAL_RCC_RTCAPB_CLK_ENABLE();

	/* Unlock RTC write protection (RM0453: 0xCA then 0x53). Without this,
	 * MX_RTC_Init leaves WPR locked and EVERY RTC->CR/WUTR write below is
	 * SILENTLY DROPPED — the wakeup timer is never armed and the chip would
	 * enter STOP2 with no wake source = strand. Re-locked on every exit. */
	RTC->WPR = 0xCAU;
	RTC->WPR = 0x53U;

	/* Disable wakeup timer to modify it */
	RTC->CR &= ~RTC_CR_WUTE;
	{
		uint32_t timeout = 100000U;
		while ((RTC->ICSR & RTC_ICSR_WUTWF) == 0 && --timeout > 0)
			;
		if (timeout == 0) {
			RTC->WPR = 0xFFU;  /* re-lock before bailing */
			return false;      /* RTC not responding, skip safe sleep */
		}
	}

	/* Select 1 Hz clock source (ck_spre) and set countdown */
	RTC->CR &= ~RTC_CR_WUCKSEL;
	RTC->CR |= RTC_CR_WUCKSEL_2;  /* WUCKSEL = 0b100 = ck_spre */
	RTC->WUTR = MGR_ERR_CRASH_LOOP_SLEEP_S - 1;

	/* Clear wakeup flag and enable wakeup timer + interrupt */
	RTC->SCR = RTC_SCR_CWUTF;
	RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;

	/* The RTC wakeup timer drives STOP2 exit through the PWR internal wake-up
	 * line (EXTI line 20 on WL55), NOT the alarm line (17). The previous code
	 * armed IM17 (RTC alarm) and never enabled the internal wake-up line, so the
	 * RTC fired WUTF but the core never left STOP2 — turning the crash-loop
	 * backstop into a permanent brick. Mirror the proven STOP2 path in
	 * mgr_lpm_uw.c (HAL_PWREx_*InternalWakeUpLine). */
	HAL_PWREx_DisableInternalWakeUpLine();
	HAL_PWREx_EnableInternalWakeUpLine();

	/* Enter STOP2 mode (lowest power with SRAM retention) */
	__disable_irq();
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
	__enable_irq();

	/* Woke up - disable wakeup timer + internal wake line, re-lock RTC WP */
	HAL_PWREx_DisableInternalWakeUpLine();
	RTC->CR &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
	RTC->SCR = RTC_SCR_CWUTF;
	RTC->WPR = 0xFFU;

	/* Reset both loop counters to give the device another chance */
	ERR_BKP_CRASH = 0;
	ERR_BKP_FSTREAK = 0;

	MGR_LOG_INFO("[ERR] Woke from safe sleep, retrying\r\n");
	return true;
}

/* ---- HardFault forensics ---------------------------------------------- */

/* Crash-info lives in TRUE-retention SRAM2 (NOLOAD section, never wiped by
 * Sram2_Init). Survives every software-class reset class — IWDG, SFT, OBL,
 * BOR, PIN — so the next boot can read the forensics regardless of how the
 * fault path exits. Validated via magic + CRC; cleared after replay. */
static __attribute__((__section__(".retentionRamNoload")))
MGR_ERR_CrashInfo_t s_retained_crash;

/* Layout invariant: detect any accidental struct growth that would push
 * the retention region past the linker reservation. Update both the assert
 * and the linker if you intentionally grow the struct. */
_Static_assert(sizeof(MGR_ERR_CrashInfo_t) == 64,
               "MGR_ERR_CrashInfo_t size changed — verify retention budget");
_Static_assert(offsetof(MGR_ERR_CrashInfo_t, crc) ==
               sizeof(MGR_ERR_CrashInfo_t) - sizeof(uint32_t),
               "crc must be the last field — CRC range depends on it");

static uint32_t crash_crc(const MGR_ERR_CrashInfo_t *c)
{
	/* Adler-style sum over everything except the crc field. */
	const uint32_t *p = (const uint32_t *)c;
	size_t n = (offsetof(MGR_ERR_CrashInfo_t, crc)) / 4;
	uint32_t s = 0;
	for (size_t i = 0; i < n; i++)
		s = s * 31u + p[i];
	return s;
}

void MGR_ERR_captureFault(uint32_t *frame, uint8_t fault_type, uint8_t app_state)
{
	/* Build the record on the STACK first, then commit with a single
	 * struct assignment. A nested fault during capture would only leave
	 * the OLD (still-valid) record in retention rather than a half-written
	 * one that could pass a CRC check by accident.
	 *
	 * Also invalidate the magic first so a fault DURING the struct copy
	 * leaves the retention in an "invalid, definitely not a replay" state
	 * rather than a torn-but-pass-CRC state. */
	extern volatile uint32_t uwTick;
	MGR_ERR_CrashInfo_t local;

	s_retained_crash.magic = 0;  /* invalidate while we rebuild */
	__DMB();

	local.magic      = MGR_ERR_CRASH_MAGIC;
	local.fault_type = fault_type;
	local.app_state  = app_state;
	local._pad       = 0;
	local.hfsr       = SCB->HFSR;
	local.cfsr       = SCB->CFSR;
	local.bfar       = SCB->BFAR;
	local.mmfar      = SCB->MMFAR;

	if (frame != NULL) {
		/* Standard ARMv7-M exception frame layout (no FPU context). */
		local.r0    = frame[0];
		local.r1    = frame[1];
		local.r2    = frame[2];
		local.r3    = frame[3];
		local.r12   = frame[4];
		local.lr    = frame[5];
		local.pc    = frame[6];
		local.xpsr  = frame[7];
	} else {
		local.r0 = local.r1 = 0;
		local.r2 = local.r3 = 0;
		local.r12 = local.lr = 0;
		local.pc = local.xpsr = 0;
	}

	local.tick = uwTick;
	local.crc  = crash_crc(&local);

	/* Atomic-ish commit: single struct copy. On Cortex-M4 with -O the
	 * compiler emits ldm/stm sequences for the copy; not truly atomic but
	 * far smaller window than field-by-field, and a nested fault during the
	 * memcpy still leaves magic=valid+crc=valid OR magic=stale+crc=stale —
	 * never the half-valid combo that would pass replay. */
	s_retained_crash = local;
	__DMB();
}

bool MGR_ERR_hasRetainedCrash(void)
{
	return s_retained_crash.magic == MGR_ERR_CRASH_MAGIC &&
	       s_retained_crash.crc   == crash_crc(&s_retained_crash);
}

bool MGR_ERR_takeRetainedCrash(MGR_ERR_CrashInfo_t *out)
{
	if (!MGR_ERR_hasRetainedCrash())
		return false;
	if (out != NULL)
		*out = s_retained_crash;
	__DMB();  /* ensure copy completes before clearing the source */
	/* Clear so we only report once. Invalidate magic FIRST so a fault
	 * during clear can't leave a partially-valid record (magic ok + crc
	 * stale would replay false forensics on the next boot). */
	s_retained_crash.magic = 0;
	__DMB();
	s_retained_crash.crc = 0;
	return true;
}

/**
 * @}
 */
