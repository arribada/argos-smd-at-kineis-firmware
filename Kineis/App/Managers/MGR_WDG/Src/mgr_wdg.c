/**
 * @file    mgr_wdg.c
 * @brief   IWDG watchdog manager (direct register access)
 *
 * Uses CMSIS IWDG register definitions — no HAL driver dependency.
 * The STM32WL HAL package doesn't include the IWDG HAL driver, so
 * we configure the watchdog via direct register access.
 *
 * IWDG register map (base 0x40003000):
 *   KR  (0x00): Key register — 0xCCCC=start, 0xAAAA=refresh, 0x5555=unlock
 *   PR  (0x04): Prescaler — 0..7 for /4../256
 *   RLR (0x08): Reload value — 0..4095
 *   SR  (0x0C): Status — bit0=PVU, bit1=RVU (update pending flags)
 *
 * Init sequence per RM0461:
 *   1. Write 0xCCCC to KR (start IWDG, activate LSI)
 *   2. Write 0x5555 to KR (unlock PR and RLR)
 *   3. Write prescaler and reload values
 *   4. Wait for SR == 0 (update complete)
 *   5. Write 0xAAAA to KR (first refresh)
 *
 * The IWDG_STOP option bit (OB) controls whether the watchdog is
 * frozen during STOP mode. Default on STM32WL is frozen (safe).
 */

/**
 * @addtogroup MGR_WDG
 * @{
 */

#include "mgr_wdg.h"
#include "stm32wlxx.h"
#include "stm32wlxx_hal.h"
#include "mgr_log.h"

/* IWDG key values */
#define IWDG_KEY_ENABLE    0xCCCCU
#define IWDG_KEY_REFRESH   0xAAAAU
#define IWDG_KEY_UNLOCK    0x5555U

/* Prescaler /256 (PR=6), Reload=2000 → timeout = 2000 * 256 / 32000 = 16s */
#define IWDG_PRESCALER     6U
#define IWDG_RELOAD        2000U

bool MGR_WDG_ensureIwdgStopOptionByte(void)
{
	/* If the bit is already set we're done — no flash write needed. */
	if ((FLASH->OPTR & FLASH_OPTR_IWDG_STOP) != 0U) {
		return false;
	}

	MGR_LOG_DEBUG("[WDG] IWDG_STOP option byte not set — programming...\r\n");

	/* Unlock FLASH for option-byte access. Order matters: FLASH first,
	 * then option-byte. Both must succeed before any HAL_FLASHEx_OBProgram. */
	if (HAL_FLASH_Unlock() != HAL_OK) {
		MGR_LOG_DEBUG("[WDG] FLASH unlock failed\r\n");
		return false;
	}
	if (HAL_FLASH_OB_Unlock() != HAL_OK) {
		MGR_LOG_DEBUG("[WDG] OB unlock failed\r\n");
		HAL_FLASH_Lock();
		return false;
	}

	FLASH_OBProgramInitTypeDef obInit = {0};
	obInit.OptionType = OPTIONBYTE_USER;
	obInit.UserType   = OB_USER_IWDG_STOP;
	obInit.UserConfig = OB_IWDG_STOP_FREEZE;  /* 1 = freeze IWDG during STOP */

	if (HAL_FLASHEx_OBProgram(&obInit) != HAL_OK) {
		MGR_LOG_DEBUG("[WDG] OB program failed\r\n");
		HAL_FLASH_OB_Lock();
		HAL_FLASH_Lock();
		return false;
	}

	/* Launch: reload the option bytes. This call triggers a SYSTEM RESET
	 * — it does not return. On the next boot the bit will be set and this
	 * function will see it and skip the entire path. */
	(void)HAL_FLASH_OB_Launch();

	/* If we somehow reach here (Launch did not reset), best effort cleanup. */
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
	return false;
}

void MGR_WDG_init(void)
{
	/* Start the IWDG first — activates LSI clock automatically.
	 * Must be done BEFORE unlock per RM0461 and HAL reference.
	 * Cannot be stopped after this!
	 */
	IWDG->KR = IWDG_KEY_ENABLE;

	/* Unlock PR and RLR registers */
	IWDG->KR = IWDG_KEY_UNLOCK;

	/* Set prescaler to /256 */
	IWDG->PR = IWDG_PRESCALER;

	/* Set reload value */
	IWDG->RLR = IWDG_RELOAD;

	/* Wait for prescaler and reload update to complete (timeout ~100ms) */
	uint32_t timeout = 100000U;
	while (IWDG->SR != 0U && --timeout > 0U) {
		/* SR bits PVU and RVU clear when update is done */
	}
	if (timeout == 0U)
		MGR_LOG_DEBUG("[WDG] WARNING: IWDG SR update timeout\r\n");

	/* First refresh */
	IWDG->KR = IWDG_KEY_REFRESH;

	/* Verify IWDG_STOP option byte is set (IWDG frozen during STOP mode).
	 * If not set, IWDG would keep running during STOP and reset the device
	 * during long sleep intervals. FLASH_OPTR bit 17 = IWDG_STOP (1=frozen). */
	if ((FLASH->OPTR & FLASH_OPTR_IWDG_STOP) == 0U)
		MGR_LOG_DEBUG("[WDG] WARNING: IWDG_STOP not set in option bytes!\r\n");

	MGR_LOG_DEBUG("[WDG] IWDG started (timeout ~16s)\r\n");
}

void MGR_WDG_refresh(void)
{
	IWDG->KR = IWDG_KEY_REFRESH;
}

void MGR_WDG_delayWithKick(uint32_t total_ms)
{
	/* Kick before starting so we have the full IWDG window for the first chunk
	 * even if the caller hadn't kicked recently. */
	IWDG->KR = IWDG_KEY_REFRESH;

	uint32_t start = HAL_GetTick();
	while ((HAL_GetTick() - start) < total_ms) {
		uint32_t elapsed = HAL_GetTick() - start;
		uint32_t remaining = (elapsed < total_ms) ? (total_ms - elapsed) : 0;
		uint32_t chunk = remaining > 1000 ? 1000 : remaining;
		HAL_Delay(chunk);
		IWDG->KR = IWDG_KEY_REFRESH;
	}
}

/**
 * @}
 */
