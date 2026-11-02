/**
 * @file    lpm_cfg_check.h
 * @brief   Detects an lpm_config left in SRAM2 by another firmware image.
 *
 * lpm_config lives in SRAM2, which survives resets and is re-initialised only
 * on a cold boot. A reflash or a DFU that follows an LPM entry would otherwise
 * boot the new image with the previous image's LPM callbacks.
 */

#ifndef LPM_CFG_CHECK_H
#define LPM_CFG_CHECK_H

#include <stdbool.h>
#include "mgr_lpm.h"

/** True when every LPM callback in @p ram is the one in @p rom. The bitmap is
 * not compared: the host may change it at run time. */
static inline bool LPM_cfgCallbacksMatch(const struct MgrLpm_EnvConfig_t *ram,
					 const struct MgrLpm_EnvConfig_t *rom)
{
	return ram->fp_sleep_enter == rom->fp_sleep_enter &&
	       ram->fp_sleep_exit == rom->fp_sleep_exit &&
	       ram->fp_stop_enter == rom->fp_stop_enter &&
	       ram->fp_stop_exit == rom->fp_stop_exit &&
	       ram->fp_standby_enter == rom->fp_standby_enter &&
	       ram->fp_shutdown_enter == rom->fp_shutdown_enter;
}

/** Boot decision: SRAM2 must be re-initialised (cold start) when the callbacks
 * do not match. A SHUTDOWN wake is left alone: SRAM2 is lost there and the boot
 * re-initialises it anyway, keeping the mode for its wake path. */
static inline bool LPM_sram2Stale(enum MgrLpm_LPM_t boot_mode, bool callbacks_match)
{
	return boot_mode != LOW_POWER_MODE_SHUTDOWN && !callbacks_match;
}

#endif /* LPM_CFG_CHECK_H */
