/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    mgr_at_cmd_list_v11.h
 * @brief   AT commands introduced by krd_fw v11.1.0 (Arribada local handlers).
 *
 * AT+DL  — Argos downlink RX control (gated on BSP_HAS_DOWNLINK).
 * AT+GNSS — GNSS position set/get (gated on BSP_HAS_GNSS).
 *
 * Both handlers return +ERROR=FEATURE_NOT_AVAILABLE on boards where the
 * hardware is absent (current SMD_* boards). The GUI probes these at
 * connect time and greys out the corresponding UI panels.
 */

#ifndef MGR_AT_CMD_LIST_V11_H
#define MGR_AT_CMD_LIST_V11_H

#include "mgr_at_cmd_common.h"

bool bMGR_AT_CMD_DL_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);
bool bMGR_AT_CMD_GNSS_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

#endif /* MGR_AT_CMD_LIST_V11_H */
