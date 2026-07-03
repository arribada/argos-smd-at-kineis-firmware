/* SPDX-License-Identifier: no SPDX license */
/**
 * @file mgr_at_cmd_common.h
 * @author Kineis
 * @brief header file for common part of the AT cmd manager
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MGR_AT_CMD_COMMON_H
#define __MGR_AT_CMD_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "strutil_lib.h"
#include "kineis_sw_conf.h" // get ERROR_RETURN_T types

/* Defines ------------------------------------------------------------------*/

enum atcmd_type_t {
	ATCMD_ACTION_MODE,
	ATCMD_STATUS_MODE,
	ATCMD_INVALID_USAGE
};

/** enum of command response */
enum atcmd_rsp_type_t {
	ATCMD_RSP_TXOK,		    /**< At command delayed response for TX done */
	ATCMD_RSP_TXACKOK,	    /**< At command delayed response for TX done */
	ATCMD_RSP_TXACKNOTOK,	/**< At command delayed response for TX timeout */
	ATCMD_RSP_SATDET,	    /**< At command delayed response for SAT detection */
	ATCMD_RSP_SATLOST,	    /**< At command delayed response for SAT lost */
	ATCMD_RSP_SATDETTO,	    /**< At command delayed response for SAT detection timeout */
	ATCMD_RSP_RXOK,		    /**< At command delayed response for RX frame reception */
	ATCMD_RSP_DLOK,		    /**< At command delayed response for RX frame reception */
	ATCMD_RSP_RXTIMEOUT,    /**< At command delayed response for RX timeout */
	ATCMD_RSP_TXTIMEOUT,    /**< At command delayed response for TX timeout */
	ATCMD_RSP_RXERROR,      /**< At command delayed response for RX error */
	ATCMD_RSP_RFABORTED     /**< At command delayed response for RF operation aborted */
};


/* functions prototypes -----------------------------------------------------*/

/** @brief : Log in debug interface a succeed message
 *
 * @return : always return true
 */
bool bMGR_AT_CMD_logSucceedMsg(void);

/** @brief Log in debug interface a failed message
 *
 * @param[in] eErrorType : the error type
 *
 * @return : always return false
 */
bool bMGR_AT_CMD_logFailedMsg(enum ERROR_RETURN_T eErrorType);

/** @brief Translate a KNS_status_t to the ERROR_RETURN_T value sent over UART.
 *
 * v11.1.0 inserted KNS_STATUS_ABORT=7 inside the [0..99] band — a value that
 * was undefined in v10 and that legacy GUI parsers therefore don't recognise.
 * This helper folds ABORT into KNS_STATUS_ERROR (=1, the generic error code
 * the GUI has always understood) and passes any other code through. Use it
 * everywhere we forward `srvcEvt.status` or a MAC return code into
 * `bMGR_AT_CMD_logFailedMsg` to keep the +ERROR=N surface stable.
 */
enum ERROR_RETURN_T MGR_AT_CMD_mapKnsStatusToError(enum KNS_status_t s);

/** @brief : This function writes in UART the response of specific command
 *
 * @param[in] atcmd_response_type : enum ATCMD_RESPONSE_TYPE_T: the type of response
 * @param[in] atcmd_rsp_data : pointer the at command response data
 *
 * @return true if command is correctly executed  false if error
 */
bool bMGR_AT_CMD_sendResponse(enum atcmd_rsp_type_t atcmd_response_type, void *atcmd_rsp_data);

/** @brief Set the Kineis TX frame handler appended at the END of the next +TX
 *  response (+TX=<code>,<data>,<hdlr>). Called from MGR_AT_CMD_macEvtProcess
 *  with srvcEvt.tx_ctxt.frm_hdlr. Diverges from KIM (handler-first) to preserve
 *  backward compatibility with existing hosts.
 */
void MGR_AT_CMD_setTxFrmHdlr(int16_t frm_hdlr);

#endif /* __MGR_AT_CMD_COMMON_H */

/**
 * @}
 */
