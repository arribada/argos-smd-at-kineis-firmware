/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kns_srvc_common.h
 * @brief   Common part of Kinéis service layer
 * @author  Kinéis
 *
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Kineis. All rights reserved.</center></h2>
 *
 * This software component is licensed by Kineis under Ultimate Liberty license, the "License";
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * * kineis.com
 *
 */

/**
 * @addtogroup KNS_SRV
 * @brief low-level RF Service module: TX/RX/TRX common part

 * @{
 */

#ifndef KNS_SRVC_COMMON_H
#define KNS_SRVC_COMMON_H

/* Includes ------------------------------------------------------------------------------------ */

#include <stdbool.h>
#include <stdint.h>
#include "kns_types.h"


#pragma GCC visibility push(default)

/* Defines ------------------------------------------------------------------------------------- */

#define DL_FRM_BITLEN 384 /* maximum bit length of a DL frame */
#define DL_FRM_SZ  (((DL_FRM_BITLEN-1)/8)+1) /*  max frame len in byte */

/* Enums --------------------------------------------------------------------------------------- */

/**
 * @enum KNS_SRVC_frame_status_t
 * @brief frame status at SERVICE SW layer level
 */
enum KNS_SRVC_frame_status_t {
	KNS_SRVC_FREE,       /**< no frame configured */
	KNS_SRVC_VALID,      /**< frame buffer contains a frame ready for transmission */
	KNS_SRVC_TX_ONGOING, /**< frame is being transmitted, wait for tx-done event */
	KNS_SRVC_RX_ONGOING  /**< in case of TRX, frame was transmitted, now in RX-after-TX */
};

/**
 * @enum KNS_SRVC_eventId_t
 * @brief List events which could be reported by service layer
 */
enum KNS_SRVC_eventId_t {
	KNS_SRVC_TX_DONE,       /**< TX completed */
	KNS_SRVC_TX_TIMEOUT,    /**< TX failed, timeout reached */
	KNS_SRVC_TXACK_DONE,    /**< UL-ACK TX completed */
	KNS_SRVC_TXACK_TIMEOUT, /**< UL-ACK TX failed, timeout reached */
	KNS_SRVC_RX_ERROR,      /**< error occurred during RX process */
	KNS_SRVC_RX_RECEIVED,   /**< RX frame received */
	KNS_SRVC_RX_TIMEOUT,    /**< RX window timeout */
	KNS_SRVC_SAT_DETECTED,  /**< SATellite detection complete */
	KNS_SRVC_SAT_LOST,      /**< SATellite lost */
	KNS_SRVC_SAT_DETECT_TIMEOUT,  /**< SATellite detection timeout reached */
	KNS_SRVC_RF_ABORTED,    /**< RF action fully aborted */
	KNS_SRVC_ERROR          /**< generic error event */
};

/* Structures ---------------------------------------------------------------------------------- */

/**
 * @struct KNS_SATDET_ctxt_t
 * @brief RX-frame-received event context structure
 *
 * All useful parameters needed when a satellite is detected:
 * * downlink frequency in Hz
 * * duration needed to detect
 * * received signal strength (RSSI) in dBFs
 */
struct KNS_SATDET_ctxt_t {
	uint32_t dl_freq;
	uint32_t dl_rx_det_duration;
	float dl_rssi;
	uint32_t satdet_patt_report;
	uint32_t nb_err_report;
};

/**
 * @struct KNS_RX_frm_ctxt_t
 * @brief RX-frameèreceived event context structure
 *
 * All useful parameters needed when some frame is received
 */
struct KNS_RX_frm_ctxt_t {
	uint8_t data[DL_FRM_SZ];
	uint16_t data_bitlen;
	float dl_rssi;
	uint32_t dl_freq;
};

/**
 * @struct KNS_SRVC_evt_t
 * @brief Generic event structure
 *
 * Union describing event reported by this driver
 */
struct KNS_SRVC_evt_t {
	enum KNS_SRVC_eventId_t id;
	union {
#ifdef USE_RX_STACK // Rx event are only usefull in KIM2 which support downlink
		struct KNS_SATDET_ctxt_t satdet_ctxt; /**< SAT DETECTion notification context*/
		struct KNS_RX_frm_ctxt_t rx_ctxt; /**< RX-frame-received notification context*/
#endif
	};
};

/**
 * @struct KNS_TX_cfg_t
 * @brief TX configuration structure need to post an UL frame
 *
 * All parameters requested to define radio tx configuration, such as:
 * * data
 * * frame type (no service, PACK, mail request)
 * * TX ISR callback
 */
struct KNS_TX_cfg_t {
	enum KNS_serviceFlag_t sf;	/** Service flag from Kineis protocol */
	struct KNS_tx_rf_cfg_t rf_cfg;
};

/**
 * @struct KNS_RX_cfg_t
 * @brief RX configuration structure
 *
 * @todo May need to add RX window length once KNS_MAC upper layer is supported
 */
struct KNS_RX_cfg_t {
	bool isSatDetectOnly; /**< just perform a one-shot-sat-detectsion */
	bool isRfAlreadyOn;	/**< tells RF chip is said to be already ON before starting RX,
				  * In case already-ON, no need to do POWER ON nor waiting for
				  * TCXOWU.
				  */
};

#pragma GCC visibility pop

#endif /* KNS_SRVC_COMMON_H */
/**
 * @}
 */
