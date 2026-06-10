/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kns_mac.h
 * @brief   Main header file for MAC layer of Kineis stack
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
 * @addtogroup KNS_MAC
 * @{
 */

#ifndef KNS_MAC_EVT_H
#define KNS_MAC_EVT_H

/* Includes ------------------------------------------------------------------------------------ */

#include <stdbool.h>
#include <stdint.h>
#include "kns_types.h"
#include "kns_cfg.h"
#include "kns_mac_prfl_cfg.h"

#pragma GCC visibility push(default)

/* Defines ------------------------------------------------------------------------------------- */

#ifdef USE_HDA4
#define KNS_MAC_USRDATA_MAXLEN 633 // 632.5 bytes long as maximum allowed in HDA4 (5060 bits)
#else
#define KNS_MAC_USRDATA_MAXLEN 25 // 24.5 bytes long as maximum allowed in LDA2L
#endif

#define DL_FRM_BITLEN 384 /* maximum bit length of a DL frame */
#define DL_FRM_SZ  (((DL_FRM_BITLEN-1)/8)+1) /*  max frame len in byte */

/* Enums --------------------------------------------------------------------------------------- */

/**
 * @enum KNS_MAC_appEvtId_t
 * @brief List events which could be reported by service layer
 */
enum KNS_MAC_appEvtId_t {
	KNS_MAC_APP_NONE_EVT,  /**< none, placed as value 0 to detect potential issues in SW */
	KNS_MAC_INIT,          /**< Init MAC profile */
	KNS_MAC_SEND_DATA,     /**< Send data on Kineis UL link */
	KNS_MAC_STOP_SEND_DATA,/**< Abort Send data on Kineis UL link */
	KNS_MAC_RX_START,      /**< Start reception on Kineis DL link (e.g. DL_BC or AOP/CS update) */
	KNS_MAC_RX_STOP,       /**< Stop reception on Kineis DL link */
	KNS_MAC_CFG,           /**< configure MAC */
	KNS_MAC_POS            /**< Send GNSS position */
};

/**
 * @enum KNS_MAC_srvcEvtId_t
 * @brief List events which could be reported by service layer
 */
enum KNS_MAC_srvcEvtId_t {
	KNS_MAC_SRVC_NONE_EVT, /**< default value meaning event is not valid */
	KNS_MAC_TX_DONE,       /**< TX completed */
	KNS_MAC_TX_TIMEOUT,    /**< TX failed, timeout reached */
	KNS_MAC_TX_ABORT,      /**< TX transmission aborted (e.g. during FIFO flush) */
	KNS_MAC_TXACK_DONE,    /**< UL-ACK TX completed */
	KNS_MAC_TXACK_TIMEOUT, /**< UL-ACK TX failed, timeout reached */
	KNS_MAC_RX_ERROR,      /**< error occurred during RX process */
	KNS_MAC_RX_RECEIVED,   /**< RX frame received */
	KNS_MAC_RX_TIMEOUT,    /**< RX window timeout */
	KNS_MAC_DL_USERBC,     /**< downlink message received: user beacon command */
	KNS_MAC_DL_SYSBC,      /**< downlink message received: system beacon command */
	KNS_MAC_DL_ALLCAST,    /**< downlink message received: allcast */
	KNS_MAC_DL_ACK,        /**< downlink message received: acknowledge */
	KNS_MAC_SAT_DETECTED,  /**< SATellite detection complete */
	KNS_MAC_SAT_LOST,      /**< SATellite lost */
	KNS_MAC_SAT_DETECT_TIMEOUT,  /**< SATellite detection timeout */
	KNS_MAC_RF_ABORTED,    /**< RF action fully aborted */
	KNS_MAC_OK,            /**< generic status telling OK to command */
	KNS_MAC_ERROR          /**< generic error event */
};

/**
 * @enum KNS_MAC_infraEvtId_t
 * @brief List events which could be reported by service layer
 */
enum KNS_MAC_infraEvtId_t {
	KNS_MAC_TRIG_SATDET,   /**< Do SATellite DETection of Kineis DL link */
	KNS_MAC_TRIG_TX        /**< Send data on Kineis UL link */
};

/**
 * @enum KNS_MAC_rxMode_t
 */
enum KNS_MAC_rxMode_t {
	KNS_MAC_RX_MODE_TEST,   /**< test mode sniffing frames */
	KNS_MAC_RX_MODE_RUNTIME /**< runtime mode to report only valid messages */
};

/* Structures ---------------------------------------------------------------------------------- */

	/* ---- APP-to-MAC events ---- */

/**
 * @struct KNS_MAC_cfg_ctxt_t
 * @brief Event context structure containing stack configuration information
 *
 * The stack configuration bitmap
 *
 * Configuration bitmap purpose (1 means active action to do, 0 means not active):
 *   bits: |  ...  |    3   |   2   |     1   |    0    |
 *         |  ...  | ------ | ----- | ------- | ------- |
 *         |  ...  |   ...  |  ...  | suspend | resume  |
 *         |  ...  |   ...  |  ...  | profile | profile |
 *
 * So far, suspend/resume config will stop/re-start the L1 timer of current MAC profile if
 * concerned by the topic (BLIND, SATDET, ...)
 */
struct KNS_MAC_cfg_ctxt_t {
	union KNS_CFG_bitmap_t bitmap;
};

/**
 * @struct KNS_MAC_send_data_ctxt_t
 * @brief Event context structure containing usefull information for the APPlication to send data
 * to Kineis
 */
struct KNS_MAC_send_data_ctxt_t {
	uint8_t usrdata[KNS_MAC_USRDATA_MAXLEN];
	uint16_t usrdata_bitlen;
	enum  KNS_serviceFlag_t sf;
	uint32_t appId; /** APP layer's unique identifier of the message.
			 * The Kineis stack will reply this unique to its OH or ERROR
			 * reply, so that APP can know which message is concerned
			 */
};

/**
 * @struct KNS_MAC_rx_ctxt_t
 * @brief Event context information to start/stop RX
 * to Kineis
 */
struct KNS_MAC_rx_ctxt_t {
	enum KNS_MAC_rxMode_t mode; /** APP layer's unique identifier of the message. */
};

/**
 * @struct KNS_MAC_pos_ctxt_t
 * @brief Event context information regarding GNSS position
 */
struct KNS_MAC_pos_ctxt_t {
	float latitude;
	float longitude;
	float altitude;
};

/**
 * @struct KNS_MAC_appEvt_t
 * @brief Generic event structure with ID and context
 *
 * Union describing event contexts
 */
struct KNS_MAC_appEvt_t {
	enum KNS_MAC_appEvtId_t id;
	union {
		struct KNS_MAC_prflInfo_t init_prfl_ctxt;
		struct KNS_MAC_cfg_ctxt_t cfg_ctxt;
		struct KNS_MAC_send_data_ctxt_t data_ctxt;
		struct KNS_MAC_rx_ctxt_t rx_ctxt;
		struct KNS_MAC_pos_ctxt_t pos_ctxt;
	};
};

	/* ---- SRV-Cto-MAC events ---- */
/**
 * @struct KNS_MAC_sendDataReply_ctxt_t
 * @brief Event context structure containing useful information when stack
 * replies to send-data requests
 */
struct __attribute__((packed)) KNS_MAC_sendDataReply_ctxt_t {
	uint32_t appId; /** APP layer's unique identifier of the message.
					 * The Kineis stack will reply this unique to its OH or ERROR
					 * reply, so that APP can know which message is concerned
					 */
	int16_t frm_hdlr; // Tx frame handler in kineis stack FIFO
};

/**
 * @struct KNS_MAC_SATDET_ctxt_t
 * @brief SATellite DETected event context structure
 *
 * All useful parameters needed when reporting satellite detection
 */
struct __attribute__((packed)) KNS_MAC_SATDET_ctxt_t {
	uint32_t detected_freq;
	uint32_t detect_duration;
	float rssi;
	uint32_t satdet_patt_report;
	uint32_t nb_err_report;
};

/**
 * @struct KNS_MAC_TX_cplt_ctxt_t
 * @brief TX-done/TX-timeout event context structure
 *
 * All useful parameters needed when reporting TX is complete
 */
struct __attribute__((packed)) KNS_MAC_TX_cplt_ctxt_t {
	int16_t frm_hdlr; // Tx frame handler in kineis stack FIFO
};

/**
 * @struct KNS_MAC_RX_frm_ctxt_t
 * @brief RX-frame-received event context structure
 *
 * All useful parameters needed when some frame is received
 */
struct __attribute__((packed)) KNS_MAC_RX_frm_ctxt_t {
	uint8_t data[DL_FRM_SZ];
	uint16_t data_bitlen;
	float rssi;
	uint32_t detected_freq;
};

/**
 * @struct KNS_MAC_dl_bc_ctxt_t
 * @brief DL-message-received  event context structure
 *
 * All useful parameters needed when some frame is received
 */
struct __attribute__((packed)) KNS_MAC_RX_dl_msg_ctxt_t {
	uint8_t data[DL_FRM_SZ];
	uint16_t data_bitlen;
	uint16_t bc_mc;
};

/**
 * @struct KNS_MAC_srvcEvt_t
 * @brief Generic structure/union describing events coming from SERVICE layer
 *
 * This events are internally used by MAC layer. But @note most of them are converted and forwarded
 * to APP layer as notifications.
 *
 */
struct __attribute__((packed)) KNS_MAC_srvcEvt_t {
	enum KNS_MAC_srvcEvtId_t id;
	enum KNS_MAC_appEvtId_t app_evt; /**< original APP event which triggered this.
					  * Correctly set for KNS_MAC_OK and KNS_MAC_ERROR evt
					  */
	enum KNS_status_t status; /** Kineis stack status, may not be valid on all events */
	union {
		struct KNS_MAC_sendDataReply_ctxt_t sendreply_ctxt;
							/**< stackOK/ERROR reply to send-data requests */
		struct KNS_MAC_prflInfo_t init_prfl_ctxt; /**< MAC profile Initialization context */
		struct KNS_MAC_SATDET_ctxt_t satdet_ctxt; /**< satellite detection notification context */
		struct KNS_MAC_TX_cplt_ctxt_t tx_ctxt; /**< TX-complete notification context
							 * Also set in case of KNS_MAC_OK and
							 * KNS_MAC_ERROR evt
							 */
		struct KNS_MAC_RX_frm_ctxt_t rx_ctxt; /**< RX-frame-received notification context */
		struct KNS_MAC_RX_dl_msg_ctxt_t msg_ctxt; /**< decoded DL msg context */
	};
};

	/* ---- INFRA-to-MAC events ---- */

/**
 * @struct KNS_MAC_infraEvt_t
 * @brief Generic structure of events coming from INFRA (e.g. timer)
 *
 * Union describing event contexts
 */
struct KNS_MAC_infraEvt_t {
	enum KNS_MAC_infraEvtId_t id;
	union {
		uint32_t trigtx_ctxt; /**< TRIG_TX event context */
	};
};

#pragma GCC visibility pop

#endif /* KNS_MAC_EVT_H */
/**
 * @}
 */
