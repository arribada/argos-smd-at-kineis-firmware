/**
 * @file    KNS_types.h
 * @brief   Kineis types definition
 * @author  Kineis
 * @date    creation 2022/07/01
 * @version 1.0
 * @note
 */

/**
 * @addtogroup KNS_TYPES
 * @brief Kineis generic types used in most of the kineis SW stack functions
 * @{
 */

#ifndef KNS_TYPES_H
#define KNS_TYPES_H

#include <stdint.h>

#pragma GCC visibility push(default)

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Enums --------------------------------------------------------------------------------------- */

/**
 * @enum KNS_status_t
 * @brief Status codes returned by driver. All KNS_xxx functions returns this status type.
 */
enum KNS_status_t {
	// general status reports
	KNS_STATUS_OK             = 0  , /**< status OK */
	KNS_STATUS_ERROR,                /**< status generic ERROR, when other one does not fit */
	KNS_STATUS_DISABLED,             /**< module/feature is disabled (transceiver, bus, ... */
	KNS_STATUS_BUSY,                 /**< module is busy (transceiver, bus, ... */
	KNS_STATUS_TIMEOUT,              /**< some SATDET, TX, RX, timeout reached */
	KNS_STATUS_BAD_LEN,              /**< TX data frame length error */
	KNS_STATUS_BAD_SETTING,          /**< wrong settings: unknown event, static cfg overflow, ... */
	KNS_STATUS_ABORT,                /**< some SATDET, TX, RX, abort reached (e.g. FIFO flush) */

	// RF errors
	KNS_STATUS_RF_ERR         = 100, /**< Error on transceiver (TX, RX, SPI/A2S/... error) */

	// RX/TX service errors
	KNS_STATUS_DL_FRM_ERR     = 150, /**< warning as some frame is received during a strange
				          * protocol sequence
				          */

	// Internal erros
	KNS_STATUS_INTERNAL_1_ERR = 200, /**< Kineis stack internal#1 error */
	KNS_STATUS_INTERNAL_2_ERR,       /**< Kineis stack internal#2 error */
	KNS_STATUS_INTERNAL_3_ERR,       /**< Kineis stack internal#3 error */
	KNS_STATUS_INTERNAL_4_ERR,       /**< Kineis stack internal#4 error */
	KNS_STATUS_INTERNAL_5_ERR,       /**< Kineis stack internal#5 error */
	KNS_STATUS_INTERNAL_6_ERR,       /**< Kineis stack internal#6 error */
	KNS_STATUS_INTERNAL_7_ERR,       /**< Kineis stack internal#7 error */

	// config errors
	KNS_STATUS_RADIO_CONF_BAD = 250, /**< radio configuration is wrong */
	KNS_STATUS_RADIO_CONF_MISSING,   /**< radio configuration is not set */
	KNS_STATUS_CREDENTIAL_ERROR,     /**< credentials are wrong */
	KNS_STATUS_KMAC_CONF_BAD,        /**< trying to TX/RX while MAC profile is able to do it */

	// OS errors
	KNS_STATUS_QFULL          = 300, /**< queue is full, FIFO is full, ... */
	KNS_STATUS_QEMPTY,               /**< queue is empty, FIFOis empty, ... */

	// MCU wrappers errors
	KNS_STATUS_MCU_AES_ERR    = 400, /**< for AES wrapper if existing */

	KNS_STATUS_MCU_DELAY_ERR  = 450, /**< for Delay wrapper if existing */

	KNS_STATUS_MCU_MISC_ERR   = 500, /**< for miscellaneous wrapper if existing */

	KNS_STATUS_MCU_NVM_ERR    = 550, /**< for NVM wrapper if existing */

	KNS_STATUS_MCU_RF_ERR     = 600, /**< for RF wrapper if existing */

	KNS_STATUS_MCU_TIM_ERR    = 650, /**< for timer wrapper if existing */

	KNS_STATUS_FLASH_ERR      = 700, /**< Error with FLASH usage (Arribada local addition,
					  *  preserved across v11.1.0 to keep existing AT
					  *  +ERROR=700 responses stable for the GUI). */

	KNS_STATUS_MAX            = 1000, /**< Max limit for kineis status enum, DO NOT EXCEED */
};

/**
 * @enum KNS_serviceFlag_t
 * @brief Kineis frames's service flag used in UL frames, also used as attribute to user data.
 */
enum KNS_serviceFlag_t {
	KNS_SF_NO_SERVICE                = 0b000, /**< No specific service, one-way UL frame */
	KNS_SF_MAIL_REQUEST              = 0b001, /**< UL frame asking for any DL beacon request? */
	KNS_SF_BC_UNICAST_ACK            = 0b010, /**< UL frame ACKnowledging a unicast beacon cmd */
	KNS_SF_BC_MULTICAST_ACK          = 0b011, /**< UL frame ACKnowledging a multicast beacon cmd */
	KNS_SF_PACK_NORMAL_REQUEST       = 0b100, /**< UL frame requesting a DL ACKnowledgment */
	KNS_SF_PACK_EMERGENCY_REQUEST    = 0b101  /**< UL frame requesting a DL emergency ACK */
};

/**
 * @enum KNS_satServiceFlag_t
 * @brief Kineis satellite service flag used in DL frames.
 */
enum KNS_satServiceFlag_t {
	KNS_SAT_SF_SYS_BC         = 0b00, /**< DL system beacon command */
	KNS_SAT_SF_USER_BC        = 0b01, /**< DL user beacon command */
	KNS_SAT_SF_POSITIVE_ACK   = 0b10, /**< DL frame ACKnowledging an UL frame */
	KNS_SAT_SF_SPARE          = 0b11, /**< Spare : Can be used for allcast messages */
};

/**
 * @enum KNS_tx_mod_t
 * @brief Transmission modulation
 *
 */
enum KNS_tx_mod_t {
	KNS_TX_MOD_NONE  = 0,  /**< no TX generation */
	KNS_TX_MOD_CW    = 1,  /**< continuous wave */
	KNS_TX_MOD_LDA2  = 2,  /**< modulation A2 (with Kineis codec) */
	KNS_TX_MOD_LDA2L = 3,  /**< modulation A2 Legacy (28-bit ID)*/
	KNS_TX_MOD_VLDA4 = 4,  /**< modulation VLDA4 (with Kineis codec) */
	KNS_TX_MOD_HDA4  = 5,  /**< modulation HDA4 (with Kineis codec) */
	KNS_TX_MOD_LDK   = 6   /**< modulation LDK */
};

/**
 * @struct KNS_rf_tx_cfg_t
 * @brief TX configuration structure
 *
 * All parameters requested to define radio tx configuration.
 */
struct KNS_tx_rf_cfg_t {
	uint32_t center_freq;         /**< Carrier uplink center frequency to transmit */
	enum KNS_tx_mod_t modulation; /**< Modulation to use for tx uplink msg */
	int8_t power;
};

#pragma GCC visibility pop

#endif /* end of KNS_TYPES_H */

/**
 * @}
 */
