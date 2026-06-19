/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kns_rf.h
 * @brief   Device communication driver for Kinéis radio frequence
 * @author  William
 * @date    Creation 2022/08/09
 * @version 1.0
 * @note
 */

/**
 * @page kns_rf_page KNS_RF DRIVER
 *
 * This page is presenting the generic RF driver APIs for communication with
 * radio HW (\ref KNS_RF). This is a common abstraction layer to any RF chip used for Kineis.
 *
 * Depending on the HW platform (UL-only KIM1, UL/DL KIM2-x), this abstraction layer will forward
 * actions to the corresponding low-level-RF drivers.
 *
 * This driver provides functions to send or receive data to/from Kineis Transceiver.
 *
 * \note As our system requires some RF POWER for correct transmission to the SATS, it may be
 * usefull to driver an external PA through an external GPIO. According to its implementation
 * an integrator can tune its GPIOS through the MCU misc wrapper.
 *
 * @section kns_app_subpages Sub-pages
 *
 * * @subpage mcu_misc_page
 *
 */

/**
 * @addtogroup KNS_RF
 * @brief Kineis RF Transceiver module. (refer to @ref kns_rf_page for general description).
 * @{
 */

#ifndef KNS_RF_H
#define KNS_RF_H

/* Includes ------------------------------------------------------------------------------------ */

#include <stdbool.h>
#include <stdint.h>
#include "kns_types.h"

#pragma GCC visibility push(default)

/* Defines ------------------------------------------------------------------------------------- */

/* Enums --------------------------------------------------------------------------------------- */

/**
 * @enum KNS_RF_mode_t
 * @brief Modes for the transceiver
 */
enum KNS_RF_mode_t {
	IDLE,      /**< Go to idle */
	RX_DECODE, /**< Start continuous reception */
	RX_DETECT, /**< Satellite detection */
	TX         /**< Transmit one package  */
};

/**
 * @enum KNS_RF_rx_mod_t
 * @brief Reception modulation
 */
enum KNS_RF_rx_mod_t {
	RX_MOD_VD401  /**< demodulation VD401 Kineis */
};


/**
 * @enum KNS_RF_eventId_t
 * @brief List events which could be reported by this driver
 * @note This enum should concatenate to all event used by Kineis OS event dispatcher
 * @todo To Be moved to common Kineis OS event list
 */
enum KNS_RF_eventId_t {
	INIT_DONE,    /**< Power ON complete */
	DEINIT_DONE,  /**< Power OFF complete */
	TCXO_READY,   /**< TCXO warmup complete */
	TX_DONE,      /**< TX completed */
	TX_TIMEOUT,   /**< TX failed, timeout reached */
	RX_RECEIVED,  /**< RX frame received */
	SAT_DETECTED, /**< SATellite detection complete */
	SAT_LOST,    /**< SATellite signal lost */
	SAT_DETECT_TIMEOUT, /**< SATellite detection timeout */
	FREQ_DETECTED, /**< SATellite detection complete */
	RF_ABORTED    /**< RF action fully aborted */
};

/* Structures ---------------------------------------------------------------------------------- */

/**
 * @struct KNS_RF_rx_cfg_t
 * @brief RX configuration structure
 *
 * All parameters requested to define radio rx configuration.
 */
struct KNS_RF_rx_cfg_t {
	uint32_t frequency;           /**< Carrier center frequency to start demodulation */
	enum KNS_RF_rx_mod_t modulation; /**< Modulation to use for tx uplink msg */
};

/**
 * @struct KNS_RF_ctxt_tx_done_t
 * @brief TX-done event context structure
 *
 * All useful parameters needed when reporting TX is complete
 */
struct KNS_RF_ctxt_tx_done_t {
	uint8_t *bitstream;
	uint16_t bitstream_bitlen;
};

/**
 * @struct KNS_RF_ctxt_rx_received_t
 * @brief RX-received event context structure
 *
 * All useful parameters needed when reporting some frame is received
 */
struct KNS_RF_ctxt_rx_received_t {
	uint32_t dl_freq;
	uint32_t dl_rx_det_duration;
	float dl_mean_iq_mag;
	uint8_t *bitstream;	/**< Pointer on received frame
				  * @todo Pointed buffer should be configured by upper layer
				  * (RX FIFO?)
				  */
	uint16_t bitstream_bitlen;
};

/**
 * @struct KNS_RF_ctxt_sat_detected_t
 * @brief satellite-detected event context structure
 *
 * All useful parameters needed when a SAT is detected
 */
struct KNS_RF_ctxt_sat_detected_t {
	uint32_t dl_freq;
	uint32_t dl_rx_det_duration;
	float dl_mean_iq_mag;
	uint32_t satdet_patt_report;
	uint32_t nb_err_report;
};

/**
 * @struct KNS_RF_evt_t
 * @brief Generic event structure
 *
 * Union describing event reported by this driver
 */
struct KNS_RF_evt_t {
	enum KNS_RF_eventId_t id;
	union {
		struct KNS_RF_ctxt_tx_done_t tx_ctxt;
		struct KNS_RF_ctxt_sat_detected_t sdet_ctxt;
		struct KNS_RF_ctxt_rx_received_t rx_ctxt;
	};
};

/* Function prototypes ------------------------------------------------------------------------- */

/**
 * @brief This function powers on the LDO for the KNS_RF RX part.
 * @param[in] eop_isr_cb: Pointer on a callback which will be called to notify end of
 *            processing. Put ``void*`` in case no notification is needed.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFRX_powerOn(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

/**
 * @brief This function powers off the LDO for the KNS_RF RX part. cb is called when done
 * @param[in] eop_isr_cb: Pointer on a callback which will be called to notify end of
 *            processing. Put ``void*`` in case no notification is needed.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFRX_powerOff(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

/**
 * @brief This function fills the internal RX configuration structure for all RX RF parameters.
 * @param[in] rx_rf_cfg configuration struct, NULL if not applicable
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFRX_setCfg(
		struct KNS_RF_rx_cfg_t *rx_rf_cfg);

/**
 * @brief This function switch on the TCXO for the right time and callback is called when tcxo is
 *        considered ready.
 * @param[in] eop_isr_cb: Pointer on a callback which will be called to notify end of
 *            processing. Put ``void*`` in case no notification is needed.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFRX_tcxoWarmup(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

/**
 * @brief This function starts reception immediately and stores the received data into the rx buffer
 *
 * Each received frame is stored in a buffer, and upper layer is notified through registered
 * callback. Callback can be called multiple times if multiple RX data till abort or poweroff call.
 *
 * @param[in] eop_isr_cb: Pointer on a callback called when frame is received.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFRX_startImmediate(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

/**
 * @brief This function starts the detection satellite mechanism till detection ok or timeout or
 *        abort or power off called.
 * @param[in] eop_isr_cb: Function pointer on a callback which will be called to notify a
 *            satellite is detected.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFRX_startSatDetect(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

/**
 * @brief This function starts aborting the current RX RF processing part (SAT_detect, RX, ...)
 * @param[in] eop_isr_cb: function pointer on a callback called when processing is
 *        aborted and driver has been cleanly stopped.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFRX_abortRf(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

/**
 * @brief This function powers on the LDO for the KNS_RF TX part.
 * @param[in] eop_isr_cb: Pointer on a callback which will be called to notify end of
 *            processing. Put ``void*`` in case no notification is needed.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFTX_powerOn(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

/**
 * @brief This function powers off the LDO for the KNS_RF TX part. cb is called when done
 * @param[in] eop_isr_cb: Pointer on a callback which will be called to notify end of
 *            processing. Put ``void*`` in case no notification is needed.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFTX_powerOff(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

/**
 * @brief This function fills the internal TX configuration structure for all TX RF parameters.
 * @param[in] tx_rf_cfg configuration struct, NULL if not applicable
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFTX_setCfg(
		struct KNS_tx_rf_cfg_t *tx_rf_cfg);

/**
 * @brief This function switch on the TCXO for the right time and callback is called when tcxo is
 *        considered ready.
 * @param[in] eop_isr_cb: Pointer on a callback which will be called to notify end of
 *            processing. Put ``void*`` in case no notification is needed.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFTX_tcxoWarmup(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

/**
 * @brief This function loads the bitstream to send with the radio.
 *
 * A frame builder will build this bitstream including the overhead protocol. This function does not
 * send it.
 *
 * @param[in] buffer binary-data buffer containing bits to be transmitted
 * @param[in] bitsize number of bits to push (limited to uint16_t)
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFTX_pushBitstream(
		uint8_t *buffer, uint16_t bitsize);

/**
 * @brief This function starts the transmission immediately of the bitstream set into the tx buffer
 *        during the push_bitstream api.
 * @param[in] eop_isr_cb: Pointer on a callback which will be called to notify end of
 *            processing. Put ``void*`` in case no notification is needed.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFTX_startImmediate(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

/**
 * @brief This function starts aborting the current TX RF processing
 * @param[in] eop_isr_cb: function pointer on a callback called when processing is
 *        aborted and driver has been cleanly stopped.
 * @warning Callback may run from ISR context and break realtime consideration. Do not exceed 100us
 * @retval KNS_status_t status: KNS_STATUS_OK or ERROR
 */
enum KNS_status_t KNS_RFTX_abortRf(
		enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt));

#pragma GCC visibility pop

#endif /* KNS_RF_H */

/**
 * @}
 */
