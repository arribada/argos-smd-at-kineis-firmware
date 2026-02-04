/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kns_app.h
 * @brief   Provides main application software using Kineis SW stack
 * @author  Kineis
 */

/**
 * @page kns_app_page APP task
 *
 * This page is presenting two APPlication task example which exercices the Kineis stack SW
 * (\ref KNS_APP) in a way to send user data to the Kineis network..
 *
 * Two applications tasks are available:
 * * A standalone app (\ref KNS_APP_stdln_loop) is sending one user message (once or 4 retransmit each 60s)
 * * A GUI app (\ref KNS_APP_gui_loop) is dedicated to communicate with the Kinéis Device Interface
 * (KDI) through AT commands (\ref mgr_at_cmd_page) over an UART link (9600 bauds, 8, N, 1).
 *
 * Those application needs some configurations from user such as:
 * * Credentials (ID, ADDR, Secret key)
 * * Radio configuration (UL frequency and modulation)
 * * Kineis protocol configuration (a.k.a. MAC profile)
 * Refer to "getting started" section for details.
 *
 * In case of GUI APP, the format of an AT command is like:
 * * "<pattern>=?\r\n" (e.g. "AT+PING=?\r\n") to get information
 * * "<pattern>=<params>\r\n" (e.g. "AT+TX=<user_data>\r\n") to set information
 * The list of supported AT commands is available through \ref cas_atcmd_list_array.
 *
 * Communication between tasks (APP and MAC) in done through events posted in OS Queues.
 * By the end, those applications shows how to:
 * * Invoke the kineis stack to send USER DATA
 * * Wait notification from the Kineis stack once the transmission is completed
 *
 * @section kns_app_subpages Sub-pages
 *
 *  * @subpage mgr_at_cmd_page
 */

/**
 * @addtogroup KNS_APP
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef KNS_APP_H
#define KNS_APP_H

/* Functions prototypes ------------------------------------------------------*/

/** @brief This fct is a very basic standalone application
 *
 * This application is posting some send-data event to MAC layer of Kineis software stack in loop as
 * long as there is no error.
 *
 * Depending on the defined MAC protocol:
 * * BASIC: APP is trying to send 2 data at start but kineis stack can only accept the first one.
 * This data is sent once only, then is goes into lowpower mode.
 * * BLIND: APP is trying to send 5 data at start but 4 are accepted. Each data is re-transmitted
 * (retx) several times periodically as per the configuration of the protocol. Once a data is fully
 * transmitted, APP will try to send a new data again.
 */
void KNS_APP_stdln_loop(void);

/** @brief Init function needed for the application SW running in front of the GUI
 *
 * It initializes the AT command manager and Kineis MAC protocol
 *
 * This fct is expected to run once at at each uC's reset (start-up or LPM wakeup), and before the
 * KNS_APP_gui_loop fct is started/resumed.
 *
 * @param[in] context some handler on required HW setting.
 *            @note this handler will be directly sent as is, to MGR_AT_CMD and MCU wrappers. Thus
 *            application can change its type as per needs.
 */
void KNS_APP_gui_init(void *context);

/** @brief This fct is the application expected to run in front of the GUI
 *
 * This application is waiting for AT commands coming from GUI to perform some actions such as:
 * * Display the firmware version ("AT+FW=?\r\n")
 * * Display the ID ("AT+ID=?\r\n")
 * * Transmit some user data ("AT+TX=<USER DATA>\r\n")
 *
 * Refer to @ref mgr_at_cmd_page page to get supported AT commands.
 */
void KNS_APP_gui_loop(void);

#endif // end KNS_APP_H

/**
 * @}
 */
