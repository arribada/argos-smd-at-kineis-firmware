/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    mcu_mvm_blind_pos.h
 * @brief   MCU wrappers to handle Non volatile memory ressources used in MAC BLIND_POS profile
 * @author  Kineis
 */

/**
 * @page mcu_mvm_blind_pos_page MCU wrappers:
 *
 * This is about storing the resources used by the BLIND_POS MAC profile:
 * * PER map
 * * PER-to-BLINDcfg abacus
 * * device's UL performance indicator
 * Those settings are part of the credentials (ID, address, DSK) provided by Kineis operator.
 *
 * @attention It is up to you to manage the storing stategy of those values during the entire life
 * of your device (handle NVM erase cycles, ...)
 *
 * PER map format:
 * ===============
 * Such map describes kineis' devices UL message reception ratio depending on devices's position over
 * the globe.
 * This map describes some area through a max-distance-to-centers, and centers positions.
 * The number of areas is not fixed, it may be only one single zone up to a lot
 *
 * @todo PRODEV-222 May need to limit the maximum number of zones
 * @todo PRODEV-222 May need to limit the maximum number of center per zone
 * @todo PRODEV-222 Need to limit the maximum altitude (e.g. +600km?)
 * @todo PRODEV-222 Define the format of the map. Below is a proposal:
 * @verbatim
       float?: timestamp of the map UTC datetime (EPOCH?)
       uint8_t: Number of zone described (1..ZONE_MAX_NB)
       Sort zones from worst to best, for each zone:
           float: PER
           float: max distance to centers in km (from 0 to FLT_MAX), 0 means invalid zone
           uint8_t: Number of centers (1..CENTER_MAX_NB)
           For each center:
               float: latitude in ° (-90..+90)
               float: longitude in ° (-180..+180)
               float: altitude in km (-6371..+ALTITUDE_MAX)
           ...
       ...
   @endverbatim
 *
 * abacus format:
 * ==============
 * @verbatim
       For each PER ratio:
           uint32_t: latitude 0°  in °
           uint32_t: retx_nb (0..MAX)
           uint32_t: retx_period in s
       ...
   @endverbatim
 *
 * device UL performance indicator format:
 * =======================================
 * @verbatim
       float: PER performance indicator (probably an offset in %)
       ...
   @endverbatim
 */

/**
 * @addtogroup MCU_WRAPPERS
 * @brief MCU wrapper used for Non volatile memory management.
 * @{
 */

#ifndef MCU_NVM_BLIND_POS_H
#define MCU_NVM_BLIND_POS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "kns_types.h"

/* Function declaration -------------------------------------------------------------*/

/**
 * @brief Copy the "PER map" in NVM
 *
 * @param[in] ptr pointer to map described as in header
 * @param[in] byte_size map's size in byte
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t MCU_NVM_setPerMap(void *ptr, uint32_t byte_size);

/**
 * @brief Get the "PER map" stored in device
 *
 * @param[out] ptr pointer to the map stored in NVM when status is ok
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t MCU_NVM_getPerMap(void **ptr);

/**
 * @brief copy the latitude/PER to BLIND configuration abacus in NVM
 *
 * @param[in] ptr pointer to abacus as described in header when status is ok
 * @param[in] byte_size abacus size in byte
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t MCU_NVM_setPerToBlindAbacus(void *ptr, uint32_t byte_size);

/**
 * @brief Get abacus stored in device
 *
 * @param[out] ptr pointer to the abacus stored in NVM when status is ok
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t MCU_NVM_getPerToBlindAbacus(void **ptr);

/**
 * @brief store the device's calibration offset regarding UL PER
 *
 * This calibration offset is said to be deduced from Kineis certifications (.e.g kind of TRPK,
 * device certification, ...)
 *
 * @param[in] ulPerCal calibration offset value
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t MCU_NVM_setUlPerCalOffset(float ulPerCal);

/**
 * @brief Get device's calibration offset for UL PER
 *
 * This calibration offset is said to be deduce from Kineis certifications (.e.g kind of TRPK,
 * device certification, ...)
 *
 * @param[in] ulPerCal pointer to calibration offset value when sattus is ok
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t MCU_NVM_getUlPerCalOffset(float *ulPerCal);

#endif /* MCU_NVM_H */

/**
 * @}
 */
