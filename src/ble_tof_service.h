/*******************************************************************************
    SIDFLC (Smartphone Interfaced Device For Launching Coins)
    Copyright (C) 2021-present Andrew Green
 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
 
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
 
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 
 *******************************************************************************/

#ifndef BLE_TOF_SERVICE_H__
#define BLE_TOF_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "tof_device_mgr.h"

// TODO: AG - Make sure to generate unique 128-bit UUID here
// Defining 128-bit base UUIDs
#define BLE_UUID_BASE_TOF                           \
{{                                                  \
    0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, \
    0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00  \
}}

#ifndef BLE_TOF_BLE_OBSERVER_PRIO
#define BLE_TOF_BLE_OBSERVER_PRIO 2
#endif

#define BLE_TOF_DEF(_name)                          \
static ble_tof_t _name;                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                 \
                     BLE_TOF_BLE_OBSERVER_PRIO,     \
                     ble_tof_on_ble_evt,            \
                     &_name)

#define BLE_UUID_TOF_SERVICE             0xF00D

// Defining 16-bit characteristic UUID
// TODO: AG - Make sure the 16-bit char UUIDs do not conflict with registered ones
#define BLE_UUID_TOF_RANGE               0xBEAA
#define BLE_UUID_TOF_SELECT              0xBEAB
#define BLE_UUID_TOF_CONFIG              0xBEAC
#define BLE_UUID_TOF_STATUS              0xBEAD
#define BLE_UUID_TOF_RANGING_ENABLE      0xBEAE
#define BLE_UUID_TOF_RESET               0xBEAF

// Forward declaration of the ble_tof_t type.
typedef struct ble_tof_s ble_tof_t;

typedef void (*ble_tof_evt_handler_t) (ble_tof_t* p_tof, ble_evt_t const *p_ble_evt);

typedef void (*ble_tof_write_handler_t) (uint16_t conn_handle, uint8_t data);
typedef void (*ble_tof_write_config_handler_t) (uint16_t conn_handle, uint8_t trgt, uint8_t cmd, uint8_t id, int32_t value);

typedef struct
{
    ble_tof_evt_handler_t            evt_handler;
    ble_tof_write_handler_t          tof_select_write_handler;
    ble_tof_write_config_handler_t   tof_config_write_handler;
    ble_tof_write_handler_t          tof_ranging_enable_write_handler;
    ble_tof_write_handler_t          tof_reset_write_handler;
} ble_tof_init_t;

// This structure contains various status information for our service.
// The name is based on the naming convention used in Nordics SDKs.
// 'ble’ indicates that it is a Bluetooth Low Energy relevant structure.
struct ble_tof_s
{
    ble_tof_evt_handler_t          evt_handler;
    uint16_t                       conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                       service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    // ToF range handles
	ble_gatts_char_handles_t       tof_range_char_handles;
    // ToF sensor select handles
    ble_gatts_char_handles_t       tof_select_char_handles;
    ble_tof_write_handler_t        tof_select_write_handler;
    // ToF sensor config handles
    ble_gatts_char_handles_t       tof_config_char_handles;
    ble_tof_write_config_handler_t tof_config_write_handler;
    // ToF sensor status handles
    ble_gatts_char_handles_t       tof_status_char_handles;
    // ToF ranging_enable handles
    ble_gatts_char_handles_t       tof_ranging_enable_char_handles;
    ble_tof_write_handler_t        tof_ranging_enable_write_handler;
    // ToF sensor reset
    ble_gatts_char_handles_t       tof_reset_char_handles;
    ble_tof_write_handler_t        tof_reset_write_handler;
};

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_tof_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_tof       Pointer to Our Service structure.
 */
ret_code_t ble_tof_init(ble_tof_t *p_tof, const ble_tof_init_t * p_tof_init);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof                    Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_select_characteristic_update(ble_tof_t *p_tof, uint8_t sensor, uint8_t sensor_type);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof                    Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_config_characteristic_update(ble_tof_t *p_tof, dev_cfg_cmd_t *config_cmd);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof                    Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_status_characteristic_update(ble_tof_t *p_tof, uint8_t *tof_status_value);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof                    Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_range_characteristic_update(ble_tof_t *p_tof, uint16_t *tof_range_value);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof                    Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_ranging_enable_characteristic_update(ble_tof_t *p_tof, uint8_t *tof_sampling_enabled);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof                    Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_reset_characteristic_update(ble_tof_t *p_tof, uint8_t *value);

/**@brief Function for updating and sending hvx Indications
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof                    Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_hvx_gatts_queue_process(void);

void tof_gatts_hvx_debug_enable(void);

#endif  /* _ BLE_TOF_SERVICE_H__ */
