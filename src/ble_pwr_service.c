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

#include <stdint.h>
#include <string.h>
#include "ble_pwr_service.h"

#include "pwr_mgr.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

static uint8_t debug_enabled = false;
static uint8_t pwr_source_enabled = false;
static uint8_t pwr_batt_status_enabled = false;
static uint8_t pwr_batt_level_enabled = false;

static void characteristic_update(ble_pwr_t *p_service, ble_gatts_char_handles_t* char_handle, uint8_t type, uint8_t *data, size_t data_len);
static void characteristic_set(ble_pwr_t *p_service, ble_gatts_char_handles_t *char_handle, uint8_t *data, size_t data_len, uint16_t offset);

static void check_cccds_enabled(ble_gatts_evt_write_t const *p_evt_write, ble_pwr_t *p_pwr, ble_evt_t const *p_ble_evt) {
    if(p_evt_write->len != 2) return;

    if (p_evt_write->handle == p_pwr->pwr_source_char_handles.cccd_handle) {
        pwr_source_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
    }
    if (p_evt_write->handle == p_pwr->pwr_batt_status_char_handles.cccd_handle) {
        pwr_batt_status_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
    }
    if (p_evt_write->handle == p_pwr->pwr_batt_level_char_handles.cccd_handle) {
        pwr_batt_level_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
    }
}

static void on_write(ble_pwr_t *p_pwr, ble_evt_t const *p_ble_evt) {
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // Check the cccds enable status
    check_cccds_enabled(p_evt_write, p_pwr, p_ble_evt);
}

// Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_pwr_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
    ble_pwr_t *p_pwr = (ble_pwr_t*) p_context;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            p_pwr->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            pwr_source_enabled = false;
            pwr_batt_status_enabled = false;
            pwr_batt_level_enabled = false;

            p_pwr->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_pwr, p_ble_evt);
            break;
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            break;
        default:
            // No implementation needed
            break;
    }

    // Call application event handler.
    if(p_pwr->evt_handler != NULL){
        p_pwr->evt_handler(p_pwr, p_ble_evt);
    }
}

/**@brief Function for adding our new characteristic to "Our service" that we initiated in the previous tutorial.
 *
 * @param[in]   p_pwr       Our Service structure.
 *
 */
static uint32_t our_char_add(ble_pwr_t *p_pwr, uint16_t uuid, bool read, bool write, uint8_t type, uint16_t value_size, uint32_t value_init, ble_gatts_char_handles_t* char_handle) {
  // Add a custom characteristic UUID
  uint32_t err_code;
  ble_uuid_t char_uuid;
  ble_uuid128_t base_uuid = BLE_UUID_BASE_PWR;
  char_uuid.uuid = uuid;
  err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
  APP_ERROR_CHECK(err_code);

  // Add read/write properties to our characteristic
  ble_gatts_char_md_t char_md;
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = read;
  char_md.char_props.write = write;

  // Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
  ble_gatts_attr_md_t cccd_md;
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  char_md.p_cccd_md = &cccd_md;
  if(type == BLE_GATT_HVX_NOTIFICATION){
      char_md.char_props.notify = true;
  }
  if(type == BLE_GATT_HVX_INDICATION){
      char_md.char_props.indicate = true;
  }

  // Configure the attribute metadata
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;

  // Set read/write security levels to our characteristic
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  // Configure the characteristic value attribute
  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;

  // Set characteristic length in number of bytes
  attr_char_value.max_len = value_size;
  attr_char_value.init_len = value_size;
  uint8_t buff[value_size];
  int32_t value = value_init;
  memcpy(&buff[0], &value, value_size);
  attr_char_value.p_value = &buff[0];

  // Add our new characteristic to the service
  err_code = sd_ble_gatts_characteristic_add(p_pwr->service_handle, &char_md, &attr_char_value, char_handle);

  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
ret_code_t ble_pwr_init(ble_pwr_t *p_pwr, const ble_pwr_init_t *p_pwr_init) {
  uint32_t err_code; // Variable to hold return codes from library and softdevice functions

  // Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
  ble_uuid_t service_uuid;
  ble_uuid128_t base_uuid = BLE_UUID_BASE_PWR;
  service_uuid.uuid = BLE_UUID_PWR_SERVICE;

  p_pwr->evt_handler = p_pwr_init->evt_handler;
  p_pwr->is_notification_supported = p_pwr_init->support_notification;

  err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
  APP_ERROR_CHECK(err_code);

  // Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
  p_pwr->conn_handle = BLE_CONN_HANDLE_INVALID;

  // Add our service
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                      &service_uuid,
                                      &(p_pwr->service_handle));

  APP_ERROR_CHECK(err_code);

  // Call the function our_char_add() to add our new characteristic to the service.
  our_char_add(p_pwr, BLE_UUID_PWR_SOURCE,      true, false, BLE_GATT_HVX_NOTIFICATION, sizeof(uint8_t),  tof_pwr_get_mngt_data()->pwr_source,   &p_pwr->pwr_source_char_handles);
  our_char_add(p_pwr, BLE_UUID_PWR_BATT_STATUS, true, false, BLE_GATT_HVX_NOTIFICATION, sizeof(uint8_t),  tof_pwr_get_mngt_data()->batt_status,   &p_pwr->pwr_batt_status_char_handles);
  our_char_add(p_pwr, BLE_UUID_PWR_BATT_LEVEL,  true, false, BLE_GATT_HVX_NOTIFICATION, sizeof(uint32_t), tof_pwr_get_mngt_data()->batt_lvl_milli_volts, &p_pwr->pwr_batt_level_char_handles);

  return err_code;
}

// Update power source data, notify if characteristic is enabled
void pwr_source_characteristic_update(ble_pwr_t *p_pwr, uint8_t *pwr_source_value) {
    if(pwr_source_enabled){
        characteristic_update(p_pwr, &p_pwr->pwr_source_char_handles, BLE_GATT_HVX_NOTIFICATION, (uint8_t*)pwr_source_value, sizeof(uint8_t));
    }
    else{
        characteristic_set(p_pwr, &p_pwr->pwr_source_char_handles, (uint8_t*) pwr_source_value, sizeof(uint8_t), 0);
    }
}
// Update batt status data, notify if characteristic is enabled
void pwr_batt_status_characteristic_update(ble_pwr_t *p_pwr, uint8_t *batt_status_value) {
    if(pwr_batt_status_enabled){
        characteristic_update(p_pwr, &p_pwr->pwr_batt_status_char_handles, BLE_GATT_HVX_NOTIFICATION, (uint8_t*)batt_status_value, sizeof(uint8_t));
    }
    else{
        characteristic_set(p_pwr, &p_pwr->pwr_batt_status_char_handles, (uint8_t*) batt_status_value, sizeof(uint8_t), 0);
    }
}
// Update batt level data, notify if characteristic is enabled
void pwr_batt_level_characteristic_update(ble_pwr_t *p_pwr, uint32_t *batt_level) {
    if(pwr_batt_level_enabled){
        characteristic_update(p_pwr, &p_pwr->pwr_batt_level_char_handles, BLE_GATT_HVX_NOTIFICATION, (uint8_t*)batt_level, sizeof(uint32_t));
    }
    else{
        characteristic_set(p_pwr, &p_pwr->pwr_batt_level_char_handles, (uint8_t*) batt_level, sizeof(uint32_t), 0);
    }
}

static void characteristic_update(ble_pwr_t *p_service, ble_gatts_char_handles_t* char_handle, uint8_t type, uint8_t *data, size_t data_len) {
  if(p_service->conn_handle != BLE_CONN_HANDLE_INVALID) {
    uint16_t len = data_len;
    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = char_handle->value_handle;
    hvx_params.type = type;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = data;

    uint32_t status = sd_ble_gatts_hvx(p_service->conn_handle, &hvx_params);

    // Print debug message if flag is enabled
    if (debug_enabled) {
        NRF_LOG_INFO("pwr hvx-i - handle: 0x%X, status: 0x%X", char_handle->value_handle, status);
    }
  }
}

static void characteristic_set(ble_pwr_t *p_service, ble_gatts_char_handles_t *char_handle, uint8_t *data, size_t data_len, uint16_t offset){
    ble_gatts_value_t p_value = {
            .len = data_len,
            .offset = offset,
            .p_value = data
    };
    sd_ble_gatts_value_set(p_service->conn_handle, char_handle->value_handle, &p_value);
}

void pwr_gatts_hvx_debug_enable(void) {
	debug_enabled = debug_enabled ? 1 : 0;
    if (debug_enabled) {
    	NRF_LOG_INFO(0, "pwr hvx - debug enable");
    }
    else {
    	NRF_LOG_INFO(0, "pwr hvx - debug disable");
    }
}
