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

#include "tof_device.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "tof_twi.h"
#include "tof_fds.h"

#include "tof_VL53LX_states.h"

#define FILE_ID(cfg_cmd_id) (cfg_cmd_id & 0xF0)

#define FILE_ID_EXT_STORAGE 0x1001
#define RKEY_EXT_STORAGE 0x1115

static device_t device;

static void config_cmd_ext_handler(void);
static void tof_device_on_error_handler(error_t error);

void tof_device_init(void) {
	error_t error =  TOF_ERROR_NONE;
    /* Initialize the i2c interface */
	error = tof_twi_init();
	if(error){
		tof_device_on_error_handler(TOF_ERROR_NO_COMMS);
		return;
	}
    /* Initialize the fds interface */
	error = tof_fds_init();
	if(error){
		tof_device_on_error_handler(TOF_ERROR_FDS_INIT);
		return;
    }
    // Read in the external stored data, ignore error if it doesn't exist
    (void)tof_fds_read(FILE_ID_EXT_STORAGE, RKEY_EXT_STORAGE, (uint8_t*)&device.ext_data[0], sizeof(device.ext_data));
    // Initialize the short range sensor
    error = vl53lx_create(&device, SNSR_TYPE_VL53L4CD, TOF_SNSR_SHORT_RANGE, (I2C_ADDR_DEFAULT + 1), PIN_TOF_SHORT_XSHUT);
	if(error){
		tof_device_on_error_handler(error);
		return;
    }
    // Initialize the long range sensor
	error = vl53lx_create(&device, SNSR_TYPE_VL53L4CX, TOF_SNSR_LONG_RANGE, (I2C_ADDR_DEFAULT + 2), PIN_TOF_LONG_XSHUT);
	if(error){
		tof_device_on_error_handler(error);
		return;
    }
    // Set the default sensor on startup
    device.sensor = &device.sensors[TOF_SNSR_SHORT_RANGE];
    // Initialize device data
    device.id_selected = TOF_SNSR_SHORT_RANGE;
    device.config_cmd.trgt = CONFIG_TRGT_NA;
    device.is_ranging_enabled = false;
    device.is_debug_enabled = false;

    NRF_LOG_INFO("ToF initialized");
}

void tof_device_uninit(void) {
    tof_twi_uninit();
}

void tof_device_process(void) {
	// Set selected sensor id to the current if invalid
	if(device.id_selected >= NUM_TOF_SNSR){
		device.id_selected = device.sensor->id;
	}
    // Switch sensors only when the current sensor status is in Standby or Error
    // Note: Switching should be done here and not by the sensors in the event of sensor error
	if(device.sensor->id != device.id_selected
        && (TOF_STATUS_STANDBY == device.sensor->status || TOF_STATUS_ERROR == device.sensor->status)){

        device.sensor = &device.sensors[device.id_selected];
    }
    // Run current state
    if(device.sensor){
    	device.sensor->state();
    }
}

void config_cmd_message(config_cmd_data_t* config_cmd){
    NRF_LOG_INFO(">> trgt: %s, cmd: %s, cfg: %d, value: %d",
            get_trgt_str(config_cmd->trgt),
            get_cmd_str(config_cmd->cmd),
            config_cmd->id,
            config_cmd->value);
}

void config_resp_message(config_cmd_data_t* config_cmd){
    NRF_LOG_INFO("<< trgt: %s, cmd: %s, cfg: %d, value: %d, status: %s",
            get_trgt_str(config_cmd->trgt),
            get_cmd_str(config_cmd->cmd),
            config_cmd->id,
            config_cmd->value,
            get_status_str(config_cmd->status));
}

void tof_config_cmd_set(uint8_t trgt, uint8_t cmd, uint8_t id, int32_t value){
    device.config_cmd.trgt = trgt;
    device.config_cmd.cmd = cmd;
    device.config_cmd.id = id;
    device.config_cmd.value = value;
    // Set the pending flag
    device.config_pending = true;
}

void tof_process_config_cmd(void) {
    // Return if no configuration pending
    if (!device.config_pending)
        return;

    switch (device.config_cmd.trgt) {
        case CONFIG_TRGT_SNSR:
            return; // Handled by sensor state machine
        case CONFIG_TRGT_EXT:
            tof_handle_config_cmd(&config_cmd_ext_handler);
            return;
        default:
            device.config_cmd.cmd = CONFIG_CMD_NA;
            tof_handle_config_cmd(NULL);
            return;
    }
}

void tof_handle_config_cmd(config_cmd_handler_t cmd_handler){
    // Print the command message
    config_cmd_message(&device.config_cmd);
    // Run handler if not NULL
    if(cmd_handler){
        cmd_handler();
    }
    // Print the response message
    config_resp_message(&device.config_cmd);
    // Notify the user
    tof_data_callback(&device, TOF_DATA_CONFIG_COMMAND);
    // Reset the cmd target
    device.config_cmd.trgt = CONFIG_TRGT_NA;
    // Reset the pending flag
    device.config_pending = false;
}

void tof_sensor_select(sensor_id_t id) {
    if (device.sensor->id == id) {
        NRF_LOG_INFO("%s already selected", device.sensor->name);
        tof_data_callback(&device, TOF_DATA_SELECTED);
    }
    else if (id < NUM_TOF_SNSR) {
        device.id_selected = id;
        NRF_LOG_INFO("%s requested", device.sensors[id].name);
        // Let state machine notify of selected sensor when switching
    }
    else {
        NRF_LOG_INFO("invalid sensor id");
        tof_data_callback(&device, TOF_DATA_SELECTED);
    }
}

const device_t* tof_device_get(void) {
    return &device;
}

int32_t tof_sensor_cached_config_get(uint8_t config_id){
    if(config_id < device.sensor->num_configs){
        return device.sensor->config[config_id].value;
    }
    else{
        return INVALID_CONFIG_VALUE;
    }
}

void tof_sensor_debug_set(uint8_t value) {
    device.is_debug_enabled = value;
}

void tof_sensor_debug_set_ref(uint16_t distance_mm_ref) {
    device.distance_mm_ref = distance_mm_ref;
}

uint16_t tof_sensor_debug_get_ref(void) {
    return device.distance_mm_ref;
}

void tof_sensor_ranging_enable_set(uint8_t value) {
    if(device.is_ranging_enabled != value){
        if(value){
            device.is_ranging_enabled = 1;
            NRF_LOG_INFO("%s ranging enabled", device.sensor->name);
        }
        else{
            device.is_ranging_enabled = 0;
            NRF_LOG_INFO("%s ranging disabled", device.sensor->name);
        }
    }
    tof_data_callback(&device, TOF_DATA_SAMPLING_ENABLED);
}

void tof_sensor_reset(uint8_t reset_type){
    device.reset_cmd = reset_type;
}

const char* get_sensor_name_str(uint8_t snsr_type){
    switch(snsr_type){
        case SNSR_TYPE_VL53L4CD:
            return "VL53L4CD";
        case SNSR_TYPE_VL53L4CX:
            return "VL53L4CX";
        default:
            return "unknown type";
    }
}

const char* get_trgt_str(uint8_t trgt){
    switch (trgt) {
        case CONFIG_TRGT_SNSR:
            return "sensor";
        case CONFIG_TRGT_EXT:
            return "ext";
        default:
            return "unknown";
    }
}

const char* get_cmd_str(uint8_t cmd){
    switch (cmd) {
        case CONFIG_CMD_GET:
            return "get";
        case CONFIG_CMD_SET:
            return "set";
        case CONFIG_CMD_RESET:
            return "reset";
        case CONFIG_CMD_STORE:
            return "store";
        default:
            return "unknown";
    }
}

const char* get_status_str(uint8_t status){
    switch (status) {
        case CONFIG_STAT_OK:
            return "ok";
        case CONFIG_STAT_UPDATED:
            return "updated";
        case CONFIG_STAT_MISMATCH:
            return "mismatch";
        case CONFIG_STAT_ERROR:
            return "error";
        case CONFIG_STAT_NA:
            return "not available";
        default:
            return "unknown";
    }
}

const char* get_error_str(uint8_t error){
    switch (error) {
        case TOF_ERROR_NONE:
            return "no error";
        case TOF_ERROR_NO_COMMS:
            return "no comms";
        case TOF_ERROR_FDS_INIT:
            return "fds init";
        case TOF_ERROR_FDS_INTERNAL:
            return "fds internal";
        case TOF_ERROR_SENSOR_INDEX_OOR:
            return "sensor idx oor";
        case TOF_ERROR_SENSOR_CREATE:
            return "sensor create";
        case TOF_ERROR_SENSOR_INIT:
            return "sensor init";
        case TOF_ERROR_SENSOR_INTERNAL:
            return "sensor internal";
        default:
            return "unknown";
    }
}

/**
 * Handles configuration commands for getting/storing external data.
 * User can stored any uint32_t data up to an index id < MAX_STORAGE_DATA_BUFF_SIZE
 */
static void config_cmd_ext_handler(void) {
    uint8_t id = device.config_cmd.id;
    uint8_t cmd = device.config_cmd.cmd;
    int32_t value = device.config_cmd.value;

    device.config_cmd.status = CONFIG_STAT_OK;

    // If configuration id is not in list or index not allowed then exit function
    if (id >= MAX_STORAGE_DATA_BUFF_SIZE && cmd != CONFIG_CMD_STORE) {
        device.config_cmd.status = CONFIG_STAT_NA;
        return;
    }

    // Process the command
    switch (cmd) {
        case CONFIG_CMD_GET:
            device.config_cmd.value = device.ext_data[id];
            return;
        case CONFIG_CMD_SET:
            device.ext_data[id] = value;
            return;
        case CONFIG_CMD_RESET:
            // Maybe cache a default value in the future
            return;
        case CONFIG_CMD_STORE:
            tof_fds_write(FILE_ID_EXT_STORAGE, RKEY_EXT_STORAGE, (uint8_t*)&device.ext_data[0], sizeof(device.ext_data));
            return;
        default:
            device.config_cmd.status = CONFIG_STAT_INVALID;
            return;
    }
}

static void tof_device_on_error_handler(error_t error){
	NRF_LOG_INFO("ToF err: %s", get_error_str(error));
}
