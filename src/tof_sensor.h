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

#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include "boards.h"
#include <stdint.h>

#include "config_cmd.h"

/** Default i2c address for the ToF sensors */
#define I2C_ADDR_DEFAULT              (0x29)
/** Timeout before rebooting the sensor */
#define ERR_TIMEOUT_MS                APP_TIMER_TICKS(2500)
/** FDS defines for sensor configurations */
#define RKEY_SNSR_DATA_CAL            (0x5001)
#define RKEY_SNSR_DATA_USER           (0x5002)
#define MAX_SNSR_CONFIG_BUFF_SIZE     (20)
#define FILD_ID_SNSR_DATA(snsr_id)    (0x0500 | snsr_id)
/** Macro for calculating the sensor configurations storage size to FDS */
#define SNSR_CFG_STORAGE_SIZE(device) (device->sensor->num_configs * sizeof(int32_t))

typedef enum {
	TOF_SENSOR_TYPE_VL53L4CD = 0,
	TOF_SENSOR_TYPE_VL53L4CX,
    NUM_TOF_SENSOR_TYPES,
	TOF_SENSOR_TYPE_NA
}ztof_sensor_type_t;

/** Sensor error mask to merge
 * with the status */
typedef enum {
    TOF_SENSOR_ERR_NONE = 0,
    TOF_SENSOR_ERR_INVALID_TYPE,
    TOF_SENSOR_ERR_SENSOR_CREATE,
    TOF_SENSOR_ERR_SENSOR_INIT,
    TOF_SENSOR_ERR_SENSOR_INTERNAL,
    TOF_SENSOR_ERR_COMMS,
    NUM_SENSOR_ERR,
    TOF_SENSOR_ERR_NA
} tof_sensor_err_t;

/** High level statuses of the sensors
 * sent to the user */
typedef enum {
    TOF_SENSOR_STATUS_BOOTING = 0,
    TOF_SENSOR_STATUS_READY,
    TOF_SENSOR_STATUS_STANDBY,
    TOF_SENSOR_STATUS_ERROR,
    NUM_TOF_SENSOR_STATUS,
    TOF_SENSOR_STATUS_NA
} tof_sensor_status_t;

typedef enum {
    TOF_SENSOR_CONFIG_TYPE_PARAM = 0,
    TOF_SENSOR_CONFIG_TYPE_CAL,
    NUM_TOF_SENSOR_CONFIG_TYPES,
    TOF_SENSOR_CONFIG_TYPE_NA
} tof_sensor_config_type_t;

typedef enum {
    TOF_SENSOR_RESET_SENSOR = 0,      // Reset sensor and maintains stored configurations
    TOF_SENSOR_RESET_SENSOR_FACTORY,  // Reset sensor and clears stored configurations
    NUM_TOF_RESET_OPTIONS,
    TOF_RESET_NA
} tof_sensor_reset_cmd_t;

/** Data types from the sensors wrapper
 * sent to the user */
typedef enum {
    TOF_DATA_SELECTED = 0,
    TOF_DATA_STATUS,
    TOF_DATA_SAMPLING_ENABLED,
    TOF_DATA_DISTANCE,
    TOF_DATA_CONFIG,
    TOF_DATA_RESET,
    NUM_TOF_DATA_TYPE,
    TOF_DATA_NA
} snsr_data_type_t;

typedef struct {
    uint8_t type;
    int32_t value;
    int32_t value_default;
} tof_sensor_cfg_data_t;

typedef void (*stateHandler_t)(void);

typedef struct {
    char name[20];
    uint8_t type; 			// type
    uint8_t id; 			// id
    uint8_t address; 		// i2c address
    uint32_t pin_xshut; 	// xshut pin id
    uint8_t num_configs; 	// number of configurations
    stateHandler_t state;	// state
    uint8_t status; 		// status
    uint8_t error; 			// active error
    uint16_t distance_mm;   // distance
    uint16_t sample_count;  // distance sample count
    tof_sensor_cfg_data_t config[MAX_SNSR_CONFIG_BUFF_SIZE]; // configuration values
    void* context; 			// unique data
} tof_sensor_t;

typedef struct {
    uint8_t id_selected;
    uint8_t ranging_enabled;
    uint8_t debug_enabled;
    tof_sensor_reset_cmd_t reset_cmd;
    cfg_cmd_data_t config_cmd;
    uint8_t config_pending;
    tof_sensor_t* sensor;
}tof_sensor_handle_t;

/**
 * Callback used to pass sensor data
 */
void tof_sensor_data_callback(snsr_data_type_t type, void* context);

/**
 * Get the sensor name
 * @return string
 */
const char* tof_sensor_get_name_str(uint8_t snsr_type);

/**
 * Get the command status
 * @return string
 */
const char* tof_sensor_get_status_str(uint8_t status);

/**
 * Get the error status
 * @return string
 */
const char* tof_sensor_get_error_str(tof_sensor_err_t error);


#endif /* TOF_SENSOR_H */

