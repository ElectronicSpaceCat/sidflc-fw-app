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

#ifndef TOF_DEVICE_MGR_H
#define TOF_DEVICE_MGR_H

#include "boards.h"
#include <stdint.h>
#include "tof_sensor.h"
#include "config_cmd.h"

/** FDS defines for user configurations*/
#define RKEY_USER_STORAGE           (0x1001)
#define FILE_ID_USER_STORAGE        (0x0101)
#define MAX_USER_CONFIG_BUFF_SIZE   (24)

typedef enum {
    TOF_DEV_MGR_SNSR_SHORT_RANGE = 0,
	TOF_DEV_MGR_SNSR_LONG_RANGE,
    NUM_TOF_DEV_MGR_SNSR,
    TOF_TOF_DEV_MGR_SNSR_NA
} tof_dev_mgr_sensor_id_t;

typedef enum {
	TOF_DEV_MGR_ERR_NONE = 0,
	TOF_DEV_MGR_ERR_COMMS_INIT,
	TOF_DEV_MGR_ERR_FDS_INIT,
	TOF_DEV_MGR_ERR_SENSOR_INIT,
	TOF_DEV_MGR_ERR_SENSOR_IDX_OOR,
	TOF_DEV_MGR_ERR_INTERNAL,
    NUM_TOF_DEV_MGR_ERR,
    TOF_DEV_MGR_ERR_NA
} tof_dev_mgr_err_t;

typedef enum {
	DEV_RESET_SYSTEM = 0,
	DEV_RESET_ACTIVE_SENSOR,
	DEV_RESET_ACTIVE_SENSOR_FACTORY,
    NUM_DEV_RESET_OPTIONS,
	DEV_RESET_NA
} dev_reset_options_t;

typedef enum {
    DEV_CFG_TRGT_SNSR = 0,
	DEV_CFG_TRGT_USER,
    NUM_DEV_CFG_TRGTS,
	DEV_CFG_TRGT_NA
} dev_cfg_trgt_t;

typedef struct {
	uint8_t trgt;
	cfg_cmd_data_t cfg;
}dev_cfg_cmd_t;

/**
 * Callback used to pass configuration command data
 */
void tof_dev_cfg_cmd_callback(dev_cfg_cmd_t* config_cmd);

/**
 * Initialize the device
 */
void tof_dev_mgr_init(void);

/**
 * Un-Initialize the device
 */
void tof_dev_mgr_uninit(void);

/**
 * Process the device handler
 */
void tof_dev_mgr_process(void);

/**
 * Select sensor
 * @param snsr_id
 */
void tof_dev_mgr_sensor_select(tof_dev_mgr_sensor_id_t snsr_id);

/**
 * Get the sensor handle data
 * @return
 */
const tof_sensor_handle_t* tof_dev_mgr_shandle_get(void);

/**
 * Debug: Get a configuration value
 * @param trgt
 * @param config_id
 * @return config value
 */
int32_t tof_dev_mgr_get_cached_config(dev_cfg_trgt_t trgt, uint8_t config_id);

/**
 * Enable/Disable range sampling
 * @param value
 */
void tof_dev_mgr_set_ranging_enable(uint8_t value);

/**
 * Debug: Enable/Disable debug mode
 * @param value
 */
void tof_dev_mgr_set_debug_enable(uint8_t value);

/**
 * Reset current sensor
 */
void tof_dev_mgr_sensor_reset(uint8_t reset_type);

/**
 * @brief Sets the configuration command data
 * @param trgt
 * @param cmd
 * @param id
 * @param value
 */
void tof_dev_mgr_set_cfg_cmd(uint8_t trgt, uint8_t cmd, uint8_t id, int32_t value);

/**
 * Sets a flag indicating to run the main process of the device manager
 */
void tof_dev_mgr_set_run_signal(void);

#endif /* TOF_DEVICE_MGR_H */
