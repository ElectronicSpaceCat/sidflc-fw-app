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

#include "tof_device_mgr.h"
#include "fds_mgr.h"
#include "tof_twi.h"
#include "tof_vl53lx.h"
#include "config_cmd.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

static tof_sensor_handle_t shandle;
static tof_sensor_t sensors[NUM_TOF_DEV_MGR_SNSR] = {
    {
        .type = TOF_SENSOR_TYPE_VL53L4CD,
        .id = TOF_DEV_MGR_SNSR_SHORT_RANGE,
        .address = (I2C_ADDR_DEFAULT + 1),
        .pin_xshut = PIN_TOF_SHORT_XSHUT
    },
    {
        .type = TOF_SENSOR_TYPE_VL53L4CX,
        .id = TOF_DEV_MGR_SNSR_LONG_RANGE,
        .address = (I2C_ADDR_DEFAULT + 2),
        .pin_xshut = PIN_TOF_LONG_XSHUT
    }
};

static struct cfg_cmd_internal_t {
    dev_cfg_cmd_t d;
    uint8_t pending;
} cfg_cmd = {
    .pending = false
};

static int32_t user_fds_data[MAX_USER_CONFIG_BUFF_SIZE];
static bool initialized = false;
static bool should_run = false;

static const char*tof_dev_mgr_get_error_str (tof_dev_mgr_err_t error);
static void tof_dev_mgr_on_error_handler (tof_dev_mgr_err_t error);
static tof_dev_mgr_err_t tof_dev_mgr_sensor_init (const tof_sensor_handle_t *shandle, tof_sensor_t *sensor);
static void tof_dev_mgr_process_cfg_cmd (void);
static void tof_dev_set_user_cfg (cfg_cmd_data_t *config_cmd);

/**
 * Initialize flash-data-storage (FDS), two-wire-interface (TWI - i2c), and the time-of-flight (ToF) sensors
 */
void tof_dev_mgr_init(void) {
    tof_dev_mgr_err_t error = TOF_DEV_MGR_ERR_NONE;
    /* Initialize the i2c interface */
    error = tof_twi_init ();
    if (error) {
        tof_dev_mgr_on_error_handler (TOF_DEV_MGR_ERR_COMMS_INIT);
        return;
    }
    /* Initialize the fds interface */
    error = fds_mgr_init ();
    if (error) {
        tof_dev_mgr_on_error_handler (TOF_DEV_MGR_ERR_FDS_INIT);
        return;
    }
    // Read in the external stored data, ignore error if it doesn't exist
    (void) fds_mgr_read (FILE_ID_USER_STORAGE, RKEY_USER_STORAGE, (uint8_t*) &user_fds_data[0], sizeof(user_fds_data));
    // Initialize sensors
    for (uint8_t i = 0; i < NUM_TOF_DEV_MGR_SNSR; ++i) {
        error = tof_dev_mgr_sensor_init (&shandle, &sensors[i]);
        if (error) {
            tof_dev_mgr_on_error_handler (TOF_DEV_MGR_ERR_SENSOR_INIT);
            return;
        }
    }
    // Initialize device data
    shandle.id_selected = TOF_DEV_MGR_SNSR_SHORT_RANGE;
    shandle.ranging_enabled = false;
    shandle.debug_enabled = false;
    shandle.sensor = &sensors[shandle.id_selected];

    initialized = true;

    NRF_LOG_INFO("ToF initialized");
}

void tof_dev_mgr_uninit(void) {
    tof_twi_uninit ();
}

/**
 * Process the device manager.
 * Switch sensors when ready and process configuration requests.
 */
void tof_dev_mgr_process(void) {
    // Make sure manager is initialized
    if (!initialized || !should_run) {
        return;
    }
    // Make sure selected sensor id is valid
    if (shandle.id_selected >= NUM_TOF_DEV_MGR_SNSR) {
        shandle.id_selected = TOF_DEV_MGR_SNSR_SHORT_RANGE;
    }
    // Make sure selected sensor is valid
    if (!shandle.sensor) {
        shandle.sensor = &sensors[shandle.id_selected];
    }
    // Is selected sensor is different than the active sensor?
    if (shandle.sensor != &sensors[shandle.id_selected]) {
        // Yes - Switch sensor only when the active sensor status is in Standby or Error
        switch (shandle.sensor->status) {
            case TOF_SENSOR_STATUS_STANDBY:
            case TOF_SENSOR_STATUS_ERROR:
                shandle.sensor = &sensors[shandle.id_selected];
                break;
            default:
                break;
        }
    }
    // Run current state
    shandle.sensor->state();
    // Process configuration commands
    tof_dev_mgr_process_cfg_cmd();

    should_run = false;
}

/**
 * Sets a configuration request for a given target.
 * @param trgt
 * @param cmd
 * @param id
 * @param value
 */
void tof_dev_mgr_set_cfg_cmd(uint8_t trgt, uint8_t cmd, uint8_t id, int32_t value) {
    switch (trgt) {
        case DEV_CFG_TRGT_SNSR:
            shandle.config_cmd.cmd = cmd;
            shandle.config_cmd.id = id;
            shandle.config_cmd.value = value;
            // Set flag to trigger sensor state machine to process the command
            shandle.config_pending = true;
            return;
        default:
            cfg_cmd.d.trgt = trgt;
            cfg_cmd.d.cfg.cmd = cmd;
            cfg_cmd.d.cfg.id = id;
            cfg_cmd.d.cfg.value = value;
            cfg_cmd.pending = true;
            return;
    }
}

/**
 * Select requested sensor. The sensor state machine will notify the user
 * when the requested sensor is active.
 * If the the sensor is the same as requested, the user will be immediately notified.
 * @param id
 */
void tof_dev_mgr_sensor_select(tof_dev_mgr_sensor_id_t id) {
    if (shandle.sensor->id == id) {
        NRF_LOG_INFO("%s already selected", shandle.sensor->name);
        tof_sensor_data_callback (TOF_DATA_SELECTED, shandle.sensor);
    }
    else if (id < NUM_TOF_DEV_MGR_SNSR) {
        shandle.id_selected = id;
        NRF_LOG_INFO("%s requested", sensors[id].name);
    }
    else {
        NRF_LOG_INFO("invalid sensor id");
        tof_sensor_data_callback (TOF_DATA_SELECTED, shandle.sensor);
    }
}

/**
 * Get the sensor handle.
 * @return
 */
const tof_sensor_handle_t* tof_dev_mgr_shandle_get(void) {
    return &shandle;
}

/**
 * Get the cached configurations per target.
 * @param trgt
 * @param config_id
 * @return
 */
int32_t tof_dev_mgr_get_cached_config(dev_cfg_trgt_t trgt, uint8_t config_id) {
    switch (trgt) {
        case DEV_CFG_TRGT_SNSR:
            if (config_id < shandle.sensor->num_configs) {
                return shandle.sensor->config[config_id].value;
            }
            break;
        case DEV_CFG_TRGT_USER:
            if (config_id < MAX_USER_CONFIG_BUFF_SIZE) {
                return user_fds_data[config_id];
            }
            break;
        default:
            break;
    }

    return INVALID_CONFIG_VALUE;
}

/**
 * Enable/Disable sensor debugging, which outputs the ranging data.
 * @param value
 */
void tof_dev_mgr_set_debug_enable(uint8_t value) {
    shandle.debug_enabled = value ? 1 : 0;
    if (shandle.debug_enabled) {
        NRF_LOG_INFO("%s range debug enable", shandle.sensor->name);
    }
    else {
        NRF_LOG_INFO("%s range debug disable", shandle.sensor->name);
    }
}

/**
 * Enable/Disable the sensor ranging and then sends a notification.
 * @param value
 */
void tof_dev_mgr_set_ranging_enable(uint8_t value) {
    shandle.ranging_enabled = value ? 1 : 0;
    if (shandle.ranging_enabled) {
        NRF_LOG_INFO("%s ranging enabled", shandle.sensor->name);
    }
    else {
        NRF_LOG_INFO("%s ranging disabled", shandle.sensor->name);
    }
    tof_sensor_data_callback (TOF_DATA_SAMPLING_ENABLED, &shandle.ranging_enabled);
}

/**
 * Send a reset command to the active sensor.
 * @param reset_type
 */
void tof_dev_mgr_sensor_reset(uint8_t reset_type) {
    shandle.reset_cmd = reset_type;
}

/**
 * Signals when the process() call should run.
 */
void tof_dev_mgr_set_run_signal(void) {
    if (!should_run) {
        should_run = true;
    }
}

/**
 * Initialize the sensor by type
 * @param shandle
 * @param sensor
 * @return
 */
static tof_dev_mgr_err_t tof_dev_mgr_sensor_init(const tof_sensor_handle_t *shandle, tof_sensor_t *sensor) {
    if (sensor->id >= NUM_TOF_SENSOR_TYPES) {
        return TOF_DEV_MGR_ERR_SENSOR_IDX_OOR;
    }
    // Create a name for the sensor using the type and id
    snprintf (sensor->name, sizeof(sensor->name), "%s_%d", tof_sensor_get_name_str (sensor->type), sensor->id);

    switch (sensor->type) {
        // VL53L4CD and VL53L4CX both use the VL53LX driver
        case TOF_SENSOR_TYPE_VL53L4CD:
        case TOF_SENSOR_TYPE_VL53L4CX: {
            tof_sensor_err_t err = vl53lx_init (shandle, sensor);
            if (TOF_SENSOR_ERR_NONE != err) {
                return TOF_DEV_MGR_ERR_SENSOR_INIT;
            }
            break;
        }
        default:
            return TOF_DEV_MGR_ERR_SENSOR_INIT;
    }

    return TOF_SENSOR_ERR_NONE;
}

/**
 * Process configuration commands.
 */
static void tof_dev_mgr_process_cfg_cmd(void) {
    switch (cfg_cmd.d.trgt) {
        case DEV_CFG_TRGT_SNSR:
            return;
        case DEV_CFG_TRGT_USER:
            if (!cfg_cmd.pending) {
                return;
            }
            cfg_cmd_process_msg ("user_data", &cfg_cmd.d.cfg, &tof_dev_set_user_cfg);
            tof_dev_cfg_cmd_callback (&cfg_cmd.d);
            cfg_cmd.pending = false;
            return;
        default:
            if (!cfg_cmd.pending) {
                return;
            }
            cfg_cmd.d.cfg.status = CONFIG_STAT_NA;
            cfg_cmd_process_msg ("invalid target", &cfg_cmd.d.cfg, NULL);
            tof_dev_cfg_cmd_callback (&cfg_cmd.d);
            cfg_cmd.pending = false;
            return;
    }
}

/**
 * Handles configuration commands for getting/storing external data.
 * User can stored any uint32_t data up to an index id < MAX_STORAGE_DATA_BUFF_SIZE
 */
static void tof_dev_set_user_cfg(cfg_cmd_data_t *config_cmd) {
    uint8_t id = config_cmd->id;
    uint8_t cmd = config_cmd->cmd;
    int32_t value = config_cmd->value;

    config_cmd->status = CONFIG_STAT_OK;

    // If configuration id not in range and command
    // not CONFIG_CMD_STORE then exit function
    if (id >= MAX_USER_CONFIG_BUFF_SIZE && cmd != CONFIG_CMD_STORE) {
        config_cmd->status = CONFIG_STAT_NA;
        return;
    }
    // Process the command
    switch (cmd) {
        case CONFIG_CMD_GET:
            config_cmd->value = user_fds_data[id];
            return;
        case CONFIG_CMD_SET:
            user_fds_data[id] = value;
            return;
        case CONFIG_CMD_RESET:
            // Maybe cache a default value in the future
            return;
        case CONFIG_CMD_STORE:
            fds_mgr_write (FILE_ID_USER_STORAGE, RKEY_USER_STORAGE, (uint8_t*) &user_fds_data[0], sizeof(user_fds_data));
            return;
        default:
            config_cmd->status = CONFIG_STAT_INVALID;
            return;
    }
}

/**
 * Get string for error message.
 * @param error
 * @return
 */
static const char* tof_dev_mgr_get_error_str(tof_dev_mgr_err_t error) {
    switch (error) {
        case TOF_DEV_MGR_ERR_NONE:
            return "ok";
        case TOF_DEV_MGR_ERR_COMMS_INIT:
            return "comms";
        case TOF_DEV_MGR_ERR_FDS_INIT:
            return "fds";
        case TOF_DEV_MGR_ERR_SENSOR_INIT:
            return "init";
        case TOF_DEV_MGR_ERR_SENSOR_IDX_OOR:
            return "idx oor";
        case TOF_DEV_MGR_ERR_INTERNAL:
            return "internal";
        case TOF_DEV_MGR_ERR_NA:
        default:
            return "unknown";
    }
}

/**
 * Log an error message.
 * @param error
 */
static void tof_dev_mgr_on_error_handler(tof_dev_mgr_err_t error) {
    NRF_LOG_INFO("ToF err: snsr_mgr - %s", tof_dev_mgr_get_error_str (error));
}
