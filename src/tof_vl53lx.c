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

#include "tof_vl53lx.h"
#include "vl53lx_api.h"
#include "fds_mgr.h"
#include "config_cmd.h"
#include "timer_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpio.h"
#include "app_timer.h"

/**
 * Configurations:
 *
 * CONFIG_PHASECAL_PATCH_PWR - Setting to 2 increases the time of the first sample which is used as a reference
 * CONFIG_RECT_OF_INTEREST   - If using value 101255430: TopLeft_XY(6,9) BottomRight_XY(9,6) is a
 *                             4x4 SPAD array in a 15x15 box where BottomLeft is (0,0) and TopRight is (15,15)
 *                             >> Reference VL53LX_UserRoi_t in vl53lx_def.h
 */
typedef enum {
    // User Configurations
    CONFIG_POWER_LEVEL = 0,
    CONFIG_PHASECAL_PATCH_PWR,
    CONFIG_TIME_BUDGET,
    CONFIG_OFFSET_MODE,
    CONFIG_DISTANCE_MODE,
    CONFIG_SMUDGE_CORR_MODE,
    CONFIG_XTALK_COMP_EN,
    CONFIG_RECT_OF_INTEREST,
    // Calibrations
    CONFIG_CAL_REFSPAD,
    CONFIG_CAL_OFFSET_SIMPLE,
    CONFIG_CAL_OFFSET_ZERO,
    CONFIG_CAL_OFFSET_VCSEL,
    CONFIG_CAL_XTALK,
    NUM_CONFIGS,
} config_t;

// All sensor data
typedef struct {
    VL53LX_Dev_t sensor;
    VL53LX_CalibrationData_t cal;
}vl53lx_data_t;

// State functions
static void state_boot(void);
static void state_prepare(void);
static void state_init(void);
static void state_idle(void);
static void state_start(void);
static void state_stop(void);
static void state_clear_int_and_start(void);
static void state_standby(void);
static void state_int_status(void);
static void state_get_result(void);
static void state_err(void);
static void state_err_timeout(void);
static void state_config(void);

// Helper functions
static void config_cmd_handler(cfg_cmd_data_t* cfg);
static uint8_t set_config(tof_sensor_t* sensor, uint8_t id, int32_t value);
static uint8_t load_config(tof_sensor_t* sensor, uint8_t id);
static void init_config_types(tof_sensor_t *sensor);
static const char* get_config_str(uint8_t config);
static void notify_config_update(uint8_t config_id);

static int32_t cfg_buff[MAX_SNSR_CONFIG_BUFF_SIZE];

APP_TIMER_DEF(err_timeout_timer_id);

// Helper macros to access sensor data
#define VL53LX_DATA(snsr)   ((vl53lx_data_t*)snsr->context)
#define VL53LX(snsr)        ((VL53LX_DEV)&(VL53LX_DATA(snsr)->sensor))

#if (NUM_CONFIGS > MAX_SNSR_CONFIG_BUFF_SIZE)
#error Increase MAX_SNSR_CONFIG_BUFF_SIZE to handle NUM_CONFIGS for this device
#endif

static tof_sensor_handle_t* shandle = NULL;

tof_sensor_err_t vl53lx_init(const tof_sensor_handle_t* handle, tof_sensor_t* sensor){
    // Set the sensor handle
    if (!shandle) {
        shandle = (tof_sensor_handle_t*) handle;
    }
    // Create the sensor data
    if (!sensor->context) {
        // Pointer to new instance of VL53LX_DEV
        // Note: Required if using multiple instances of same sensor type
        sensor->context = (void*) malloc (sizeof(vl53lx_data_t));
        // Check if memory allocated
        if (!sensor->context) {
            return TOF_SENSOR_ERR_SENSOR_CREATE;
        }
    }

    sensor->num_configs = NUM_CONFIGS;
    sensor->status = TOF_SENSOR_STATUS_BOOTING;
    sensor->state = state_boot;

    /* Configure XSHUT pin as output */
    nrf_gpio_cfg_output (sensor->pin_xshut);
    /* Set XSHUT pin low to trigger a "fresh out of reset" condition */
    nrf_gpio_pin_clear (sensor->pin_xshut);
    /* Init the configuration types */
    init_config_types (sensor);

    NRF_LOG_INFO("%s created", sensor->name);

    return TOF_SENSOR_ERR_NONE;
}

static void err_timeout_timer_handler(void *p_context) {
    tof_sensor_handle_t *shandle = (tof_sensor_handle_t*) p_context;
    shandle->sensor->state = state_boot;
}

static void load_default_configs(void) {
    // Load and copy default configurations
    for (uint8_t i = 0; i < shandle->sensor->num_configs; ++i) {
        load_config(shandle->sensor, i);
        // Store copy of the factory default values
        shandle->sensor->config[i].value_default = shandle->sensor->config[i].value;
    }
}

static void state_boot(void) {
    // Restart the device by toggling the xshut pin
    nrf_gpio_pin_clear(shandle->sensor->pin_xshut);
    PollingDelayMS(5);
    nrf_gpio_pin_set(shandle->sensor->pin_xshut);
    PollingDelayMS(5);
    // Notify sensor selected
    tof_sensor_data_callback(TOF_DATA_SELECTED, shandle->sensor);
    // Notify sensor status
    shandle->sensor->status = TOF_SENSOR_STATUS_BOOTING;
    tof_sensor_data_callback(TOF_DATA_STATUS, &shandle->sensor->status);
    // Set the factory default address
    VL53LX(shandle->sensor)->i2c_slave_address = I2C_ADDR_DEFAULT;
    // Wait for boot-up
    if (!VL53LX_WaitDeviceBooted(VL53LX(shandle->sensor))) {
        NRF_LOG_INFO("%s booted", shandle->sensor->name);
    }
    else {
        NRF_LOG_INFO("%s err: boot", shandle->sensor->name);
        shandle->sensor->state = state_err;
        return;
    }
    /* Set new address */
    if (!VL53LX_SetDeviceAddress(VL53LX(shandle->sensor), ((uint8_t)shandle->sensor->address << 1))) {
        VL53LX(shandle->sensor)->i2c_slave_address = shandle->sensor->address;
        NRF_LOG_INFO("%s address set to 0x%X", shandle->sensor->name, shandle->sensor->address);
    }
    else {
        NRF_LOG_INFO("%s err: setting address", shandle->sensor->name);
        shandle->sensor->state = state_err;
        return;
    }
    // Get device information (not really necessary)
    VL53LX_DeviceInfo_t dInfo;
    if (!VL53LX_GetDeviceInfo(VL53LX(shandle->sensor), &dInfo)) {
        NRF_LOG_INFO("%s get info", shandle->sensor->name);
    }
    else {
        NRF_LOG_INFO("%s err: get info", shandle->sensor->name);
        shandle->sensor->state = state_err;
        return;
    }

    shandle->sensor->state = state_prepare;
}

static void state_prepare(void) {
    // Data init
    if (!VL53LX_DataInit (VL53LX(shandle->sensor))) {
        NRF_LOG_INFO("%s data init", shandle->sensor->name);
    }
    else {
        NRF_LOG_INFO("%s err: data init", shandle->sensor->name);
        shandle->sensor->state = state_err;
        return;
    }
    // Load and copy factory default configurations
    load_default_configs();

    uint16_t status = 0;
    // Prepare the sensor based on reset command, the default attempts to load user configurations
    switch (shandle->reset_cmd) {
        default:
        case TOF_SENSOR_RESET_SENSOR:
            // Read calibration data from storage if it exists
            status = fds_mgr_read(
                    FILE_ID_SNSR_DATA(shandle->sensor->id),
                    RKEY_SNSR_DATA_CAL,
                    (uint8_t*) &VL53LX_DATA(shandle->sensor)->cal,
                    sizeof(VL53LX_DATA(shandle->sensor)->cal));

            if(status) {
                break;
            }

            // Read user configurations from storage if it exists
            status = fds_mgr_read (
                    FILE_ID_SNSR_DATA(shandle->sensor->id),
                    RKEY_SNSR_DATA_USER,
                    (uint8_t*) &cfg_buff,
                    SNSR_CFG_STORAGE_SIZE(shandle));

            if(status) {
                break;
            }

            // Set calibration data
            status = VL53LX_SetCalibrationData(VL53LX(shandle->sensor), &VL53LX_DATA(shandle->sensor)->cal);

            if(!status) {
                NRF_LOG_INFO("%s set cal data", shandle->sensor->name);
            }
            else {
                NRF_LOG_INFO("%s err: set cal data", shandle->sensor->name);
                break;
            }

            // Set Distance mode first (according to data sheet)
            if (CONFIG_STAT_ERROR != set_config(shandle->sensor, CONFIG_DISTANCE_MODE, cfg_buff[CONFIG_DISTANCE_MODE])) {
                load_config (shandle->sensor, CONFIG_DISTANCE_MODE);
                NRF_LOG_INFO("%s set %s", shandle->sensor->name, get_config_str(CONFIG_DISTANCE_MODE));
            }
            else {
                NRF_LOG_INFO("%s set %s: error",shandle->sensor->name, get_config_str(CONFIG_DISTANCE_MODE));
                shandle->sensor->state = state_err;
                return;
            }
            // Set rest of the configurations
            for (int i = 0; i < shandle->sensor->num_configs; ++i) {
                // Skip Distance mode since it's handled outside the loop
                if (CONFIG_DISTANCE_MODE == i) {
                    continue;
                }
                // Skip non parameter configuration types
                if (TOF_SENSOR_CONFIG_TYPE_PARAM != shandle->sensor->config[i].type) {
                    continue;
                }
                // Set the configuration. If no error then cache the value
                if (CONFIG_STAT_ERROR != set_config(shandle->sensor, i, cfg_buff[i])) {
                    load_config (shandle->sensor, i);
                    NRF_LOG_INFO("%s set %s", shandle->sensor->name, get_config_str(i));
                }
                else {
                    NRF_LOG_INFO("%s set %s: error", shandle->sensor->name, get_config_str(i));
                    shandle->sensor->state = state_err;
                    return;
                }
            }
            break;

        case TOF_SENSOR_RESET_SENSOR_FACTORY:
            fds_mgr_delete(FILE_ID_SNSR_DATA(shandle->sensor->id), RKEY_SNSR_DATA_CAL);
            fds_mgr_delete(FILE_ID_SNSR_DATA(shandle->sensor->id), RKEY_SNSR_DATA_USER);
            break;
    }

    // Run a basic calibration if no calibration/user data stored
    if(status){
        // Perform ref spad
        if (!set_config(shandle->sensor, CONFIG_CAL_REFSPAD, 0)) {
            NRF_LOG_INFO("%s set %s", shandle->sensor->name, get_config_str(CONFIG_CAL_REFSPAD));
        }
        else {
            NRF_LOG_INFO("%s set %s: error", shandle->sensor->name, get_config_str(CONFIG_CAL_REFSPAD));
            shandle->sensor->state = state_err;
            return;
        }
        // Perform xtalk
        if (!set_config(shandle->sensor, CONFIG_CAL_XTALK, 0)) {
            NRF_LOG_INFO("%s set %s", shandle->sensor->name, get_config_str(CONFIG_CAL_XTALK));
        }
        else {
            NRF_LOG_INFO("%s set %s: error", shandle->sensor->name, get_config_str(CONFIG_CAL_XTALK));
            shandle->sensor->state = state_err;
            return;
        }
    }

    // Go to state_init
    shandle->sensor->state = state_init;
}

static void state_init(void) {
    shandle->sensor->sample_count = 0;
    shandle->sensor->distance_mm = 0;
    shandle->sensor->status = TOF_SENSOR_STATUS_READY;
    NRF_LOG_INFO("%s ready", shandle->sensor->name);
    // Notify user
    tof_sensor_data_callback(TOF_DATA_SELECTED, shandle->sensor);
    tof_sensor_data_callback(TOF_DATA_STATUS, &shandle->sensor->status);
    tof_sensor_data_callback(TOF_DATA_DISTANCE, &shandle->sensor->distance_mm);
    tof_sensor_data_callback(TOF_DATA_SAMPLING_ENABLED, &shandle->ranging_enabled);
    // Clear any reset commands if previously set
    if (shandle->reset_cmd < NUM_TOF_RESET_OPTIONS) {
        shandle->reset_cmd = TOF_RESET_NA;
        tof_sensor_data_callback(TOF_DATA_RESET, &shandle->reset_cmd);
    }
    // Go to state_idle
    shandle->sensor->state = state_idle;
}

static void state_idle(void) {
    if (shandle->sensor->id != shandle->id_selected) {
        shandle->sensor->state = state_standby;
    }
    else if (shandle->config_pending) {
        shandle->sensor->state = state_config;
    }
    else if (shandle->reset_cmd < NUM_TOF_RESET_OPTIONS) {
        shandle->sensor->state = state_boot;
    }
    else if (shandle->ranging_enabled) {
        shandle->sensor->state = state_start;
    }
    else {
        // Do nothing..
    }
}

static void state_start(void) {
    if (!VL53LX_StartMeasurement(VL53LX(shandle->sensor))) {
        shandle->sensor->state = state_int_status;
    }
    else {
        NRF_LOG_INFO("%s err: start ranging", shandle->sensor->name);
        shandle->sensor->state = state_err;
    }
}

static void state_stop(void) {
    (void) app_timer_stop(err_timeout_timer_id);

    if (!VL53LX_StopMeasurement(VL53LX(shandle->sensor))) {
        shandle->sensor->sample_count = 0;
        shandle->sensor->distance_mm = 0;
        tof_sensor_data_callback(TOF_DATA_DISTANCE, &shandle->sensor->distance_mm);
        NRF_LOG_INFO("%s stopped", shandle->sensor->name);
        shandle->sensor->state = state_idle;
    }
    else {
        NRF_LOG_INFO("%s err: stopping", shandle->sensor->name);
        shandle->sensor->state = state_err;
    }
}

void state_clear_int_and_start(void) {
    if (shandle->sensor->id != shandle->id_selected) {
        shandle->sensor->state = state_stop;
    }
    else if (shandle->config_pending) {
        shandle->sensor->state = state_config;
    }
    else if (shandle->reset_cmd < NUM_TOF_RESET_OPTIONS) {
        shandle->sensor->state = state_boot;
    }
    else if (!shandle->ranging_enabled) {
        shandle->sensor->state = state_stop;
    }
    else if (!VL53LX_ClearInterruptAndStartMeasurement(VL53LX(shandle->sensor))) {
        shandle->sensor->state = state_int_status;
    }
    else {
        NRF_LOG_INFO("%s err: clear int and start", shandle->sensor->name);
        shandle->sensor->state = state_err;
    }
}

static void state_standby(void) {
    if (shandle->sensor->id != shandle->id_selected) {
        shandle->sensor->status = TOF_SENSOR_STATUS_STANDBY;
    }
    else {
        shandle->sensor->state = state_init;
    }
}

static void state_int_status(void) {
    uint8_t pMeasurementDataReady;
    if (!VL53LX_GetMeasurementDataReady(VL53LX(shandle->sensor), &pMeasurementDataReady)) {
        if (pMeasurementDataReady) {
            shandle->sensor->state = state_get_result;
        }
    }
    else {
        NRF_LOG_INFO("%s err: get status", shandle->sensor->name);
        shandle->sensor->state = state_err;
    }
}

static void state_get_result(void) {
    VL53LX_MultiRangingData_t rangeData;
    if (!VL53LX_GetMultiRangingData(VL53LX(shandle->sensor), &rangeData)) {
        shandle->sensor->sample_count++;
        int16_t lrange = 0;
        for (int j = 0; j < rangeData.NumberOfObjectsFound; ++j) {
            // Keep the largest range value
            if (lrange < rangeData.RangeData[j].RangeMilliMeter) {
                lrange = rangeData.RangeData[j].RangeMilliMeter;
            }
            if (shandle->debug_enabled) {
                NRF_LOG_INFO("%s obj: %d, status=%d, D=%5dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
                        shandle->sensor->name,
                        j,
                        rangeData.RangeData[j].RangeStatus,
                        rangeData.RangeData[j].RangeMilliMeter,
                        rangeData.RangeData[j].SignalRateRtnMegaCps / 65536.0,
                        rangeData.RangeData[j].AmbientRateRtnMegaCps / 65536.0);
            }
        }
        shandle->sensor->distance_mm = lrange;
        tof_sensor_data_callback(TOF_DATA_DISTANCE, &shandle->sensor->distance_mm);
        shandle->sensor->state = state_clear_int_and_start;
    }
    else {
        NRF_LOG_INFO("%s err: get result", shandle->sensor->name);
        shandle->sensor->state = state_err;
    }
}

static void state_err(void) {
    uint32_t err_code = app_timer_create(&err_timeout_timer_id, APP_TIMER_MODE_SINGLE_SHOT, err_timeout_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(err_timeout_timer_id, ERR_TIMEOUT_MS, shandle->sensor);
    APP_ERROR_CHECK(err_code);

    shandle->sensor->status = TOF_SENSOR_STATUS_ERROR;
    tof_sensor_data_callback(TOF_DATA_STATUS, &shandle->sensor->status);

    shandle->sensor->state = state_err_timeout;
}

static void state_err_timeout(void) {
    // Sit here until err timeout forces a reboot
}

static void state_config(void) {
    if (shandle->config_pending) {
        // Process the configuration
        cfg_cmd_process_msg (shandle->sensor->name, &shandle->config_cmd, &config_cmd_handler);
        // Notify user
        tof_sensor_data_callback (TOF_DATA_CONFIG, &shandle->config_cmd);
        // Reset the pending flag
        shandle->config_pending = false;
    }
    else {
        // Reset state to idle
        shandle->sensor->state = state_idle;
    }
}

static void config_cmd_handler(cfg_cmd_data_t* cfg) {
    uint8_t id = shandle->config_cmd.id;
    uint8_t cmd = shandle->config_cmd.cmd;
    int32_t value = shandle->config_cmd.value;

    shandle->config_cmd.status = CONFIG_STAT_OK;

    // If configuration id not in range and command
    // not CONFIG_CMD_STORE then exit function
    if (id >= shandle->sensor->num_configs && cmd != CONFIG_CMD_STORE) {
        shandle->config_cmd.status = CONFIG_STAT_NA;
        return;
    }

    // Process the command
    switch (cmd) {
        case CONFIG_CMD_GET:
            // Override the value to send back
            shandle->config_cmd.value = shandle->sensor->config[id].value;
            return;
        case CONFIG_CMD_SET:
            // Return if value is the same
            if (value == shandle->sensor->config[id].value) {
                return;
            }
            break;
        case CONFIG_CMD_RESET:
            // Return if value is the same
            if (shandle->sensor->config[id].value == shandle->sensor->config[id].value_default) {
                shandle->config_cmd.value = shandle->sensor->config[id].value_default;
                return;
            }
            else {
                // Set the value to factory default
                value = shandle->sensor->config[id].value_default;
            }
            break;
        case CONFIG_CMD_STORE: // Note: Currently stores all configurations, not individually
            // Get cal data
            VL53LX_GetCalibrationData(VL53LX(shandle->sensor), &VL53LX_DATA(shandle->sensor)->cal);
            // Store the cal data
            fds_mgr_write(
                FILE_ID_SNSR_DATA(shandle->sensor->id),
                RKEY_SNSR_DATA_CAL,
                (uint8_t*)&VL53LX_DATA(shandle->sensor)->cal,
                sizeof(VL53LX_DATA(shandle->sensor)->cal));

            // Buffer user configurations
            for(int i = 0; i < shandle->sensor->num_configs; ++i){
                cfg_buff[i] = shandle->sensor->config[i].value;
            }
            // Store user configurations
            fds_mgr_write(
                FILE_ID_SNSR_DATA(shandle->sensor->id),
                RKEY_SNSR_DATA_USER,
                (uint8_t*)&cfg_buff,
                SNSR_CFG_STORAGE_SIZE(shandle));
            return;
        default:
            shandle->config_cmd.status = CONFIG_STAT_INVALID;
            return;
    }

    // Try to set the requested value
    uint8_t status = set_config(shandle->sensor, id, value);

    // Load configuration
    load_config(shandle->sensor, id);

    // Override the value to send back
    shandle->config_cmd.value = shandle->sensor->config[id].value;

    // Check status
    if (status) {
        shandle->config_cmd.status = CONFIG_STAT_ERROR;
        return;
    }
    else if (CONFIG_CMD_SET == cmd
            && value != shandle->sensor->config[id].value
            && shandle->sensor->config[id].value != INVALID_CONFIG_VALUE) {
        shandle->config_cmd.status = CONFIG_STAT_MISMATCH;
        return;
    }
    else {
        return;
    }
}

static uint8_t set_config(tof_sensor_t *sensor, uint8_t id, int32_t value) {
    uint8_t status = CONFIG_STAT_OK;

    switch (id) {
        /** Configurations */
        case CONFIG_POWER_LEVEL:
            status = VL53LX_SetVCSELPowerAdjust(VL53LX(sensor), value);
            break;
        case CONFIG_PHASECAL_PATCH_PWR:
            status = VL53LX_SetTuningParameter(VL53LX(sensor), VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER, value);
            break;
        case CONFIG_TIME_BUDGET:
            status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(VL53LX(sensor), (uint32_t) value);
            break;
        case CONFIG_OFFSET_MODE:
            status = VL53LX_SetOffsetCorrectionMode(VL53LX(sensor), (uint8_t) value);
            break;
        case CONFIG_DISTANCE_MODE:
            status = VL53LX_SetDistanceMode(VL53LX(sensor), (uint8_t) value);
            break;
        case CONFIG_SMUDGE_CORR_MODE:
            status = VL53LX_SetSmudgeCorrectionMode(VL53LX(sensor), (uint8_t) value);
            break;
        case CONFIG_XTALK_COMP_EN:
            status = VL53LX_SetXTalkCompensationEnable(VL53LX(sensor), (uint8_t) value);
            break;
        case CONFIG_RECT_OF_INTEREST:{
            status = VL53LX_SetUserROI(VL53LX(sensor), (VL53LX_UserRoi_t*)&value);
            break;
        }
        /** Calibrations */
        case CONFIG_CAL_REFSPAD:
            status = VL53LX_PerformRefSpadManagement(VL53LX(sensor));
            if(!status){
                notify_config_update(CONFIG_TIME_BUDGET);
            }
            break;
        case CONFIG_CAL_OFFSET_SIMPLE:
            status = VL53LX_PerformOffsetSimpleCalibration(VL53LX(sensor), value);
            if(!status){
                notify_config_update(CONFIG_TIME_BUDGET);
            }
            break;
        case CONFIG_CAL_OFFSET_ZERO:
            status = VL53LX_PerformOffsetZeroDistanceCalibration(VL53LX(sensor));
            if(!status){
                notify_config_update(CONFIG_TIME_BUDGET);
            }
            break;
        case CONFIG_CAL_OFFSET_VCSEL:
            status = VL53LX_PerformOffsetPerVcselCalibration(VL53LX(sensor), value);
            if(!status){
                notify_config_update(CONFIG_TIME_BUDGET);
            }
            break;
        case CONFIG_CAL_XTALK:
            status = VL53LX_PerformXTalkCalibration(VL53LX(sensor));
            if(!status){
                notify_config_update(CONFIG_TIME_BUDGET);
            }
            break;
        default:
            status = CONFIG_STAT_NA;
    }

    return status;
}

static uint8_t load_config(tof_sensor_t *sensor, uint8_t id) {
    uint8_t status = CONFIG_STAT_OK;

    switch (id) {
        /** Configurations */
        case CONFIG_POWER_LEVEL: {
            uint8_t power_level;
            status = VL53LX_GetVCSELPowerAdjust(VL53LX(sensor), &power_level);
            if (!status) {
                sensor->config[id].value = (int32_t)power_level;
            }
            break;
        }
        case CONFIG_PHASECAL_PATCH_PWR:{
            int32_t pTuningParameterValue;
            status = VL53LX_GetTuningParameter(VL53LX(sensor), VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER, &pTuningParameterValue);
            if (!status) {
                sensor->config[id].value = pTuningParameterValue;
            }
            break;
        }
        case CONFIG_TIME_BUDGET: {
            uint32_t time_budget;
            status = VL53LX_GetMeasurementTimingBudgetMicroSeconds(VL53LX(sensor), &time_budget);
            if (!status) {
                sensor->config[id].value = (int32_t)time_budget;
            }
            break;
        }
        case CONFIG_OFFSET_MODE:
            sensor->config[id].value = (int32_t)VL53LX(sensor)->Data.LLData.offset_correction_mode;
            break;
        case CONFIG_DISTANCE_MODE:
            sensor->config[id].value = (int32_t)VL53LX(sensor)->Data.CurrentParameters.DistanceMode;
            break;
        case CONFIG_SMUDGE_CORR_MODE:{
            VL53LX_SmudgeCorrectionModes mode;
            status = VL53LX_GetSmudgeCorrectionMode(VL53LX(sensor), &mode);
            if (!status) {
                sensor->config[id].value = (int32_t)mode;
            }
            break;
        }
        case CONFIG_XTALK_COMP_EN:{
            uint8_t xtalk_comp_en;
            status = VL53LX_GetXTalkCompensationEnable(VL53LX(sensor), &xtalk_comp_en);
            if (!status) {
                sensor->config[id].value = (int32_t)xtalk_comp_en;
            }
            break;
        }
        case CONFIG_RECT_OF_INTEREST:{
            VL53LX_UserRoi_t roi;
            status = VL53LX_GetUserROI(VL53LX(sensor), &roi);
            if (!status) {
                memcpy(&sensor->config[id].value, &roi, sizeof(VL53LX_UserRoi_t));
            }
            break;
        }
        /** Calibrations */
        case CONFIG_CAL_REFSPAD:
        case CONFIG_CAL_OFFSET_SIMPLE:
        case CONFIG_CAL_OFFSET_ZERO:
        case CONFIG_CAL_OFFSET_VCSEL:
        case CONFIG_CAL_XTALK:
            sensor->config[id].value = INVALID_CONFIG_VALUE;
            break;
        default:
            break;
    }

    return status;
}

static void notify_config_update(uint8_t config_id){
    // Only notify of configuration updates when sensor is ready
    if (TOF_SENSOR_STATUS_READY != shandle->sensor->status){
        return;
    }
    // Check if configuration id is valid
    if (config_id >= shandle->sensor->num_configs) {
        return;
    }

    cfg_cmd_data_t cfg_cmd = {
		.cmd = CONFIG_CMD_GET,
		.id = config_id
    };

    // Reload the configuration and set status
    if (!load_config(shandle->sensor, config_id)) {
    	cfg_cmd.value = shandle->sensor->config[config_id].value;
    	cfg_cmd.status = CONFIG_STAT_UPDATED;
    }
    else{
    	cfg_cmd.value = INVALID_CONFIG_VALUE;
    	cfg_cmd.status = CONFIG_STAT_ERROR;
    }
    // Notify
    tof_sensor_data_callback(TOF_DATA_CONFIG, &cfg_cmd);
}

static void init_config_types(tof_sensor_t *sensor){
    for(int i = 0; i < sensor->num_configs; ++i){
        switch (i) {
            /** Parameters */
            case CONFIG_POWER_LEVEL:
            case CONFIG_PHASECAL_PATCH_PWR:
            case CONFIG_TIME_BUDGET:
            case CONFIG_OFFSET_MODE:
            case CONFIG_DISTANCE_MODE:
            case CONFIG_SMUDGE_CORR_MODE:
            case CONFIG_XTALK_COMP_EN:
            case CONFIG_RECT_OF_INTEREST:
                sensor->config[i].type = TOF_SENSOR_CONFIG_TYPE_PARAM;
                break;
            /** Calibrations */
            case CONFIG_CAL_REFSPAD:
            case CONFIG_CAL_OFFSET_SIMPLE:
            case CONFIG_CAL_OFFSET_ZERO:
            case CONFIG_CAL_OFFSET_VCSEL:
            case CONFIG_CAL_XTALK:
                sensor->config[i].type = TOF_SENSOR_CONFIG_TYPE_CAL;
                break;
            default:
                sensor->config[i].type = TOF_SENSOR_CONFIG_TYPE_NA;
                break;
        }
    }
}

static const char* get_config_str(uint8_t config) {
    switch (config) {
        case CONFIG_POWER_LEVEL:
            return "pwr lvl";
        case CONFIG_PHASECAL_PATCH_PWR:
            return "patch pwr";
        case CONFIG_TIME_BUDGET:
            return "time budget";
        case CONFIG_OFFSET_MODE:
            return "offset mode";
        case CONFIG_DISTANCE_MODE:
            return "dist mode";
        case CONFIG_SMUDGE_CORR_MODE:
            return "smudge corr en";
        case CONFIG_XTALK_COMP_EN:
            return "xtalk comp en";
        case CONFIG_RECT_OF_INTEREST:
            return "roi";
        case CONFIG_CAL_REFSPAD:
            return "ref spad";
        case CONFIG_CAL_OFFSET_SIMPLE:
            return "offset simple";
        case CONFIG_CAL_OFFSET_ZERO:
            return "offset zero";
        case CONFIG_CAL_OFFSET_VCSEL:
            return "offst vcsel";
        case CONFIG_CAL_XTALK:
            return "xtalk";
        default:
            return "unknown";
    }
}
