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

#include <fds_mgr.h>
#include "tof_VL53LX_states.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "timer_delay.h"
#include "vl53lx_api.h"

/**
 * Configurations:
 *
 * CONFIG_PHASECAL_PATCH_PWR - Setting to 2 increases the time of the first sample which is used as a reference
 * CONFIG_RECT_OF_INTEREST   - If using value 101255430: TopLeft_XY(6,9) BottomRight_XY(9,6) is a
 *                             4x4 SPAD array in a 15x15 box where BottomLeft is (0,0) and TopRight is (15,15)
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

#if (NUM_CONFIGS > MAX_CONFIG_BUFF_SIZE)
#error Increase MAX_CONFIG_BUFF_SIZE to handle NUM_CONFIGS for this sensor type
#endif

// All sensor data
typedef struct {
    VL53LX_Dev_t sensor;
    VL53LX_CalibrationData_t cal;
}sensor_data_t;

// Helper macros to access sensor data
#define VL53LX(snsr)        ((VL53LX_DEV)&(((sensor_data_t*)snsr->context)->sensor))
#define VL53LX_DATA(snsr)   ((sensor_data_t*)snsr->context)

APP_TIMER_DEF(err_timeout_timer_id);
static device_t* device = NULL;
static int32_t cfg_buff[MAX_CONFIG_BUFF_SIZE];

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
static void config_cmd_handler(void);
static uint8_t set_config(snsr_data_t* sensor, uint8_t id, int32_t value);
static uint8_t load_config(snsr_data_t* sensor, uint8_t id);
static void init_config_types(snsr_data_t *sensor);
static const char* get_config_str(uint8_t config);
static void notify_config_update(uint8_t config_id);


error_t vl53lx_create(device_t* dev, snsr_data_t* sensor, uint8_t type, uint8_t id, uint8_t address, uint8_t xshut_pin) {
    if(NULL == device){ // Since this can be called multiple times, only set it once
        device = dev;
    }
    // Check id
    if(id >= NUM_TOF_SNSR){
    	return TOF_ERROR_SENSOR_INDEX_OOR;
    }
	// Set the type
    switch(type){
        // VL53L4CD and VL53L4CX both use the VL53LX driver
		case SNSR_TYPE_VL53L4CD:
		case SNSR_TYPE_VL53L4CX:
			sensor->type = type;
			break;

		default:
			return TOF_ERROR_SENSOR_TYPE;
    }

    snprintf(sensor->name, sizeof(sensor->name), "%s_%d", get_sensor_name_str(type), id);

    sensor->id = id;
    sensor->address = address;
    sensor->pin_xshut = xshut_pin;
    sensor->num_configs = NUM_CONFIGS;
    sensor->state = state_boot;
    sensor->status = TOF_STATUS_BOOTING;

    // Pointer to new instance of VL53LX_DEV
    // Note: Required if using multiple instances of same sensor type
    sensor->context = (void*)malloc(sizeof(sensor_data_t));

    if(NULL == sensor->context){
    	return TOF_ERROR_SENSOR_CREATE;
    }

    /* Configure XSHUT pin as output */
    nrf_gpio_cfg_output(sensor->pin_xshut);
    /* Set XSHUT pin low to trigger a "fresh out of reset" condition */
    nrf_gpio_pin_clear(sensor->pin_xshut);

    /* Init the configuration types */
    init_config_types(sensor);
    /* Init the flash-data-storage if not already */
    if(tof_fds_init()){
    	return TOF_ERROR_SENSOR_INIT;
    }

    NRF_LOG_INFO("%s created", sensor->name);

    return TOF_ERROR_NONE;
}

static void err_timeout_timer_handler(void *p_context) {
    device_t *device = (device_t*) p_context;
    device->sensor->state = state_boot;
    device->sensor->status = TOF_STATUS_ERROR;
    tof_data_callback(device, TOF_DATA_STATUS);
}

static void load_default_configs(void) {
    // Load and copy default configurations
    for (uint8_t i = 0; i < device->sensor->num_configs; ++i) {
        load_config(device->sensor, i);
        // Store copy of the factory default values
        device->sensor->config[i].value_default = device->sensor->config[i].value;
    }
}

static void state_boot(void) {
    // Restart the device by toggling the xshut pin
    nrf_gpio_pin_clear(device->sensor->pin_xshut);
    PollingDelayMS(5);
    nrf_gpio_pin_set(device->sensor->pin_xshut);
    PollingDelayMS(5);

    // Notify sensor selected
    tof_data_callback(device, TOF_DATA_SELECTED);
    // Notify sensor status
    device->sensor->status = TOF_STATUS_BOOTING;
    tof_data_callback(device, TOF_DATA_STATUS);

    // Set the factory default address
    VL53LX(device->sensor)->i2c_slave_address = I2C_ADDR_DEFAULT;

    // Wait for boot-up
    if (!VL53LX_WaitDeviceBooted(VL53LX(device->sensor))) {
        NRF_LOG_INFO("%s booted", device->sensor->name);
    }
    else {
        NRF_LOG_INFO("%s err: boot", device->sensor->name);
        device->sensor->state = state_err;
        return;
    }

    /* Set new address */
    if (!VL53LX_SetDeviceAddress(VL53LX(device->sensor), ((uint8_t)device->sensor->address << 1))) {
        VL53LX(device->sensor)->i2c_slave_address = device->sensor->address;
        NRF_LOG_INFO("%s address set to 0x%X", device->sensor->name, device->sensor->address);
    }
    else {
        NRF_LOG_INFO("%s err: setting address", device->sensor->name);
        device->sensor->state = state_err;
        return;
    }

    // Get device information (only use if comparing sensor meta data)
    VL53LX_DeviceInfo_t dInfo;
    if (!VL53LX_GetDeviceInfo(VL53LX(device->sensor), &dInfo)) {
        NRF_LOG_INFO("%s get info", device->sensor->name);
    }
    else {
        NRF_LOG_INFO("%s err: get info", device->sensor->name);
        device->sensor->state = state_err;
        return;
    }

    device->sensor->state = state_prepare;
}

static void state_prepare(void) {
    uint8_t status = 0;

    // Data init
    if (!VL53LX_DataInit(VL53LX(device->sensor))) {
        NRF_LOG_INFO("%s data init", device->sensor->name);
    }
    else {
        NRF_LOG_INFO("%s err: data init", device->sensor->name);
        device->sensor->state = state_err;
        return;
    }

    // Load and copy default configurations
    load_default_configs();

    // Is a factory reset requested?
    if (TOF_RESET_SENSOR_FACTORY == device->reset_cmd) {
        // Yes - Delete all sensor data
        tof_fds_delete(FILD_ID_SNSR_DATA(device->sensor->id), RKEY_SNSR_DATA_CAL);
        tof_fds_delete(FILD_ID_SNSR_DATA(device->sensor->id), RKEY_SNSR_DATA_USER);
    }

    // Read cal data from storage if it exists
    status = tof_fds_read(
    		FILD_ID_SNSR_DATA(device->sensor->id),
			RKEY_SNSR_DATA_CAL,
            (uint8_t*)&VL53LX_DATA(device->sensor)->cal,
            sizeof(VL53LX_DATA(device->sensor)->cal));

    // Cal data read?
    if(!status){
        // Yes - Set calibration data
        if (!VL53LX_SetCalibrationData(VL53LX(device->sensor), &VL53LX_DATA(device->sensor)->cal)) {
            NRF_LOG_INFO("%s set cal data", device->sensor->name);
        }
        else {
            NRF_LOG_INFO("%s err: set cal data", device->sensor->name);
        }

        // Read user data from storage if it exists
        status = tof_fds_read(
        		FILD_ID_SNSR_DATA(device->sensor->id),
				RKEY_SNSR_DATA_USER,
                (uint8_t*)&cfg_buff,
                SNSR_CFGS_SIZE(device));

        // User data read?
        if(!status){
            // Yes - Set Distance mode first (according to data sheet)
            if(CONFIG_STAT_ERROR != set_config(device->sensor, CONFIG_DISTANCE_MODE, cfg_buff[CONFIG_DISTANCE_MODE])){
                load_config(device->sensor, CONFIG_DISTANCE_MODE);
                NRF_LOG_INFO("%s set %s", device->sensor->name, get_config_str(CONFIG_DISTANCE_MODE));
            }
            else{
                NRF_LOG_INFO("%s set %s: error", device->sensor->name, get_config_str(CONFIG_DISTANCE_MODE));
                device->sensor->state = state_err;
            }

            // Yes - Set all applicable configurations
            for(int i = 0; i < device->sensor->num_configs; ++i){
                // Skip Distance mode since it's handled outside the loop
                if(CONFIG_DISTANCE_MODE == i){
                    continue;
                }

                // Skip non parameter configuration types
                if(CONFIG_TYPE_PARAM != device->sensor->config[i].type){
                    continue;
                }

                // Set the configuration. If no error then cache the value
                if(CONFIG_STAT_ERROR != set_config(device->sensor, i, cfg_buff[i])){
                    load_config(device->sensor, i);
                    NRF_LOG_INFO("%s set %s", device->sensor->name, get_config_str(i));
                }
                else{
                    NRF_LOG_INFO("%s set %s: error", device->sensor->name, get_config_str(i));
                    device->sensor->state = state_err;
                }
            }
        }
    }
    // No - Then run required start up calibrations
    else{
        // Perform ref spad
        if (!set_config(device->sensor, CONFIG_CAL_REFSPAD, 0)) {
            NRF_LOG_INFO("%s set %s", device->sensor->name, get_config_str(CONFIG_CAL_REFSPAD));
        }
        else {
            NRF_LOG_INFO("%s set %s: error", device->sensor->name, get_config_str(CONFIG_CAL_REFSPAD));
            device->sensor->state = state_err;
            return;
        }

        // Perform xtalk
        if (!set_config(device->sensor, CONFIG_CAL_XTALK, 0)) {
            NRF_LOG_INFO("%s set %s", device->sensor->name, get_config_str(CONFIG_CAL_XTALK));
        }
        else {
            NRF_LOG_INFO("%s set %s: error", device->sensor->name, get_config_str(CONFIG_CAL_XTALK));
            device->sensor->state = state_err;
            return;
        }
    }

    // Go to state_init
    device->sensor->state = state_init;
}

static void state_init(void) {
    device->sample_count = 0;
    device->distance_mm = 0;

    device->sensor->status = TOF_STATUS_READY;
    NRF_LOG_INFO("%s ready", device->sensor->name);

    tof_data_callback(device, TOF_DATA_SELECTED);
    tof_data_callback(device, TOF_DATA_STATUS);
    tof_data_callback(device, TOF_DATA_DISTANCE);
    tof_data_callback(device, TOF_DATA_SAMPLING_ENABLED);

    if (TOF_RESET_SENSOR == device->reset_cmd ||
        TOF_RESET_SENSOR_FACTORY == device->reset_cmd) {
        device->reset_cmd = TOF_RESET_NA;
        tof_data_callback(device, TOF_DATA_RESET);
    }

    // Go to state_idle
    device->sensor->state = state_idle;
}

static void state_idle(void) {
    // If this sensor is not the selected then standby
    if (device->sensor->id != device->id_selected) {
        device->sensor->state = state_standby;
    }
    // Process configuration commands
    else if (device->config_pending && CONFIG_TRGT_SNSR == device->config_cmd.trgt) {
        device->sensor->state = state_config;
    }
    // Reset the sensor if the reset command was sent
    else if (TOF_RESET_SENSOR == device->reset_cmd ||
             TOF_RESET_SENSOR_FACTORY == device->reset_cmd) {
        device->sensor->state = state_boot;
    }
    // Run the sensor ranging if enabled
    else if (device->is_ranging_enabled) {
        device->sensor->state = state_start;
    }
    else {
        // Do nothing..
    }
}

static void state_start(void) {
    if (!VL53LX_StartMeasurement(VL53LX(device->sensor))) {
        device->sensor->state = state_int_status;
    }
    else {
        NRF_LOG_INFO("%s err: start ranging", device->sensor->name);
        device->sensor->state = state_err;
    }
}

static void state_stop(void) {
    // Make sure a to stop any err timers
    (void) app_timer_stop(err_timeout_timer_id);

    if (!VL53LX_StopMeasurement(VL53LX(device->sensor))) {
        device->sample_count = 0;
        device->distance_mm = 0;
        tof_data_callback(device, TOF_DATA_DISTANCE);
        NRF_LOG_INFO("%s stopped", device->sensor->name);
        device->sensor->state = state_idle;
    }
    else {
        NRF_LOG_INFO("%s err: stopping", device->sensor->name);
        device->sensor->state = state_err;
    }
}

void state_clear_int_and_start(void) {
    // If this sensor is not the selected then standby
    if (device->sensor->id != device->id_selected) {
        device->sensor->state = state_stop;
    }
    // Process configuration commands
    else if (device->config_pending && CONFIG_TRGT_SNSR == device->config_cmd.trgt) {
        device->sensor->state = state_config;
    }
    // Reset the sensor if the command was sent
    else if (TOF_RESET_SENSOR == device->reset_cmd ||
             TOF_RESET_SENSOR_FACTORY == device->reset_cmd) {
        device->sensor->state = state_boot;
    }
    // Stop the sensor ranging if disabled
    else if (!device->is_ranging_enabled) {
        device->sensor->state = state_stop;
    }
    else if (!VL53LX_ClearInterruptAndStartMeasurement(VL53LX(device->sensor))) {
        device->sensor->state = state_int_status;
    }
    else {
        NRF_LOG_INFO("%s err: clear int and start", device->sensor->name);
        device->sensor->state = state_err;
    }
}

static void state_standby(void) {
    // Wait here until the state machine switches to another sensor
    if (device->sensor->id != device->id_selected) {
        device->sensor->status = TOF_STATUS_STANDBY;
    }
    else {
        device->sensor->state = state_init;
    }
}

static void state_int_status(void) {
    uint8_t pMeasurementDataReady;
    if (!VL53LX_GetMeasurementDataReady(VL53LX(device->sensor), &pMeasurementDataReady)) {
        if (pMeasurementDataReady) {
            device->sensor->state = state_get_result;
        }
    }
    else {
        NRF_LOG_INFO("%s err: get status", device->sensor->name);
        device->sensor->state = state_err;
    }
}

static void state_get_result(void) {
    VL53LX_MultiRangingData_t rangeData;
    if (!VL53LX_GetMultiRangingData(VL53LX(device->sensor), &rangeData)) {
        device->sample_count++;
        int16_t lrange = 0;
        for (int j = 0; j < rangeData.NumberOfObjectsFound; ++j) {
            // Keep the largest range value
            if (lrange < rangeData.RangeData[j].RangeMilliMeter) {
                lrange = rangeData.RangeData[j].RangeMilliMeter;
            }
            if (device->is_debug_enabled) {
                NRF_LOG_INFO("%s obj: %d, status=%d, D=%5dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
                        device->sensor->name,
                        j,
                        rangeData.RangeData[j].RangeStatus,
                        rangeData.RangeData[j].RangeMilliMeter,
                        rangeData.RangeData[j].SignalRateRtnMegaCps / 65536.0,
                        rangeData.RangeData[j].AmbientRateRtnMegaCps / 65536.0);
            }
        }
        device->distance_mm = lrange;
        tof_data_callback(device, TOF_DATA_DISTANCE);
        device->sensor->state = state_clear_int_and_start;
    }
    else {
        NRF_LOG_INFO("%s err: get result", device->sensor->name);
        device->sensor->state = state_err;
    }
}

static void state_err(void) {
    uint32_t err_code = app_timer_create(&err_timeout_timer_id, APP_TIMER_MODE_SINGLE_SHOT, err_timeout_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(err_timeout_timer_id, ERR_TIMEOUT_MS, device);
    APP_ERROR_CHECK(err_code);

    device->sensor->state = state_err_timeout;
}

static void state_err_timeout(void) {
    // Sit here until err timeout forces a reboot
}

static void state_config(void) {
    // Process the configuration command
    tof_handle_config_cmd(&config_cmd_handler);
    // Reset state to idle
    device->sensor->state = state_idle;
}

static void config_cmd_handler(void) {
    uint8_t id = device->config_cmd.id;
    uint8_t cmd = device->config_cmd.cmd;
    int32_t value = device->config_cmd.value;

    device->config_cmd.status = CONFIG_STAT_OK;

    // If configuration id not in list and command
    // not CONFIG_CMD_STORE then exit. The store command
    // currently accepts any id since it stores all configurations.
    if (id >= device->sensor->num_configs && cmd != CONFIG_CMD_STORE) {
        device->config_cmd.status = CONFIG_STAT_NA;
        return;
    }

    // Process the command
    switch (cmd) {
        case CONFIG_CMD_GET:
            // Override the value to send back
            device->config_cmd.value = device->sensor->config[id].value;
            return;
        case CONFIG_CMD_SET:
            // Return if value is the same
            if (value == device->sensor->config[id].value) {
                return;
            }
            break;
        case CONFIG_CMD_RESET:
            // Return if value is the same
            if (device->sensor->config[id].value == device->sensor->config[id].value_default) {
                device->config_cmd.value = device->sensor->config[id].value_default;
                return;
            }
            else {
                // Set the value to factory default
                value = device->sensor->config[id].value_default;
            }
            break;
        case CONFIG_CMD_STORE: // Note: Currently stores all configurations, not individually
            // Get cal data
            VL53LX_GetCalibrationData(VL53LX(device->sensor), &VL53LX_DATA(device->sensor)->cal);
            // Store the cal data
            tof_fds_write(
            		FILD_ID_SNSR_DATA(device->sensor->id),
					RKEY_SNSR_DATA_CAL,
                    (uint8_t*)&VL53LX_DATA(device->sensor)->cal,
                    sizeof(VL53LX_DATA(device->sensor)->cal));

            // Buffer user configurations
            for(int i = 0; i < device->sensor->num_configs; ++i){
                cfg_buff[i] = device->sensor->config[i].value;
            }
            // Store user configurations
            tof_fds_write(
            		FILD_ID_SNSR_DATA(device->sensor->id),
					RKEY_SNSR_DATA_USER,
                    (uint8_t*)&cfg_buff,
                    SNSR_CFGS_SIZE(device));
            return;
        default:
            device->config_cmd.status = CONFIG_STAT_INVALID;
            return;
    }

    // Try to set the requested value
    uint8_t status = set_config(device->sensor, id, value);

    // Load configuration
    load_config(device->sensor, id);

    // Override the value to send back
    device->config_cmd.value = device->sensor->config[id].value;

    // Check status
    if (status) {
        device->config_cmd.status = CONFIG_STAT_ERROR;
        return;
    }
    else if (CONFIG_CMD_SET == cmd
            && value != device->sensor->config[id].value
            && device->sensor->config[id].value != INVALID_CONFIG_VALUE) {
        device->config_cmd.status = CONFIG_STAT_MISMATCH;
        return;
    }
    else {
        return;
    }
}

static uint8_t set_config(snsr_data_t *sensor, uint8_t id, int32_t value) {
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

static uint8_t load_config(snsr_data_t *sensor, uint8_t id) {
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
    if (TOF_STATUS_READY != device->sensor->status){
        return;
    }

    if (config_id >= device->sensor->num_configs) {
        return;
    }

    // Set the configuration command data
    device->config_cmd_updated.trgt = CONFIG_TRGT_SNSR;
    device->config_cmd_updated.cmd = CONFIG_CMD_GET;
    device->config_cmd_updated.id = config_id;

    // Reload the configuration and set status
    if (!load_config(device->sensor, config_id)) {
    	device->config_cmd_updated.value = device->sensor->config[config_id].value;
    	device->config_cmd_updated.status = CONFIG_STAT_UPDATED;
    }
    else{
    	device->config_cmd_updated.value = INVALID_CONFIG_VALUE;
    	device->config_cmd_updated.status = CONFIG_STAT_ERROR;
    }

    // Notify
    tof_data_callback(device, TOF_DATA_CONFIG_UPDATED);
}

static void init_config_types(snsr_data_t *sensor){
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
                sensor->config[i].type = CONFIG_TYPE_PARAM;
                break;
            /** Calibrations */
            case CONFIG_CAL_REFSPAD:
            case CONFIG_CAL_OFFSET_SIMPLE:
            case CONFIG_CAL_OFFSET_ZERO:
            case CONFIG_CAL_OFFSET_VCSEL:
            case CONFIG_CAL_XTALK:
                sensor->config[i].type = CONFIG_TYPE_CAL;
                break;
            default:
                sensor->config[i].type = CONFIG_TYPE_NA;
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
