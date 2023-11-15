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

#include "tof_sensor.h"

/**
 * Get the senosr name
 * @return string
 */
const char* tof_sensor_get_name_str(uint8_t snsr_type) {
    switch(snsr_type){
        case TOF_SENSOR_TYPE_VL53L4CD:
            return "VL53L4CD";
        case TOF_SENSOR_TYPE_VL53L4CX:
            return "VL53L4CX";
        default:
            return "unknown type";
    }
}

/**
 * Get the command status
 * @return string
 */
const char* tof_sensor_get_status_str(uint8_t status){
    switch (status) {
        case TOF_SENSOR_STATUS_BOOTING:
            return "booting";
        case TOF_SENSOR_STATUS_READY:
            return "ready";
        case TOF_SENSOR_STATUS_STANDBY:
            return "standby";
        case TOF_SENSOR_STATUS_ERROR:
            return "error";
        case TOF_SENSOR_STATUS_NA:
            return "not available";
        default:
            return "unknown";
    }
};

/**
 * Get the error status
 * @return string
 */
const char* tof_sensor_get_error_str(tof_sensor_err_t error) {
    switch (error) {
        case TOF_SENSOR_ERR_NONE:
            return "sensor ok";
        case TOF_SENSOR_ERR_INVALID_TYPE:
            return "sensor invalid type";
        case TOF_SENSOR_ERR_SENSOR_CREATE:
            return "sensor create";
        case TOF_SENSOR_ERR_SENSOR_INIT:
            return "sensor init";
        case TOF_SENSOR_ERR_SENSOR_INTERNAL:
            return "sensor internal";
        case TOF_SENSOR_ERR_COMMS:
            return "sensor comms";
        case TOF_SENSOR_ERR_NA:
        default:
            return "unknown";
    }
}
