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

#include "tof_state_machine.h"

#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "timer_delay.h"
#include "tof_fds.h"

#include "tof_VL53LX_states.h"

static device_t* device;

void tof_sm_init(void) {
    // Initialize device
    tof_device_init();

    device = tof_device_get();

    // Initialize device data
    device->id_selected = TOF_SNSR_SHORT_RANGE;
    device->is_debug_enabled = false;
    device->is_ranging_enabled = false;
    device->config_cmd.trgt = CONFIG_TRGT_NA;

    // Initialize the short range sensor
    vl53lx_init(SNSR_TYPE_VL53L4CD, TOF_SNSR_SHORT_RANGE, (I2C_ADDR_DEFAULT + 1), PIN_TOF_SHORT_XSHUT);

    // Initialize the long range sensor
    vl53lx_init(SNSR_TYPE_VL53L4CX, TOF_SNSR_LONG_RANGE, (I2C_ADDR_DEFAULT + 2), PIN_TOF_LONG_XSHUT);

    // Set the default sensor on startup
    device->sensor = &device->sensors[TOF_SNSR_SHORT_RANGE];
}

void tof_sm_uninit(void) {
    tof_device_uninit();
}

void tof_sm_run(void) {
    // Switch sensors only when the current sensor status is in Standby or Error
    // Note: Switching should be done here and not by the sensors in the event of sensor error
    if(device->sensor->id != device->id_selected &&
        (TOF_STATUS_STANDBY == device->sensor->status  || TOF_STATUS_ERROR == device->sensor->status)){
        device->sensor = &device->sensors[device->id_selected];
    }

    // Run current state
    device->sensor->state();
}
