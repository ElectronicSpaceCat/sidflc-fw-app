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

#ifndef TOF_VL53LX_STATES_H
#define TOF_VL53LX_STATES_H

#include "tof_device.h"

error_t vl53lx_create(device_t* dev, snsr_data_t* sensor, uint8_t type, uint8_t id, uint8_t address, uint8_t xshut_pin);

#endif /* TOF_VL53LX_STATES_H */
