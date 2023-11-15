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

#ifndef SRC_TOF_PWR_MONITOR_H_
#define SRC_TOF_PWR_MONITOR_H_

#include <stdint.h>

typedef struct{
  uint8_t pwr_source;
  uint8_t batt_status;
  uint8_t charge_state;
  uint32_t batt_lvl_milli_volts;
  uint8_t batt_lvl_percent;
  uint8_t low_voltage_sd_state;
}pwr_mngt_data_t;

/** The data types that the pwr monitor can send */
typedef enum{
  TOF_PWR_DATA_INPUT_SOURCE = 0,
  TOF_PWR_DATA_BATT_STATUS,
  TOF_PWR_DATA_BATT_LEVEL,
  NUM_TOF_PWR_DATA_TYPE
}tof_pwr_data_type_t;

void tof_pwr_init(void);
void tof_pwr_uninit(void);
void tof_pwr_batt_sample_voltage(void);
void tof_pwr_batt_sample_voltage_delayed(uint32_t delay_time_ms);
void tof_pwr_batt_print_enable(void);
const pwr_mngt_data_t* tof_pwr_get_mngt_data(void);
void tof_pwr_reset(void);
void tof_pwr_shutdown(void);
void tof_pwr_shutdown_enable(void);
void tof_pwr_data_callback(pwr_mngt_data_t* m_pwr_mngt_data, tof_pwr_data_type_t type);

#endif /* SRC_TOF_PWR_MONITOR_H_ */
