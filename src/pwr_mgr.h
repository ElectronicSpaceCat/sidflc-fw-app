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

#ifndef SRC_PWR_MGR_MONITOR_H_
#define SRC_PWR_MGR_MONITOR_H_

#include <stdint.h>

typedef struct{
  uint8_t pwr_source;
  uint8_t batt_status;
  uint8_t charge_state;
  uint32_t batt_lvl_milli_volts;
  uint8_t batt_lvl_percent;
  uint8_t low_voltage_sd_state;
}pwr_mgr_data_t;

/** The data types that the pwr monitor can send */
typedef enum{
  PWR_MGR_DATA_INPUT_SOURCE = 0,
  PWR_MGR_DATA_BATT_STATUS,
  PWR_MGR_DATA_BATT_LEVEL,
  NUM_PWR_MGR_DATA_TYPE
}pwr_mgr_data_type_t;

void pwr_mgr_init(void);
void pwr_mgr_uninit(void);
void pwr_mgr_process(void);
void pwr_mgr_batt_sample_voltage(void);
void pwr_mgr_batt_debug_enable(void);
void pwr_mgr_reset(void);
void pwr_mgr_shutdown(void);
void pwr_mgr_shutdown_enable(void);
void pwr_mgr_data_callback(pwr_mgr_data_t* m_pwr_mgr_data, pwr_mgr_data_type_t type);
const pwr_mgr_data_t* pwr_mgr_get_data(void);

#endif /* SRC_PWR_MGR_MONITOR_H_ */
