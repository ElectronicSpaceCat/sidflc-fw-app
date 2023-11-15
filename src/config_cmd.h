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

#ifndef CONFIG_CMD_H
#define CONFIG_CMD_H

#include <stdint.h>

#define INVALID_CONFIG_ID     (0xFF)
#define INVALID_CONFIG_VALUE  (0x7FFFFFFF)

typedef enum {
	CONFIG_CMD_GET = 0,
	CONFIG_CMD_SET,
	CONFIG_CMD_RESET,
	CONFIG_CMD_STORE,
    NUM_CONFIG_CMDS,
	CONFIG_CMD_NA
} cfg_cmd_t;

typedef enum {
	CONFIG_STAT_OK = 0,
	CONFIG_STAT_UPDATED,
	CONFIG_STAT_MISMATCH,
	CONFIG_STAT_ERROR,
	CONFIG_STAT_INVALID,
    NUM_CONFIG_STATS,
	CONFIG_STAT_NA
} cfg_cmd_status_t;

typedef struct {
    uint8_t cmd;
    uint8_t id;
    int32_t value;
    uint8_t status;
}cfg_cmd_data_t;

typedef void (*cfg_cmd_handler_t)(cfg_cmd_data_t*);

/**
 * @brief Function for running a configuration command handler
 * @param handler
 */
void cfg_cmd_process_msg(const char* src, cfg_cmd_data_t* cfg, cfg_cmd_handler_t handler);

#endif /* CONFIG_CMD_H */
