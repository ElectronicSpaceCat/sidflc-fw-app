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

#include "config_cmd.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

static const char* cfg_cmd_get_cmd_str(uint8_t cmd);
static const char* cfg_cmd_get_cmd_status_str(uint8_t status);
static void cfg_cmd_log_cmd_msg(const char* src, const cfg_cmd_data_t* cfg);
static void cfg_cmd_log_cmd_msg_resp(const char* src, const cfg_cmd_data_t* cfg);

void cfg_cmd_process_msg(const char* src, cfg_cmd_data_t* cfg, cfg_cmd_handler_t handler) {
	cfg_cmd_log_cmd_msg(src, cfg);

	if(handler){
		handler(cfg);
	}

	cfg_cmd_log_cmd_msg_resp(src, cfg);
}

void cfg_cmd_log_cmd_msg(const char* src, const cfg_cmd_data_t* cfg) {
	NRF_LOG_INFO("%s << cmd: %s, cfg: %d, value: %d",
		src,
		cfg_cmd_get_cmd_str(cfg->cmd),
		cfg->id,
		cfg->value
	);
}

void cfg_cmd_log_cmd_msg_resp(const char* src, const cfg_cmd_data_t* cfg) {
	NRF_LOG_INFO("%s >> cmd: %s, cfg: %d, value: %d, status: %s",
		src,
		cfg_cmd_get_cmd_str(cfg->cmd),
		cfg->id,
		cfg->value,
		cfg_cmd_get_cmd_status_str(cfg->status)
	);
}

/**
 * Get configuration command string
 */
static const char* cfg_cmd_get_cmd_str(uint8_t cmd){
    switch (cmd) {
        case CONFIG_CMD_GET:
            return "get";
        case CONFIG_CMD_SET:
            return "set";
        case CONFIG_CMD_RESET:
            return "reset";
        case CONFIG_CMD_STORE:
            return "store";
        default:
            return "unknown";
    }
}

/**
 * Get configuration status string
 */
static const char* cfg_cmd_get_cmd_status_str(uint8_t status){
    switch (status) {
        case CONFIG_STAT_OK:
            return "ok";
        case CONFIG_STAT_UPDATED:
            return "updated";
        case CONFIG_STAT_MISMATCH:
            return "mismatch";
        case CONFIG_STAT_ERROR:
            return "error";
        case CONFIG_STAT_INVALID:
            return "invalid";
        case CONFIG_STAT_NA:
            return "not available";
        default:
            return "unknown";
    }
};

