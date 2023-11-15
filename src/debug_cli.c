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

#include "debug_cli.h"
#include "ble_tof_service.h"
#include "ble_pwr_service.h"
#include "tof_device_mgr.h"
#include "pwr_mgr.h"
#include "stdlib.h"
#include "boards.h"
#include "SEGGER_RTT.h"
#include "string.h"
#include "stdbool.h"
#include "peer_manager.h"

#define CHAR_BUFF_SIZE 20

// Command buffer
static char _buff[CHAR_BUFF_SIZE];

void debug_cli_init(void) {
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    SEGGER_RTT_WriteString(0, "SIDFLC (Smartphone Interfaced Device For Launching Coins)\n");
    SEGGER_RTT_WriteString(0, "Copyright (C) 2021-present Andrew Green\n");
    SEGGER_RTT_WriteString(0, "This program comes with ABSOLUTELY NO WARRANTY.\n");
    SEGGER_RTT_WriteString(0, "This is free software, and you are welcome to redistribute it\n");
    SEGGER_RTT_WriteString(0, "in accordance to the GPL-3.0 license.\n\n");
}

void debug_cli_process(void) {
    static uint8_t idx = 0;
    static int cmd_ready = 0;

    // Exit immediately if no data to process
    if (!SEGGER_RTT_HasKey()) {
        return;
    }

    if (cmd_ready) {
        cmd_ready = 0;
        idx = 0;
    }

    int r = 0;

    // Read characters as long as they are available
    while (SEGGER_RTT_HasKey()) {
        // Empty buffer if command is ready
        if(cmd_ready){
            SEGGER_RTT_GetKey();
            continue;
        }

        // Exit if buffer exceeded
        if (idx >= (CHAR_BUFF_SIZE - 1)) {
            SEGGER_RTT_WriteString(0, "exceeded max chars\n");
            cmd_ready = 1; // Set ready and return which will reset the command
            return;
        }

        // Read characters
        r = SEGGER_RTT_GetKey();
        // Newline or carriage return seen?
        if (r == '\r' || r == '\n') {
            // Yes - Terminate the string and mark command ready
            _buff[idx++] = '\0';
            cmd_ready = 1;
        }
        // No - Fill string buffer
        else {
            _buff[idx++] = (char) r;
        }
    }

    // Exit if command not ready for process
    if (!cmd_ready) {
        return;
    }

    // Parse the command
    char *token = strtok(_buff, ":");

    // Get first token and process it
    if (!strcmp(token, "help")) {
        SEGGER_RTT_WriteString(0, "\nCommands:\n");
        SEGGER_RTT_WriteString(0, "batt:print            batt - toggle batt sampling, batt:print - toggle debug\n");
        SEGGER_RTT_WriteString(0, "rng:print             rng - toggle ranging, rng:print - toggle debug\n");
        SEGGER_RTT_WriteString(0, "ble:tof|pwr:print     debug ble:tof|pwr service\n");
        SEGGER_RTT_WriteString(0, "snsr:id               snsr - get snsr id, snsr:id select snsr\n");
        SEGGER_RTT_WriteString(0, "cfg:trgt:cmd:id:value send config cmd, -cmds for cmd list, -trgts for trgt list\n");
        SEGGER_RTT_WriteString(0, "cfg:trgt:all          get all trgt configs\n");
        SEGGER_RTT_WriteString(0, "rst:s|sf|d            reset: s - sensor, sf - sensor factory, d -device\n");
        SEGGER_RTT_WriteString(0, "shutdown:en           shutdown - power off device, shutdown:en - toggle shutdown enabled\n\n");
        return;
    }
    else if (!strcmp(token, "-cmds")) {
        SEGGER_RTT_WriteString(0, "config Commands:\n");
        SEGGER_RTT_WriteString(0, "0  get\n");
        SEGGER_RTT_WriteString(0, "1  set\n");
        SEGGER_RTT_WriteString(0, "2  reset\n");
        SEGGER_RTT_WriteString(0, "3  store\n");
        return;
    }
    else if (!strcmp(token, "-trgts")) {
        SEGGER_RTT_WriteString(0, "config targets:\n");
        SEGGER_RTT_WriteString(0, "0  active sensor\n");
        SEGGER_RTT_WriteString(0, "1  user\n");
        return;
    }
    else if (!strcmp(token, "batt")) {
        token = strtok(NULL, ":");
        if (NULL == token) {
            tof_pwr_batt_sample_voltage();
        }
        else if (!strcmp(token, "print")) {
            tof_pwr_batt_print_enable();
        }
        else{
            // Do nothing..
        }
        return;
    }
    else if (!strcmp(_buff, "rng")) {
        token = strtok(NULL, ":");
        if (NULL == token) {
            const tof_sensor_handle_t *shandle = tof_dev_mgr_shandle_get();
            tof_dev_mgr_set_ranging_enable(shandle->ranging_enabled? 0 : 1);
        }
        else if (!strcmp(token, "print")) {
            const tof_sensor_handle_t *shandle = tof_dev_mgr_shandle_get();
            tof_dev_mgr_set_debug_enable(shandle->debug_enabled? 0 : 1);
        }
        else{
            // Do nothing..
        }
        return;
    }
    else if (!strcmp(token, "ble")) {
        token = strtok(NULL, ":");
        if (!strcmp(token, "tof")) {
            token = strtok(NULL, ":");
            if (!strcmp(token, "print")) {
                tof_gatts_hvx_debug_enable();
            }
        }
        else if (!strcmp(token, "pwr")) {
            token = strtok(NULL, ":");
            if (!strcmp(token, "print")) {
                pwr_gatts_hvx_debug_enable();
            }
        }
        else{
            // Do nothing..
        }
        return;
    }
    else if (!strcmp(_buff, "snsr")) {
        token = strtok(NULL, ":");
        if (NULL == token) {
            const tof_sensor_handle_t *shandle = tof_dev_mgr_shandle_get();
            SEGGER_RTT_printf(0, "sensor selected: %s, addr: 0x%X\n", shandle->sensor->name, shandle->sensor->address);
        }
        else {
            uint8_t id = (uint8_t) atol(token);
            tof_dev_mgr_sensor_select(id);
        }
        return;
    }
    else if (!strcmp(_buff, "cfg")) {
        token = strtok(NULL, ":");
		if (NULL == token) {
			SEGGER_RTT_WriteString(0, "cfg trgt missing\n");
			return;
		}

		uint8_t trgt = (uint8_t) atol(token);

        token = strtok(NULL, ":");
        if (!strcmp(token, "all")) {
            switch (trgt) {
                case DEV_CFG_TRGT_SNSR: {
                    const tof_sensor_handle_t *shandle = tof_dev_mgr_shandle_get();
                    for (uint8_t i = 0; i < shandle->sensor->num_configs; ++i) {
                        int32_t value = tof_dev_mgr_get_cached_config(trgt, i);
                        SEGGER_RTT_printf(0, "config: %d : %d\n", i, value);
                    }
                    return;
                }
                case DEV_CFG_TRGT_USER:
                    for (uint8_t i = 0; i < MAX_USER_CONFIG_BUFF_SIZE; ++i) {
                        int32_t value = tof_dev_mgr_get_cached_config(trgt, i);
                        SEGGER_RTT_printf(0, "config: %d : %d\n", i, value);
                    }
                	return;
                default:
                	SEGGER_RTT_WriteString(0, "cfg trgt na\n");
                    return;
            }
        }

		if (NULL == token) {
			SEGGER_RTT_WriteString(0, "cfg cmd missing\n");
			return;
		}
		uint8_t cmd = (uint8_t) atol(token);

		token = strtok(NULL, ":");
		if (NULL == token) {
			SEGGER_RTT_WriteString(0, "cfg id missing\n");
			return;
		}
		uint8_t id = (uint8_t) atol(token);

		if(cmd != CONFIG_CMD_SET){
			tof_dev_mgr_set_cfg_cmd(trgt, cmd, id, 0);
			return;
		}

		token = strtok(NULL, ":");
		if (NULL == token) {
			SEGGER_RTT_WriteString(0, "cfg value missing\n");
			return;
		}

		int32_t value = (int32_t) atol(token);
		tof_dev_mgr_set_cfg_cmd(trgt, cmd, id, value);

        return;
    }
    else if (!strcmp(_buff, "rst")) {
        token = strtok(NULL, ":");
        if (!strcmp(token, "s")) {
        	tof_dev_mgr_sensor_reset(TOF_SENSOR_RESET_SENSOR);
            return;
        }
        else if (!strcmp(token, "sf")) {
        	tof_dev_mgr_sensor_reset(TOF_SENSOR_RESET_SENSOR_FACTORY);
            return;
        }
        else if (!strcmp(token, "d")) {
            tof_pwr_reset();
            return;
        }
        else{
            // Do nothing..
        }
    }
    else if (!strcmp(_buff, "shutdown")) {
        token = strtok(NULL, ":");
        if (NULL == token) {
        	tof_pwr_shutdown();
        	return;
        }
        else if (!strcmp(token, "en")) {
            tof_pwr_shutdown_enable();
        }
        return;
    }
    else{
        // Nothing to process..
    }

    SEGGER_RTT_WriteString(0, "unknown command\n");
}
