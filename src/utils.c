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

#include <string.h>
#include "utils.h"
#include "nrf_dfu_types.h"

#define BOOTLOADER_SETTINGS_LOC  0x0007F000   /**< Location in flash where the bootloader settings starts..should match the bootloader's linker map */

static void get_verison_str(void);

// Char buffer for the firmware version string
static char fw_version_str[15];
static uint8_t is_initialized = 0;

/**
 * Get the formated version string
 * Note: Should call
 * @return
 */
const char* tof_utils_get_version_str_ptr(void) {
    if(!is_initialized) {
        is_initialized = 1;
        get_verison_str();
    }

    return &fw_version_str[0];
}


static void get_verison_str(void) {
    nrf_dfu_settings_t dfu_settings;
    static char buff[15];

    // Get version data from bootloader settings location
    memcpy(&dfu_settings, (uint32_t*)BOOTLOADER_SETTINGS_LOC, sizeof(nrf_dfu_settings_t));
    uint32_t app_version = dfu_settings.app_version;

    sprintf(buff, "%ld", app_version);

    size_t slen = strlen(buff);
    uint8_t counter = slen % 2;
    uint8_t idx = 0;


    // Reduce the raw version data from memory to a formated
    // version string in the form major.minor.revision  i.e. "1.0.0"
    for(int i = 0; i <= slen; ++i){
        // Null terminate string buff when last char reached
        if(i >= slen){
            fw_version_str[idx] = '\0';
            break;
        }
        // Ignore leading zeros when single digit value
        if(!counter && buff[i] == '0'){
            counter++;
            continue;
        }
        // Copy valid data to buffer
        fw_version_str[idx++] = buff[i];
        // Place the decimal after each version number
        if(counter++ % 2 && idx < slen){
            counter = 0;
            fw_version_str[idx++] = '.';
        }
    }
}
