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

/**
 * Function for reducing raw version data from memory to a readable semantic format.
 * Output follows the format major.minor.revision  i.e. "1.0.0"
 */
void tof_utils_reduce_version_str(const char* str_buff, char* buff) {
    size_t slen = strlen(str_buff);
    uint8_t counter = slen % 2;
    uint8_t idx = 0;

    for(int i = 0; i <= slen; ++i){
        // Null terminate string buff when last char reached
        if(i >= slen){
            buff[idx] = '\0';
            break;
        }
        // Ignore leading zeros when single digit value
        if(!counter && str_buff[i] == '0'){
            counter++;
            continue;
        }
        // Copy valid data to buffer
        buff[idx++] = str_buff[i];
        // Place the decimal after each version number
        if(counter++ % 2 && idx < slen){
            counter = 0;
            buff[idx++] = '.';
        }
    }
}
