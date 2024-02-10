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

#ifndef TOF_UTILS_H
#define TOF_UTILS_H

#include <stdint.h>

typedef struct {
    char* settings_ptr;   /**< Version of the current DFU settings struct layout. */
    char* app_ptr;        /**< Version of the last stored application. */
    char* bootloader_ptr; /**< Version of the last stored bootloader. */
} version_str_t;

const version_str_t* tof_utils_get_versions(void);

#endif /* TOF_UTILS_H */
