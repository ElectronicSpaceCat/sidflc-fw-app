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

#ifndef TOF_FDS_H
#define TOF_FDS_H

#include "boards.h"
#include <stdint.h>

/**
 * Note: - Record keys should be in the range 0x0001 - 0xBFFF
 *       - File IDs    should be in the range 0x0000 - 0xBFFF
 */

void tof_fds_init(void);
ret_code_t tof_fds_write(uint16_t file_id, uint16_t record_key, uint8_t* data, size_t data_len);
ret_code_t tof_fds_read(uint16_t file_id, uint16_t record_key, uint8_t* data, size_t data_len);
ret_code_t tof_fds_delete(uint16_t file_id, uint16_t record_key);

#endif /* TOF_FDS_H */
