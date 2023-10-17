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

#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// NOTE: AG - Set the BT PCB version accordingly for proper pin mapping
#if defined(BT_VERSION_5_0)
#define PIN_PWR_ON_LED                  (12)
#define PIN_PWR_ON_STATUS               (18)
#define PIN_PS_HOLD                     (7)
#define PIN_PB_OUT                      (2)
#define PIN_INT                         (9)
#define PIN_SDA                         (13) // xx - Dev board
#define PIN_SCL                         (11) // xx - Dev board
#define PIN_TOF_SHORT_XSHUT             (29) // xx - Dev board
#define PIN_TOF_LONG_XSHUT              (17) // xx - Dev board
#define PIN_CHARGE_STATUS               (22)
#define PIN_BM                          (3)
#define PIN_BM_EN                       (19)
#else
#error No pin mapping defined for this board!
#endif

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
