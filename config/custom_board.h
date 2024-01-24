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
#define PIN_SDA                         (13)
#define PIN_SCL                         (11)
#define PIN_TOF_SHORT_XSHUT             (29)
#define PIN_TOF_LONG_XSHUT              (17)
#define PIN_CHARGE_STATUS               (22)
#define PIN_BM                          (3)
#define PIN_BM_EN                       (19)
#elif defined(BT_VERSION_6_0)
#define PIN_PWR_ON_LED                  (27)
#define PIN_PWR_ON_STATUS               (15)
#define PIN_PS_HOLD                     (8)
#define PIN_PB_OUT                      (4)
#define PIN_INT                         (7)
#define PIN_SDA                         (11)
#define PIN_SCL                         (9)
#define PIN_TOF_SHORT_XSHUT             (13)
#define PIN_TOF_LONG_XSHUT              (14)
#define PIN_CHARGE_STATUS               (22)
#define PIN_BM                          (12) // TODO - FIX, this is not ANx pin
#define PIN_BM_EN                       (28)
#elif defined(BT_VERSION_7_0)
#define PIN_PWR_ON_LED                  (26)
#define PIN_PWR_ON_STATUS               (15)
#define PIN_PS_HOLD                     (8)
#define PIN_PB_OUT                      (4)
#define PIN_INT                         (7)
#define PIN_SDA                         (11)
#define PIN_SCL                         (9)
#define PIN_TOF_SHORT_XSHUT             (6)
#define PIN_TOF_LONG_XSHUT              (14)
#define PIN_CHARGE_STATUS               (22)
#define PIN_BM                          (28)
#define PIN_BM_EN                       (27)
#else
#error No pin mapping defined for this board!
#endif

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
