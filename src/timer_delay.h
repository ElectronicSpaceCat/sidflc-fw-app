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

#ifndef TIMER_DELAY_H
#define TIMER_DELAY_H

#include <stdint.h>
#include <app_timer.h>


/**
 * If using a lfclk at 32.768kHz then we only get ~30.517578125us
 * per tick and the app_timer takes a minimum of 5 ticks,
 * so we only get a minimum of (5 * 30.517578125) = 152.587890625us
 *
 * The best we can do is approximate a minimum allowed
 * value when using the delay in microseconds.
 *
 * 152us minimum plus increments of 30us.
 *
 * @param time_us
 * @return status
 */
uint32_t PollingDelayUS(uint32_t time_us);

/**
 *
 * @param time_ms
 * @return status
 */
uint32_t PollingDelayMS(uint32_t time_ms);

void wfe(volatile uint8_t* is_active);

#endif /* TIMER_DELAY_H */
