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

#include "timer_delay.h"

#include "boards.h"
#include "app_timer.h"

#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#endif

// See timer_delay.h for info on this macro
#define APP_TIMER_TICKS_US(US)          \
            ((uint32_t)MAX(5, (US/30)))

APP_TIMER_DEF(m_polling_delay_timer_id);

static void polling_delay_timer_handler(void *p_context) {
    uint8_t *_delay = (uint8_t*) p_context;
    *_delay = false;
}

static uint32_t createTimer(uint32_t ticks){
    static uint8_t m_delay_active = true;
    m_delay_active = true;

    uint32_t err_code = app_timer_create(&m_polling_delay_timer_id, APP_TIMER_MODE_SINGLE_SHOT, polling_delay_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_polling_delay_timer_id, ticks, &m_delay_active);
    APP_ERROR_CHECK(err_code);

    wfe(&m_delay_active);

    return err_code;
}

uint32_t PollingDelayUS(uint32_t time_us) {
    return createTimer(APP_TIMER_TICKS_US(time_us));
}

uint32_t PollingDelayMS(uint32_t time_ms) {
    return createTimer(APP_TIMER_TICKS(time_ms));
}

void wfe(volatile uint8_t* is_active) {
#ifdef SOFTDEVICE_PRESENT
    uint32_t err_code = NRF_SUCCESS;
#endif // SOFTDEVICE_PRESENT
    if (is_active) {
        while (*is_active){
#ifdef SOFTDEVICE_PRESENT
            if (nrf_sdh_is_enabled()) {
                err_code = sd_app_evt_wait();
                APP_ERROR_CHECK(err_code);
            }
#else
            {
              __WFE();
            }
#endif // SOFTDEVICE_PRESENT
        }
    }
    else {
#ifdef SOFTDEVICE_PRESENT
        if (nrf_sdh_is_enabled()) {
            err_code = sd_app_evt_wait();
            APP_ERROR_CHECK(err_code);
        }
#else
        {
          __WFE();
        }
#endif // SOFTDEVICE_PRESENT
    }
}
