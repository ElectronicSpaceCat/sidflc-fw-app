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

#include "boards.h"
#include "tof_twi.h"

#include "nrfx_twim.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "timer_delay.h"

// NOTE: Do NOT wrap the value in parentheses or it will break build!
#define TWIM_INSTANCE_ID   0

static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(TWIM_INSTANCE_ID);
static volatile uint8_t m_twi_xfer_active = false;
static volatile nrfx_twim_evt_type_t m_twi_xfer_evt = NRFX_TWIM_EVT_DONE;

static ret_code_t map_evt_to_nrf_err(nrfx_twim_evt_type_t evt);

__STATIC_INLINE void print_err_msg(nrfx_twim_xfer_type_t xfrType, nrfx_twim_evt_type_t evt) {
    NRF_LOG_INFO("ToF TWI xfer type: %d, event: %d", xfrType, evt);
    m_twi_xfer_evt = evt;
}

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrfx_twim_evt_t const *p_event, void *p_context) {
    switch (p_event->type) {
        case NRFX_TWIM_EVT_DONE:
            if (p_event->xfer_desc.type == NRFX_TWIM_XFER_RX) {
                // ...
            }
            break;
        case NRFX_TWIM_EVT_DATA_NACK:
            break;
        case NRFX_TWIM_EVT_ADDRESS_NACK:
        case NRFX_TWIM_EVT_OVERRUN:
        	print_err_msg(p_event->xfer_desc.type, p_event->type);
        	break;
        case NRFX_TWIM_EVT_BUS_ERROR:
        	print_err_msg(p_event->xfer_desc.type, p_event->type);
        	NRF_LOG_INFO("ToF TWI resetting");
        	tof_twi_uninit();
        	nrfx_twim_bus_recover(PIN_SCL, PIN_SDA);
        	tof_twi_init();
            break;
        default:
            break;
    }
    // Set false regardless of event type to ensure this is non-blocking
    m_twi_xfer_active = false;
}

/// NOTE: AG - I2CWrite implementation for range sensors
int I2CWrite(uint8_t dev, uint8_t *buff, uint8_t len) {
    m_twi_xfer_active = true;

    ret_code_t err_code = nrfx_twim_tx(&m_twi, dev, buff, len, false);
    APP_ERROR_CHECK(err_code);

    wfe(&m_twi_xfer_active);

    if (NRFX_TWIM_EVT_DONE != m_twi_xfer_evt) {
        err_code = map_evt_to_nrf_err(m_twi_xfer_evt);
        m_twi_xfer_evt = NRFX_TWIM_EVT_DONE;
    }

    return err_code;
}

/// NOTE: AG - I2CRead implementation for range sensors
int I2CRead(uint8_t dev, uint8_t *buff, uint8_t len) {
    m_twi_xfer_active = true;

    ret_code_t err_code = nrfx_twim_rx(&m_twi, dev, buff, len);
    APP_ERROR_CHECK(err_code);

    wfe(&m_twi_xfer_active);

    if (NRFX_TWIM_EVT_DONE != m_twi_xfer_evt) {
        err_code = map_evt_to_nrf_err(m_twi_xfer_evt);
        m_twi_xfer_evt = NRFX_TWIM_EVT_DONE;
    }

    return err_code;
}

/**
 * @brief TWI initialization.
 */
ret_code_t tof_twi_init(void) {
    ret_code_t err_code;

    const nrfx_twim_config_t twi_config = {
        .scl = PIN_SCL,
        .sda = PIN_SDA,
        .frequency = I2C_FREQ,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .hold_bus_uninit = false
    };

    err_code = nrfx_twim_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    if (NRF_SUCCESS == err_code) {
        nrfx_twim_enable(&m_twi);
        NRF_LOG_INFO("ToF TWI enabled");
    }
    else {
        NRF_LOG_INFO("ToF TWI error");
    }

    return err_code;
}

ret_code_t tof_twi_uninit(void){
    nrfx_twim_uninit(&m_twi);
    return NRF_SUCCESS;
}

static ret_code_t map_evt_to_nrf_err(nrfx_twim_evt_type_t evt){
    switch(evt){
		case NRFX_TWIM_EVT_DONE:
			return NRFX_SUCCESS;
		case NRFX_TWIM_EVT_ADDRESS_NACK:
			return NRFX_ERROR_DRV_TWI_ERR_ANACK;
		case NRFX_TWIM_EVT_DATA_NACK:
			return NRFX_ERROR_DRV_TWI_ERR_DNACK;
		case NRFX_TWIM_EVT_OVERRUN:
			return NRFX_ERROR_DRV_TWI_ERR_OVERRUN;
		case NRFX_TWIM_EVT_BUS_ERROR:
		default:
			return NRFX_ERROR_INTERNAL;
    }
}
