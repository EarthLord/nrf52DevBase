/* 	Copyright (c) 2015, Prithvi Raj Narendra
 *	All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without modification,
 *	are permitted provided that the following conditions are met:
 *
 *	1. Redistributions of source code must retain the above copyright notice,
 *	this list of conditions and the following disclaimer.
 *
 *	2. Redistributions in binary form must reproduce the above copyright notice,
 *	this list of conditions and the following disclaimer in the documentation
 *	and/or other materials provided with the distribution.
 *
 *	3. Neither the name of the copyright holder nor the names of its contributors
 *	may be used to endorse or promote products derived from this software without
 *	specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *	IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *	INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *	OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *	WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *	POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NRF52_GPIO_H_
#define NRF52_GPIO_H_

#include "nrf.h"

/**
 * @brief Pin configuration function for nrf52
 *
 * This function allows to set any aspect in PIN_CNF register.
 * Check nrf52_bitfields.h for the correct values
 * @param pin_num Specifies the pin number (allowed values 0-31).
 * @param dir   Pin direction (GPIO_PIN_CNF_DIR_Input or GPIO_PIN_CNF_DIR_Output)
 * @param input Connect or disconnect input buffer (GPIO_PIN_CNF_INPUT_Connect or GPIO_PIN_CNF_INPUT_Disconnect)
 * @param pull  Pull configuration (GPIO_PIN_CNF_PULL_Disabled, GPIO_PIN_CNF_PULL_Pulldown or GPIO_PIN_CNF_PULL_Pullup)
 * @param drive Drive configuration (Standard GPIO_PIN_CNF_DRIVE_S0S1)
 * @param sense Pin sensing mechanism (GPIO_PIN_CNF_SENSE_Disabled, GPIO_PIN_CNF_SENSE_High or GPIO_PIN_CNF_SENSE_Low)
 */
__STATIC_INLINE void nrf_gpio_cfg( 	uint32_t pin_num,
									uint32_t dir,
									uint32_t input,
									uint32_t pull,
									uint32_t drive,
									uint32_t sense);

/**
 * @brief Function for configuring the given GPIO pin number as output with default values.
 *        This function can be used to configure pin range as simple input with gate driving GPIO_PIN_CNF_DRIVE_S0S1 (normal cases).
 *
 * @param pin_num specifies the pin number (allowed values 0-31)
 *
 * @note  Sense capability on the pin is disabled, and input is disconnected from the buffer as the pins are configured as output.
 */
__STATIC_INLINE void nrf_gpio_cfg_output(uint32_t pin_num);

/**
 * @brief Function for configuring the given GPIO pin number as input with default values.
 *        This function can be used to configure pin range as simple input with gate driving GPIO_PIN_CNF_DRIVE_S0S1 (normal cases).
 *
 * @param pin_num Specifies the pin number (allowed values 0-30).
 * @param pull_config State of the pin range pull resistor (no pull, pulled down or pulled high).
 *
 * @note  Sense capability on the pin is disabled, and input is connected to buffer so that the GPIO->IN register is readable
 */
__STATIC_INLINE void nrf_gpio_cfg_input(uint32_t pin_num, uint32_t pull_config);

/**
 * @brief Function for setting a GPIO pin.
 *
 * @param pin_num specifies the pin number [0:31] to set.
 */
__STATIC_INLINE void nrf_gpio_pin_set(uint32_t pin_num);

/**
 * @brief Function for clearing a GPIO pin.
 *
 * @param pin_num specifies the pin number [0:31] to clear.
 */
__STATIC_INLINE void nrf_gpio_pin_clear(uint32_t pin_num);

/**
 * @brief Function for toggling a GPIO pin.
 *
 * @param pin_num specifies the pin number [0:31] to toggle.
 */
__STATIC_INLINE void nrf_gpio_pin_toggle(uint32_t pin_num);

__STATIC_INLINE void nrf_gpio_cfg(	uint32_t pin_num,
									uint32_t dir,
									uint32_t input,
									uint32_t pull,
									uint32_t drive,
									uint32_t sense){
	NRF_P0->PIN_CNF[pin_num] = (dir   << GPIO_PIN_CNF_DIR_Pos)
						  | (input << GPIO_PIN_CNF_INPUT_Pos)
						  | (pull  << GPIO_PIN_CNF_PULL_Pos)
						  | (drive << GPIO_PIN_CNF_DRIVE_Pos)
						  | (sense << GPIO_PIN_CNF_SENSE_Pos);
}

__STATIC_INLINE void nrf_gpio_cfg_output(uint32_t pin_num){
    nrf_gpio_cfg(pin_num,
				GPIO_PIN_CNF_DIR_Output,
				GPIO_PIN_CNF_INPUT_Disconnect,
				GPIO_PIN_CNF_PULL_Disabled,
				GPIO_PIN_CNF_DRIVE_S0S1,
				GPIO_PIN_CNF_SENSE_Disabled);
}

__STATIC_INLINE void nrf_gpio_cfg_input(uint32_t pin_num, uint32_t pull_config){
    nrf_gpio_cfg(pin_num,
				GPIO_PIN_CNF_DIR_Input,
				GPIO_PIN_CNF_INPUT_Connect,
				pull_config,
				GPIO_PIN_CNF_DRIVE_S0S1,
				GPIO_PIN_CNF_SENSE_Disabled);
}

__STATIC_INLINE void nrf_gpio_pin_set(uint32_t pin_num){
    NRF_P0->OUTSET = (1UL << pin_num);
}

__STATIC_INLINE void nrf_gpio_pin_clear(uint32_t pin_num){
    NRF_P0->OUTCLR = (1UL << pin_num);
}

__STATIC_INLINE void nrf_gpio_pin_toggle(uint32_t pin_num){
    const uint32_t pin_bit   = 1UL << pin_num;
    const uint32_t pin_state = ((NRF_P0->OUT >> pin_num) & 1UL);

    if (pin_state == 0)
    {
        // Low to high
        NRF_P0->OUTSET = pin_bit;
    }
    else
    {
        // high to low
        NRF_P0->OUTCLR = pin_bit;
    }
}

#endif /* NRF52_GPIO_H_ */
