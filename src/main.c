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

/**
 *
 * @file
 * Main file where the code execution for the observer starts
 * @author
 * Prithvi
 */

#include <stdbool.h>
#include <stdint.h>
#include "tfp_printf.h"
#include "nrf_delay.h"
#include "nrf52-gpio.h"
#include "nrf52-clock.h"
#include "board.h"
#include "nrf52-uart.h"
#include "nrf52-rtc1.h"
#include "nrf52-timer0.h"
#include "nrf52-timer1.h"
#include "nrf52-rng.h"
#include "link-layer.h"
#include "ble.h"

static void log_dump_handler(void){
	dump_log();
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
	uint8_t adrs[] = {0x0B, 0x0E, 0x0A, 0x0C, 0x00, 0x01};
	uint8_t adv_data[] = {
			/* Len, type, data */
			0x02, GAP_ADV_FLAGS, 0x04,
			0x08, GAP_ADV_NAME_FULL, 'B', 'l', 'e', 's', 's', 'e', 'd'};

	uint8_t scan_rsp[] = {
			0x02, GAP_ADV_TRANSMIT_PWR, 0
	};

	adv_param_t param = {800, ADV_SCAN_IND_PARAM, RANDOM_ADRS_PARAM, CH_ALL_PARAM};

	hfclk_xtal_init();
	lfclk_init();

	nrf_gpio_cfg_output(LED1);
	nrf_gpio_cfg_output(LED2);
	nrf_gpio_cfg_output(LED3);
	nrf_gpio_cfg_output(LED4);

	uart_init();
	ms_timer_init();
	us_timer_init();
	profile_timer_init();

	tfp_printf("hello\n");

	ll_set_random_adrs(adrs);
	ll_set_adv_tx_power(0);
	ll_set_adv_data(sizeof(adv_data), adv_data);
	ll_set_adv_param(&param);
	ll_set_scan_rsp_data(sizeof(scan_rsp), scan_rsp);

	ll_start_adv();

	start_ms_timer(MS_TIMER0, MS_REPEATED_CALL, RTC_TICKS_MS(1027), log_dump_handler);

    while (true){
    	__WFI();
    }
}
