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

static void timer_handler(void){
	nrf_gpio_pin_toggle(LED1);
PROFILE_START;
	tfp_printf("a111 b222 c333 d444\n");
	tfp_printf("abcde\n");
	tfp_printf("wxyz\n");
	tfp_printf("another one to test this uart implementation\n");
	tfp_printf("this is not the last one\n");
	tfp_printf("more are needed to see where the bug is\n");
//	nrf_delay_ms(120);
//	tfp_printf("0000000000111111111122222222223333333333444444444455555555556666666666777777777788888888889999999999\n");
//	tfp_printf("AAAAAAAAAABBBBBBBBBBCCCCCCCCCCDDDDDDDDDDEEEEEEEEEEFFFFFFFFFFGGGGGGGGGGHHHHHHHHHHIIIIIIIIIIJJJJJJJJJJ\n");
PROFILE_STOP;
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
	hfclk_xtal_init();
	lfclk_init();

	nrf_gpio_cfg_output(LED1);
	nrf_gpio_cfg_output(LED2);
	nrf_gpio_cfg_output(LED3);
	nrf_gpio_cfg_output(LED4);

	uart_init();
	ms_timer_init();
	timer_init();
	start_ms_timer(MS_TIMER0, REPEATED_CALL, RTC_TICKS(1000), timer_handler);

    while (true){
    	__WFI();
    }
}
