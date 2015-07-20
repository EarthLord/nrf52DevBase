/* 	Copyright (c) 2014, Prithvi Raj Narendra
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
 * @addtogroup hf-timer
 * @{
 *
 * @file
 * This file contains the implementation for profiling and time-stamping timer using TIMER0 peripheral
 * @author
 * Prithvi
 */

#include "nrf52-timer0.h"
#include "nrf52-clock.h"
#include "tfp_printf.h"

//TODO Add state and checks to see if timer is on before using it

void profile_timer_init(void){
	/* Initialize the HF clock if it is not already running*/
	hfclk_xtal_init();

    NRF_TIMER0->TASKS_STOP	   = 1;                    		// Stop timer.
	NRF_TIMER0->MODE           = TIMER_MODE_MODE_Timer;  	// Set the timer in Timer Mode.
	NRF_TIMER0->PRESCALER      = TIMER0_PRESCALER;			// Prescaler 0 produces 16 MHz.
	NRF_TIMER0->BITMODE        = TIMER0_BITSIZE;  			// 32 bit mode.
	NRF_TIMER0->TASKS_CLEAR    = 1;                         // clear the task first to be usable for later.

    NRF_TIMER0->TASKS_START   = 1;                    		// Start timer.
}

void profile_timer_deinit(){
    NRF_TIMER0->TASKS_STOP	   		= 1;               		// Stop timer.
    NRF_TIMER0->TASKS_SHUTDOWN	   	= 1;               		// Fully stop timer.
}

void printfcomma (uint32_t num) {
    if (num < 1000) {
        tfp_printf ("%d", (int)num);
        return;
    }
    printfcomma (num/1000);
    tfp_printf (",%03d",(int) num%1000);
}

/**
 * @}
 */
