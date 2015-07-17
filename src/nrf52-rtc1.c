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
 * @addtogroup rtc-timer
 * @{
 *
 * @file
 * This file contains the implementation for milli-second timers using RTC1 peripheral
 * @author
 * Prithvi
 * \todo In all functions make sure the parameters passed are valid with proper error codes
 */

#include "nrf52-rtc1.h"
#include "irq_priority.h"
#include <stddef.h>

/**
 * Structure to hold the @ref timer_mode and handler to be called upon expiry
 */
struct ms_timer{
	uint32_t timer_mode_t;
	void (*timer_handler)(void);
}ms_timer_t[MS_TIMER_MAX];

/** Timers currently used based on the first four bits from LSB */
static uint32_t ms_timers_status;

void ms_timer_init(void){

	NRF_RTC1->TASKS_STOP = 1;

	for(uint32_t i = MS_TIMER0; i < MS_TIMER_MAX; i++){
		ms_timer_t[i].timer_mode_t = SINGLE_CALL;
		ms_timer_t[i].timer_handler  = NULL;
	}

	ms_timers_status = 0;
	NRF_RTC1->PRESCALER = RTC_PRESCALER;      // Set prescaler to a TICK of RTC_FREQUENCY.
	NVIC_SetPriority(RTC1_IRQn,PRIORITY_RTC1_IRQn);
	NVIC_EnableIRQ(RTC1_IRQn);    			  // Enable Interrupt for RTC1 in the core.
}

/**@todo Take care of values of ticks passed which are greater than 2^24, now it is suppressed to 2^24 when greater
 * @warning Works only for input less than 512000 milli-seconds, or 8.5 min without when @ref RTC_PRESCALER is 0 */
void start_ms_timer(timer_num id, timer_mode mode, uint32_t ticks, void (*handler)(void)){

	/* make sure the number of ticks to interrupt is less than 2^24 */
	ticks &= 0xFFFFFF;

	ms_timer_t[id].timer_handler = handler;
	if(mode == SINGLE_CALL){
		ms_timer_t[id].timer_mode_t  = SINGLE_CALL;
	}else{
		ms_timer_t[id].timer_mode_t  = ticks;
	}

	NRF_RTC1->CC[id]	= NRF_RTC1->COUNTER + ticks;

	NRF_RTC1->EVENTS_COMPARE[id] = 0;
	NRF_RTC1->EVTENSET 		= 1 << (RTC_INTENSET_COMPARE0_Pos + id);
	NRF_RTC1->INTENSET 		= 1 << (RTC_INTENSET_COMPARE0_Pos + id);

	if(ms_timers_status == 0){
		lfclk_init();
		NRF_RTC1->TASKS_START = 1;
	}
	ms_timers_status |= 1 << id;
}

void stop_ms_timer(timer_num id){
	ms_timer_t[id].timer_mode_t  = SINGLE_CALL;
	ms_timer_t[id].timer_handler = NULL;
	ms_timers_status &= ~(1 << id);
	NRF_RTC1->EVTENCLR 		= 1 << (RTC_INTENSET_COMPARE0_Pos + id);
	NRF_RTC1->INTENCLR 		= 1 << (RTC_INTENSET_COMPARE0_Pos + id);

	if(ms_timers_status == 0){
		NRF_RTC1->TASKS_STOP = 1;
		//Check if other peripherals are using LF_CLK and call lfclk_deinit();
	}
}

bool is_ms_timer_on(timer_num id){
	if((ms_timers_status & (1<<id)) == 0){
		return false;
	}
	else{
		return true;
	}
}

/** @brief Function for handling the RTC1 interrupts.
 * Triggered Compare register of timer ID
 */
void
RTC1_IRQHandler(){
	for(uint32_t i = MS_TIMER0; i < MS_TIMER_MAX; i++){
		if(NRF_RTC1->EVENTS_COMPARE[i]){
			NRF_RTC1->EVENTS_COMPARE[i] = 0;

			if(ms_timer_t[i].timer_handler != NULL){
				ms_timer_t[i].timer_handler();
			}

			if(ms_timer_t[i].timer_mode_t == SINGLE_CALL){
				stop_ms_timer(i);
			}else{
				NRF_RTC1->CC[i] += ms_timer_t[i].timer_mode_t;
			}
		}
	}
}
/**
 * @}
 */
