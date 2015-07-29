/*
 * nrf52-timer1.c
 *
 *  Created on: 17-Jul-2015
 *      Author: prithvi
 */

#include "nrf52-timer1.h"
#include "irq_priority.h"
#include <stddef.h>

//TODO Implement a deinit function to shutdown the Timer to save power

/**
 * Structure to hold the @ref timer_mode and handler to be called upon expiry
 */
static struct us_timer_t{
	volatile uint32_t timer_mode;
	void (*timer_handler)(void);
}us_timer[US_TIMER_MAX];

/** Timers currently used based on the first four bits from LSB */
static volatile uint32_t us_timers_status;

void us_timer_init(void){
	/* Initialize the HF clock if it is not already running*/
	hfclk_xtal_init();

	for(us_timer_num id = US_TIMER0; id < US_TIMER_MAX; id++){
		us_timer[id].timer_mode = US_SINGLE_CALL;
		us_timer[id].timer_handler  = NULL;
	}
	us_timers_status = 0;

    NRF_TIMER1->TASKS_STOP	   	= 1;                    	// Stop timer.
	NRF_TIMER1->MODE           	= TIMER_MODE_MODE_Timer;  	// Set the timer in Timer Mode.
	NRF_TIMER1->PRESCALER      	= TIMER1_PRESCALER;			// Prescaler 4 produces 1 MHz.
	NRF_TIMER1->BITMODE        	= TIMER1_BITSIZE;  			// 32 bit mode.
	NRF_TIMER1->TASKS_CLEAR    	= 1;                        // clear the Timer first to be usable for later.
	NRF_TIMER1->TASKS_START 	= 1;

	NVIC_SetPriority(TIMER1_IRQn, PRIORITY_TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
}

void start_us_timer(us_timer_num id, us_timer_mode mode, uint32_t ticks, void (*handler)(void)){
	us_timer[id].timer_handler = handler;
	if(mode == US_SINGLE_CALL){
		us_timer[id].timer_mode  = US_SINGLE_CALL;
	}else{
		us_timer[id].timer_mode  = ticks;
	}

	NRF_TIMER1->TASKS_CAPTURE[id] = 1;
	NRF_TIMER1->CC[id] += ticks;

	NRF_TIMER1->EVENTS_COMPARE[id] 	= 0;
	NRF_TIMER1->INTENSET 			= 1 << (TIMER_INTENSET_COMPARE0_Pos + id);

	us_timers_status |= 1 << id;
}

void stop_us_timer(us_timer_num id){
	us_timer[id].timer_mode  = US_SINGLE_CALL;
	us_timers_status &= ~(1 << id);

	NRF_TIMER1->INTENCLR 		= 1 << (TIMER_INTENSET_COMPARE0_Pos + id);
}

void us_timer_deinit(){
    NRF_TIMER1->TASKS_STOP	   		= 1;               		// Stop timer.
    NRF_TIMER1->TASKS_SHUTDOWN	   	= 1;               		// Fully stop timer.
}

bool is_us_timer_on(us_timer_num id){
	return ((us_timers_status & (1<<id)) != 0);
}

/** @brief Function for handling the RTC1 interrupts.
 * Triggered Compare register of timer ID
 */
void TIMER1_IRQHandler(){
	for(us_timer_num id = US_TIMER0; id < US_TIMER_MAX; id++){
		if(NRF_TIMER1->EVENTS_COMPARE[id]){
			NRF_TIMER1->EVENTS_COMPARE[id] = 0;

			if(us_timer[id].timer_handler != NULL){
				us_timer[id].timer_handler();
			}

			if(us_timer[id].timer_mode == US_SINGLE_CALL){
				stop_us_timer(id);
			}else{
				NRF_TIMER1->TASKS_CAPTURE[id] = 1;
				NRF_TIMER1->CC[id] += us_timer[id].timer_mode;
			}
		}
	}
}
