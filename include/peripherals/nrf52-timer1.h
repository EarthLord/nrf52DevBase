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
 * @addtogroup peripheral-drivers
 * @{
 *
 * @defgroup micro-timer Micro-Second Timer
 * Driver to use micro-second timers using TOMER1 peripheral
 * @{
 *
 * @file
 * This file contains the declarations for micro-second timers using TIMER1 peripheral
 * @author
 * Prithvi
 */

#ifndef NRF52_TIMER1_H_
#define NRF52_TIMER1_H_

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf52-clock.h"

#define DIV_ROUNDED(x,y) 			((x + y/2)/y)

/** Prescalar to the HF Clock for the TIMER0 peripheral based on f = HFCLK/(2^prescaler)
 * 	With 4, the TIMER ticks every us. Also it'll use the PCLK1M lowering power used.*/
#define TIMER1_PRESCALER 	4
/** Number of bits in the timer. TIMER 1 and 2 can only be 8 or 16 bit */
#define TIMER1_BITSIZE 		TIMER_BITMODE_BITMODE_32Bit

/**
 * @enum timer_num
 * @brief Enumeration used for specifying the four timers that can be used with a RTC peripheral
 */
typedef enum {
	US_TIMER0,  //!< Microsecond Timer 0
	US_TIMER1,  //!< Microsecond Timer 1
	US_TIMER2,  //!< Microsecond Timer 2
	US_TIMER3,  //!< Microsecond Timer 3
	US_TIMER_MAX//!< Not a timer, just used to find the number of timers
}us_timer_num;

/**
 * @enum timer_mode
 * @brief Enumeration to specify the mode of operation of the timer
 */
typedef enum {
	US_SINGLE_CALL, //!< One shot call of the timer
	US_REPEATED_CALL//!< Repeated call of the timer
}us_timer_mode;

/**
 * Initialize the TIMER1 peripheral to use as a micro-second timer.
 */
void us_timer_init(void);

/**
 * Start a micro-second timer
 */
void start_us_timer(us_timer_num id, us_timer_mode mode, uint32_t ticks, void (*handler)(void));

/**
 * Stop a milli-second timer
 * @param id		ID of the timer to be stopped
 *
 * @note Stopping an already stopped timer will not be an issue
 */
void stop_us_timer(us_timer_num id);

/**
 * Returns if a timer is on
 * @param id		ID of timer being enquired
 * @return			Boolean value indicating if a timer is ON
 */
bool is_us_timer_on(us_timer_num id);


#endif /* NRF52_TIMER1_H_ */
