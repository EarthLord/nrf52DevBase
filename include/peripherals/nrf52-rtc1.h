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
 * @addtogroup peripheral-drivers
 * @{
 *
 * @defgroup rtc-timer Milli-Second RTC Timer
 * Driver to use milli-second timers using RTC1 peripheral
 * @{
 *
 * @file
 * This file contains the declarations for milli-second timers using RTC1 peripheral
 * @author
 * Prithvi
 */

#ifndef NRF52_RTC1_H
#define NRF52_RTC1_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf52-clock.h"

#define DIV_ROUNDED(x,y) 			((x + y/2)/y)

/** RTC clock freq from LF clock in Hertz. Changable, but should be usable by @ref RTC_TICKS */
#define RTC_FREQUENCY				32768
/* Check if RTC_FREQUENCY is power of 2 and less than or equal to 32.768 kHz */
#if (!(!(RTC_FREQUENCY & (RTC_FREQUENCY-1)) && RTC_FREQUENCY && (RTC_FREQUENCY<=32768)))
#error RTC_FREQUENCY must be a power of 2 with a maximum frequency of 32768 Hz
#endif
/** Prescalar to the LF Clock for the RTC peripheral based on f = LFCLK/(prescaler + 1) */
#define RTC_PRESCALER         		((LFCLK_FREQUENCY/RTC_FREQUENCY) - 1)
/** Calculation to findout the number of RTC ticks for the passed time in milli-seconds */
#define RTC_TICKS_MS(ms)				((uint32_t) DIV_ROUNDED( (RTC_FREQUENCY*ms) , 1000) )
/** Calculation to findout the number of RTC ticks for the passed time in multiples of 0.625 ms */
#define RTC_TICKS_625(ms)				((uint32_t) DIV_ROUNDED( (RTC_FREQUENCY*ms) , 1600) )
/** Calculation to findout the number of RTC ticks for the passed time in multiples of 1.25 ms */
#define RTC_TICKS_1250(ms)				((uint32_t) DIV_ROUNDED( (RTC_FREQUENCY*ms) , 800) )

/**
 * @enum timer_num
 * @brief Enumeration used for specifying the four timers that can be used with a RTC peripheral
 */
typedef enum {
	MS_TIMER0,  //!< Millisecond Timer 0
	MS_TIMER1,  //!< Millisecond Timer 1
	MS_TIMER2,  //!< Millisecond Timer 2
	MS_TIMER3,  //!< Millisecond Timer 3
	MS_TIMER_MAX//!< Not a timer, just used to find the number of timers
}ms_timer_num;

/**
 * @enum timer_mode
 * @brief Enumeration to specify the mode of operation of the timer
 */
typedef enum {
	MS_SINGLE_CALL, 	//!< One shot call of the timer
	MS_REPEATED_CALL	//!< Repeated call of the timer
}ms_timer_mode;

/**
 * Initialize the RTC1 peripheral to use as a milli-second timer.
 * Uses @ref RTC_PRESCALER
 */
void ms_timer_init(void);

/**
 * Start a milli-second timer
 * @param id		ID of the timer to be used from @ref timer_num
 * @param mode  	Mode of the timer as specified in @ref timer_mode
 * @param ticks 	The number of ticks of the RTC after which the timer expires
 * @param handler 	Pointer to a function which needs to be called on the expiry of the timer
 *
 * @note Starting an already started will restart the timer with the current number of ticks passed.
 * @ref is_ms_timer_on can be used to check if a timer is already running.
 */
void start_ms_timer(ms_timer_num id, ms_timer_mode mode, uint32_t ticks, void (*handler)(void));

/**
 * Stop a milli-second timer
 * @param id		ID of the timer to be stopped
 *
 * @note Stopping an already stopped timer will not be an issue
 */
void stop_ms_timer(ms_timer_num id);

/**
 * Returns if a timer is on
 * @param id		ID of timer being enquired
 * @return			Boolean value indicating if a timer is ON
 */
bool is_ms_timer_on(ms_timer_num id);

#endif /* NRF52_RTC1_H */
/**
 * @}
 * @}
 */
