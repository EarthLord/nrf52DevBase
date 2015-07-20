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
 * @defgroup hf-timer Profile Timer
 * Driver to use timer based on TIMER0 for code profiling and time-stamping
 * @{
 *
 * @file
 * This file contains the declarations for profiling and time-stamping timer using TIMER0 peripheral
 * @author
 * Prithvi
 *
 */

#ifndef NRF52_TIMER0_H_
#define NRF52_TIMER0_H_

#include <stdint.h>
#include "nrf.h"

/** Prescalar to the HF Clock for the TIMER0 peripheral based on f = HFCLK/(2^prescaler) */
#define TIMER0_PRESCALER 0
/** Number of bits in the timer. TIMER 1 and 2 can only be 8 or 16 bit */
#define TIMER0_BITSIZE 	TIMER_BITMODE_BITMODE_32Bit

/** Print the current time in micro-seconds from the startup (beginning of TIMER0).
 * This is used for time stamping at different parts in the code
 * @todo Make the @ref PRINT_TIME self configuring based on prescalar used */
#define PRINT_TIME		do{	NRF_TIMER0->TASKS_CAPTURE[3] = 1; \
						printfcomma(NRF_TIMER0->CC[3]/16); \
						tfp_printf("us\n");	}while(0)

/** @anchor profile-start-stop
 * @name Definitions for marking the beginning and ending of code to profile
 * @todo Make the @ref PROFILE_START and @ref PROFILE_STOP self configuring based on prescalar used
 * @{*/
/** Point of start of profiling */
#define PROFILE_START	do{	NRF_TIMER0->TASKS_CAPTURE[2] = 1; }while(0)
/** Point of end of profiling. The time from start is displayed in nano-seconds with an accuracy of 62.5 ns */
#define PROFILE_STOP	do{ NRF_TIMER0->TASKS_CAPTURE[3] = 1; \
						printfcomma((NRF_TIMER0->CC[3] - NRF_TIMER0->CC[2])/16);	  \
						tfp_printf(".%03d",(int)((((NRF_TIMER0->CC[3] - NRF_TIMER0->CC[2]) & 0x0F)*125)/2)); \
						tfp_printf("ns\n");	}while(0)
/** @} */

/** Initialize the Timer using TIMER0 using @ref TIMER_PRESCALER and @ref TIMER_BITSIZE */
void profile_timer_init(void);

/**
 * Takes in an unsigned integer and prints it with a ',' after every three digits
 * @param num	Number to be printed with commas
 */
void printfcomma(uint32_t num);

/**
 * The time from start-up in micro-seconds
 * @return Time from start, the four most significant bits are always zero
 * @todo Take care of overflow of time
 * @warning The time overflows after 268 seconds
 */
inline uint32_t read_time_us(void){
	NRF_TIMER0->TASKS_CAPTURE[3] = 1;
	return(NRF_TIMER0->CC[3]/16);
}

/** @brief Fully stop the profiling timer (TIMER0) to save power. @ref profile_timer_init
 *  	needs to be called again before using it.
 */
void profile_timer_deinit();

#endif /* NRF52_TIMER0_H_ */
/**
 * @}
 * @}
 */
