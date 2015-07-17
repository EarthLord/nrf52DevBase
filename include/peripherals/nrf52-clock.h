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
 * @defgroup clocks-init Clock Setup
 * Driver to initialize and de-initialize both the HF and LF clock in nrf52832 SoC
 * @{
 *
 * @file
 * This file contains the declarations for HF and LF clock initialization and de-initialization
 * @author
 * Prithvi
 */

#ifndef CLOCK_INIT_H_
#define CLOCK_INIT_H_

#include "nrf.h"

/** Define the source of the LF Clock. Used in @ref lfclk_init. */
#define SRC_LFCLK 					CLOCK_LFCLKSRC_SRC_Xtal
/** HFCLK frequency in Hertz, constant of 64 MHz. */
#define HFCLK_FREQUENCY		  		(64000000UL)
/** LFCLK frequency in Hertz, constant of 32.768 kHz. */
#define LFCLK_FREQUENCY           	(32768UL)

/** Function to initialize the LF clock */
void lfclk_init(void);

/** Function to de-initialize the LF clock.
 * Saves power as LF clock is not running, but peripherals which use LFCLK cannot run */
void lfclk_deinit(void);

/** Function to initialize the HF clock to use the crystal */
void hfclk_xtal_init(void);

/** Function to de-initialize the HF clock from using the crystal.
 * RC oscillator will be used to generate HF clock. Saves power but not accurate. */
void hfclk_xtal_deinit(void);

/** Function that switches to the constant latency mode. In this mode the HFCLK is
 *  kept on in idle mode which gives fast wake up and higher power consumption. */
static inline void power_mode_const_latency();

/** Function that switches to the low power mode. In this mode the HFCLK is
 *  turned off if not required in idle mode which low power consumption but high
 *  wake up time. */
static inline void power_mode_low_power();

static inline void power_mode_const_latency(){
	NRF_POWER->TASKS_CONSTLAT = 1;
}

static inline void power_mode_low_power(){
	NRF_POWER->TASKS_LOWPWR = 1;
}

#endif /* CLOCK_INIT_H_ */
/**
 * @}
 * @}
 */
