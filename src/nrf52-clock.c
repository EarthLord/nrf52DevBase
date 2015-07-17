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
 * @addtogroup clocks-init
 * @{
 *
 * @file
 * This file contains the implementation for HF and LF clock initialization and de-initialization
 * @author
 * Prithvi
 */

#include "nrf52-clock.h"
#include "nrf.h"

void lfclk_init(void)
{
  if(!(NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Running)){
	NRF_CLOCK->LFCLKSRC = (SRC_LFCLK << CLOCK_LFCLKSRC_SRC_Pos);

    NRF_CLOCK->INTENSET = CLOCK_INTENSET_LFCLKSTARTED_Msk;
    // Enable wake-up on event
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	/* Wait for the external oscillator to start up. */
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0){
	  __WFE();
    }
    /* Clear the event and the pending interrupt */
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
  	NRF_CLOCK->INTENSET = 0;

  	//Due to errata 20 in Eng rev A
	NRF_RTC1->TASKS_STOP = 0;
	NRF_RTC0->TASKS_STOP = 0;
  }
}

void lfclk_deinit(void)
{
	//Due to errata 49 in Eng rev A
  //NRF_CLOCK->TASKS_LFCLKSTOP = 1;
}

void hfclk_xtal_init(void)
{
  /* Check if 16 MHz crystal oscillator is already running. */
  if (!(NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Xtal)){
	  NRF_CLOCK->INTENSET = CLOCK_INTENSET_HFCLKSTARTED_Msk;
	  // Enable wake-up on event
	  SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

	  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_HFCLKSTART = 1;
	  /* Wait for the external oscillator to start up. */
	  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0){
		__WFE();
	  }
	  /* Clear the event and the pending interrupt */
	  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	  NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
	  NRF_CLOCK->INTENSET = 0;
  }
}

void hfclk_xtal_deinit(void)
{
  NRF_CLOCK->TASKS_HFCLKSTOP = 1;
}
/**
 * @}
 */
