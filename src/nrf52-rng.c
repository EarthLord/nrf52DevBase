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

#include <stdbool.h>
#include <nrf.h>
#include "irq_priority.h"
#include "nrf52-rng.h"

static volatile bool rng_done;
static volatile uint8_t *rng_ptr;
static volatile uint32_t rng_len;

void RNG_IRQHandler(void){
	NRF_RNG->EVENTS_VALRDY = 0;	
	(void) NRF_RNG->EVENTS_VALRDY;
	if (rng_len){
		*rng_ptr = NRF_RNG->VALUE;
		rng_ptr++;
		rng_len--;
	}else{
		rng_done = true;
	}		
}

void rng_get_bytes(uint8_t * rng_buf_ptr, uint32_t rng_buf_len){
	rng_done = false;
	rng_ptr = rng_buf_ptr;
	rng_len = rng_buf_len;

	NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Enabled;
	NRF_RNG->INTENSET = (RNG_INTENSET_VALRDY_Set << RNG_INTENSET_VALRDY_Pos);
	NRF_RNG->EVENTS_VALRDY = 0;

	NVIC_SetPriority(RNG_IRQn, PRIORITY_RNG_IRQn);
	NVIC_EnableIRQ(RNG_IRQn);	

	NRF_RNG->TASKS_START = 1;
	while (false == rng_done){
		__WFI();
	}
	NRF_RNG->TASKS_STOP = 1;

	NRF_RNG->INTENCLR = (RNG_INTENCLR_VALRDY_Clear << RNG_INTENCLR_VALRDY_Pos);
	NVIC_DisableIRQ(RNG_IRQn);	
}

