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

#include "link-layer.h"
#include "nrf52-radio.h"
#include "nrf52-rtc1.h"

/** @brief length of MAC address in BLE
 */
#define ADRS_LEN			6

static struct ll_context {
	/**Advertisement**/
	uint16_t adv_intvl;

} ll_ctx;

void ll_set_adv_param(adv_param_t * adv_param){
	ll_ctx.adv_intvl = adv_param->adv_intvl;

	radio_adv_param_t radio_adv_param;
	switch(adv_param->adv_type){
		case ADV_IND_PARAM:
			radio_adv_param.adv_type = ADV_IND;
			break;
		case ADV_DIRECT_IND_PARAM:
			radio_adv_param.adv_type = ADV_DIRECT_IND;
			break;
		case ADV_SCAN_IND_PARAM:
			radio_adv_param.adv_type = ADV_SCAN_IND;
			break;
		case ADV_NONCONN_IND_PARAM:
			radio_adv_param.adv_type = ADV_NONCONN_IND;
			break;
	}

	radio_adv_param.adv_ch_map = adv_param->adv_ch_map;
	radio_adv_param.own_adrs_type = adv_param->own_adrs_type;

	radio_set_adv_param(&radio_adv_param);
}

void adv_intvl_handler(void){
	radio_prepare_adv();
	radio_send_adv();
}

void ll_start_adv(void){
	radio_init();
	start_ms_timer(MS_TIMER1, MS_REPEATED_CALL, RTC_TICKS_625(ll_ctx.adv_intvl), adv_intvl_handler);
	adv_intvl_handler();
}

void ll_stop_adv(void){
	stop_ms_timer(MS_TIMER1);
}
