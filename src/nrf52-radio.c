/* 	Copyright (c) 2015, Prithvi Raj Narendra
 *  Copyright (c) 2013 Paulo B. de Oliveira Filho <pauloborgesfilho@gmail.com>
 *  Copyright (c) 2013 Claudio Takahasi <claudio.takahasi@gmail.com>
 *  Copyright (c) 2013 João Paulo Rechi Vita <jprvita@gmail.com>
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


#include "nrf52-radio.h"
#include "nrf52-clock.h"
#include "nrf52-timer1.h"
#include "nrf52-timer0.h"
#include "tfp_printf.h"
#include "nrf.h"
#include "irq_priority.h"
#include <string.h>

static inline void adv_set_ch_freq();
//static inline int8_t ch2freq(uint8_t ch);

/* Link Layer specification Section 2.1.2, Core 4.1 page 2503 */
#define ADV_ACCESS_ADRS				0x8E89BED6

/* Link Layer specification Section 3.1.1, Core 4.1 page 2522 */
#define ADV_CRC_INIT				0x555555

#define ADV_HEADER_PDU_OFFSET		0
#define ADV_HEADER_LEN_OFFSET		1
#define ADV_ADRS_OFFSET				2
#define ADV_PAYLOAD_OFFSET			8

#define ADV_TX_ADRS_TYPE_BIT_POS	6
#define ADV_RX_ADRS_TYPE_BIT_POS	7

#define MAX_PAYLOAD_LEN				37
#define MAX_RADIO_PDU				39

#define ADV_IDX_CH_37				0
#define ADV_IDX_CH_38				1
#define ADV_IDX_CH_39				2

#define BASE_SHORTS							\
		(RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos)		\
		| (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos)

const int8_t radio_pwr_levels[] = {4, 3, 0, -4, -8, -12, -16, -20, -40};

const uint8_t adv_channels[] = {37, 38, 39};
const uint8_t adv_freq[] = {2, 26, 80};

void irq_null_end(void);
void irq_adv_nc_end(void);

void irq_null_dis(void);
void irq_adv_nc_dis(void);

typedef enum{
	STOP,
	ADV_NC,
}radio_states;

void (*end_handler[])(void) = {
	irq_null_end,				//IDLE
	irq_adv_nc_end				//ADV_NC
};

void (*dis_handler[])(void) = {
	irq_null_dis,				//IDLE
	irq_adv_nc_dis				//ADV_NC
};

static struct radio_context {
	/** Radio specific **/
	radio_states state;					//4 bytes
	uint8_t adv_buf[MAX_RADIO_PDU];		//39 bytes

	/**General**/
	uint8_t adrs_type;					//1 bytes
	uint8_t MAC_adrs[ADRS_LEN];			//6 bytes

	/**Advertisement**/
	uint8_t adv_type;					//1 bytes
	uint8_t adv_ch_map;					//1 bytes
	int8_t adv_pwr;						//1 bytes
	volatile uint8_t adv_idx;			//1 bytes
} radio_ctx;

#ifdef DEBUG
void add_log(void);
volatile uint32_t log_cnt = LOG_BUFFER_SIZE-1;
volatile log_t log_buf[LOG_BUFFER_SIZE];

void add_log(void){
	if(log_cnt){
		log_buf[log_cnt].time =read_time_us();
		log_buf[log_cnt].radio_state =  NRF_RADIO->STATE;
		log_buf[log_cnt].radio_ctx_state = radio_ctx.state;
		log_buf[log_cnt].freq = radio_ctx.adv_idx;
		log_cnt--;
	}
}

void dump_log(){
	uint32_t i;
	for(i = LOG_BUFFER_SIZE-1; i>log_cnt;i--){
		tfp_printf("Time:");
		printfcomma(log_buf[i].time);
		tfp_printf("; HwState:%d; SwState:%d; Freq:%d\n",
				log_buf[i].radio_state, log_buf[i].radio_ctx_state,
				log_buf[i].freq);
	}
	log_cnt = LOG_BUFFER_SIZE-1;
}

#else
void add_log(void);
void add_log(void){}
void dump_log(void){}
#endif

void radio_set_adv_param(radio_adv_param_t * adv_param){
	radio_ctx.adv_type = adv_param->adv_type;
	radio_ctx.adv_ch_map = adv_param->adv_ch_map;
	radio_ctx.adrs_type = adv_param->own_adrs_type;
}

int8_t radio_get_adv_tx_power(void){
	return radio_ctx.adv_pwr;
}

void radio_set_adv_tx_power(int8_t pwr){
	uint32_t i;
	for(i = (sizeof(radio_pwr_levels)-1); i; i--){
		if(radio_pwr_levels[i] >= pwr){
			radio_ctx.adv_pwr = radio_pwr_levels[i];
			break;
		}
	}
	if(0 == i){
		radio_ctx.adv_pwr = radio_pwr_levels[0];
	}
}

void radio_set_random_adrs(uint8_t * rand_adrs){
	memcpy(radio_ctx.MAC_adrs, rand_adrs , ADRS_LEN);
}

void radio_set_adv_data(uint8_t len, uint8_t* data_ptr){
	memcpy(radio_ctx.adv_buf + ADV_PAYLOAD_OFFSET, data_ptr, len);
	radio_ctx.adv_buf[ADV_HEADER_LEN_OFFSET] = len + ADRS_LEN;
}

static inline void adv_set_ch_freq(){
	//TODO Change for data channels
	NRF_RADIO->DATAWHITEIV = adv_channels[radio_ctx.adv_idx];
	NRF_RADIO->FREQUENCY = adv_freq[radio_ctx.adv_idx];
}

void radio_prepare_adv(void)
{
	if(radio_ctx.adv_ch_map & (1<<ADV_IDX_CH_37)){
		radio_ctx.adv_idx = ADV_IDX_CH_37;
	} else if(radio_ctx.adv_ch_map & (1<<ADV_IDX_CH_38)){
		radio_ctx.adv_idx = ADV_IDX_CH_38;
	} else if(radio_ctx.adv_ch_map & (1<<ADV_IDX_CH_39)){
		radio_ctx.adv_idx = ADV_IDX_CH_39;
	}

	adv_set_ch_freq();

	NRF_RADIO->BASE0 = (ADV_ACCESS_ADRS << 8) & 0xFFFFFF00;
	NRF_RADIO->PREFIX0 = (ADV_ACCESS_ADRS >> 24) & RADIO_PREFIX0_AP0_Msk;
	NRF_RADIO->CRCINIT = ADV_CRC_INIT;
	NRF_RADIO->TXPOWER = radio_ctx.adv_pwr;

	radio_ctx.adv_buf[ADV_HEADER_PDU_OFFSET] = radio_ctx.adv_type;
	radio_ctx.adv_buf[ADV_HEADER_PDU_OFFSET] |=
			(radio_ctx.adrs_type & 0x01) << ADV_TX_ADRS_TYPE_BIT_POS;

	memcpy(radio_ctx.adv_buf + ADV_ADRS_OFFSET, radio_ctx.MAC_adrs, ADRS_LEN);

	NRF_RADIO->PACKETPTR = (uint32_t) radio_ctx.adv_buf;

	radio_ctx.state = STOP;
}

void radio_send_adv(void){
	radio_ctx.state = ADV_NC;
	NRF_RADIO->TASKS_TXEN = 1UL;
}

void radio_init(void){
	hfclk_xtal_init();

	/************ Refer nrf52 manual instead of nrf51 *************/

	NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled;

	/* nRF51 Series Reference Manual v2.1, section 6.1.1, page 18
	 * PCN-083 rev.1.1
	 *
	 * Fine tune BLE deviation parameters. Not required for nrf52.
	if ((NRF_FICR->OVERRIDEEN & FICR_OVERRIDEEN_BLE_1MBIT_Msk)
					== (FICR_OVERRIDEEN_BLE_1MBIT_Override
					<< FICR_OVERRIDEEN_BLE_1MBIT_Pos)) {
		NRF_RADIO->OVERRIDE0 = NRF_FICR->BLE_1MBIT[0];
		NRF_RADIO->OVERRIDE1 = NRF_FICR->BLE_1MBIT[1];
		NRF_RADIO->OVERRIDE2 = NRF_FICR->BLE_1MBIT[2];
		NRF_RADIO->OVERRIDE3 = NRF_FICR->BLE_1MBIT[3];
		NRF_RADIO->OVERRIDE4 = NRF_FICR->BLE_1MBIT[4] | 0x80000000;
	} */

	/* nRF51 Series Reference Manual v2.1, section 16.2.7, page 86 */
	NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos;

	/* Link Layer specification section 4.1, Core 4.1, page 2524
	 * nRF51 Series Reference Manual v2.1, section 16.2.7, page 92
	 *
	 * Set the inter frame space (T_IFS) to 150 us.
	 */
	NRF_RADIO->TIFS = 150;

	/* nRF51 Series Reference Manual v2.1, section 16.2.9, page 88
	 *
	 * Enable data whitening, set the maximum payload length and set the
	 * access address size (3 + 1 octets).
	 */
	NRF_RADIO->PCNF1 =
		(RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos) |
		(MAX_PAYLOAD_LEN << RADIO_PCNF1_MAXLEN_Pos) |
		(3UL << RADIO_PCNF1_BALEN_Pos);

	/* nRF51 Series Reference Manual v2.1, section 16.1.4, page 74
	 * nRF51 Series Reference Manual v2.1, section 16.2.14-15, pages 89-90
	 *
	 * Preset the address to use when receive and transmit packets (logical
	 * address 0, which is assembled by base address BASE0 and prefix byte
	 * PREFIX0.AP0.
	 */
	NRF_RADIO->RXADDRESSES = 1UL;
	NRF_RADIO->TXADDRESS = 0UL;

	/* nRF51 Series Reference Manual v2.1, section 16.1.7, page 76
	 * nRF51 Series Reference Manual v2.1, sections 16.1.16-17, page 90
	 *
	 * Configure the CRC length (3 octets), polynominal and set it to
	 * ignore the access address when calculate the CRC.
	 */
	NRF_RADIO->CRCCNF =
		(RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
		(RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
	NRF_RADIO->CRCPOLY = 0x100065B;

	/* nRF51 Series Reference Manual v2.1, section 16.1.2, page 74
	 * nRF51 Series Reference Manual v2.1, sections 16.1.8, page 87
	 * Link Layer specification section 2.3, Core 4.1, page 2504
	 * Link Layer specification section 2.4, Core 4.1, page 2511
	 *
	 * Configure the header size. The nRF51822 has 3 fields before the
	 * payload field: S0, LENGTH and S1. These fields can be used to store
	 * the PDU header.
	 */
	NRF_RADIO->PCNF0 = (1UL << RADIO_PCNF0_S0LEN_Pos)
			| (8UL << RADIO_PCNF0_LFLEN_Pos)
			| (0UL << RADIO_PCNF0_S1LEN_Pos);

	/* nRF51 Series Reference Manual v2.1, section 16.1.8, page 76
	 * nRF51 Series Reference Manual v2.1, section 16.1.10-11, pages 78-80
	 * nRF51 Series Reference Manual v2.1, section 16.2.1, page 85
	 *
	 * Enable READY_START short: when the READY event happens, initialize
	 * the START task.
	 *
	 * Enable END_DISABLE short: when the END event happens, initialize the
	 * DISABLE task.
	 */
	NRF_RADIO->SHORTS = BASE_SHORTS;

	/* Trigger RADIO interruption when an DISABLE or END event happens */
	NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk | RADIO_INTENSET_DISABLED_Msk;

	NVIC_SetPriority(RADIO_IRQn, PRIORITY_RADIO_IRQn);
	NVIC_ClearPendingIRQ(RADIO_IRQn);
	NVIC_EnableIRQ(RADIO_IRQn);
}

void radio_deinit(void){
	radio_ctx.state = STOP;
	NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled;
}

void irq_null_end(void){}
void irq_null_dis(void){}

void irq_adv_nc_end(void){
	//add_log();
}


void irq_adv_nc_dis(void){
	add_log();

	switch(radio_ctx.adv_idx){
		case ADV_IDX_CH_37:
			if(radio_ctx.adv_ch_map & (1<<ADV_IDX_CH_38)){
				radio_ctx.adv_idx = ADV_IDX_CH_38;
			} else if(radio_ctx.adv_ch_map & (1<<ADV_IDX_CH_39)){
				radio_ctx.adv_idx = ADV_IDX_CH_39;
			} else {
				radio_deinit();
				return;
			}
			break;
		case ADV_IDX_CH_38:
			if(radio_ctx.adv_ch_map & (1<<ADV_IDX_CH_39)){
				radio_ctx.adv_idx = ADV_IDX_CH_39;
			} else {
				radio_deinit();
				return;
			}
			break;
		case ADV_IDX_CH_39:
		default:
			radio_deinit();
			return;
			break;
	}

	adv_set_ch_freq();
	NRF_RADIO->TASKS_TXEN = 1UL;
}


void RADIO_IRQHandler(void){
	if(1 == NRF_RADIO->EVENTS_RSSIEND){
		NRF_RADIO->EVENTS_RSSIEND = 0;
		(void) NRF_RADIO->EVENTS_RSSIEND;
	}

	if(1 == NRF_RADIO->EVENTS_END){
		NRF_RADIO->EVENTS_END = 0;
		(void) NRF_RADIO->EVENTS_END;
		end_handler[radio_ctx.state]();
	}

	if((1 == NRF_RADIO->EVENTS_DISABLED) && (0 == NRF_RADIO->EVENTS_END)){
		NRF_RADIO->EVENTS_DISABLED = 0;
		(void) NRF_RADIO->EVENTS_DISABLED;
		dis_handler[radio_ctx.state]();
	}
}

