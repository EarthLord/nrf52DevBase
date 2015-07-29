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
 * @brief Defines and declarations relevant for many layers of BLE
 */

#ifndef BLE_H_
#define BLE_H_

#include "stdint.h"
#include "stdbool.h"

#define DEBUG

#ifdef DEBUG

#define LOG_BUFFER_SIZE		128

typedef struct  {
	uint32_t time;
	const char* func_name;
	uint8_t radio_state;
	uint8_t	radio_ctx_state;
	uint8_t freq;
}log_t;

void dump_log(void);

#else

#endif

/**************** Link Layer ****************/

/** @brief length of MAC address in BLE */
#define ADRS_LEN			6
#define ADV_HEADER_LEN		2

typedef enum{
	 ADV_IND, 			// Connectable undirected advertising
	 ADV_DIRECT_IND,	// Connectable high duty cycle directed advertising (high duty cycle)
	 ADV_NONCONN_IND,	// Non connectable undirected advertising
	 SCAN_REQ,			// Scan request from scanner
	 SCAN_RSP,			// Scan response from advertiser
	 CONNECT_REQ,		// Connect request from initiator
	 ADV_SCAN_IND, 		// Scannable undirected advertising
	 ADV_PDU_MASK
}adv_pdu_types_t;

typedef enum {
	PUBLIC_ADRS_PARAM,
	RANDOM_ADRS_PARAM
	/* Page 1279 of 4.2 spec. Controller generates Resolvable Private Address based on the local
	IRK from resolving list. If resolving list contains no matching entry,
	use public/private address. */
	//RESV_PRIV_ADRS_PUBLIC
	//RESV_PRIV_ADRS_PRIVATE
}adrs_type_t;


/**************** GAP ****************/

#define GAP_ADV_FLAGS  			0x1
#define GAP_ADV_UUID16_INCOMP  	0x2
#define GAP_ADV_UUID16_ALL 		0x3
#define GAP_ADV_UUID32_INCOMP  	0x4
#define GAP_ADV_UUID32_ALL 		0x5
#define GAP_ADV_UUID128_INCOMP 	0x6
#define GAP_ADV_UUID128_ALL		0x7
#define GAP_ADV_NAME_SHORT  	0x8
#define GAP_ADV_NAME_FULL		0x9
#define GAP_ADV_TRANSMIT_PWR 	0xA
#define GAP_ADV_CONN_INTERVAL 	0x12
#define GAP_ADV_SERVICE_DATA 	0x16

#endif /* BLE_H_ */
