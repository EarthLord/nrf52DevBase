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
 * @defgroup radio-driver Radio driver
 * Radio Driver for the nrf52832 SoC to implement BLE protocol
 * @{
 *
 * @file
 * This file contains the Radio Driver declarations for the nrf52832 SoC
 * @author
 * Prithvi
 */

#ifndef NRF52_RADIO_H_
#define NRF52_RADIO_H_

#include <stdint.h>
#include "ble.h"

typedef struct {
	/* @ref adv_type_param */
	adv_pdu_types_t adv_type;
	/* @ref adrs_type_param*/
	adrs_type_t own_adrs_type;
//	Peer_Address_Type,
//	Peer_Address,
	/* @ref ch_map_param.  */
	uint8_t adv_ch_map;
//	Advertising_Filter_Policy;
} radio_adv_param_t;

void radio_set_adv_param(radio_adv_param_t * adv_param);

int8_t radio_get_adv_tx_power(void);

void radio_set_adv_tx_power(int8_t pwr);

void radio_set_random_adrs(uint8_t * rand_adrs);

void radio_set_adv_data(uint8_t len, uint8_t* data_ptr);

void radio_set_scan_rsp_data(uint8_t len, uint8_t* data_ptr);

void radio_prepare_adv(void);

void radio_send_adv(void);

void radio_init(void);

void radio_deinit(void);

/** @} */
#endif /* NRF52_RADIO_H_ */
