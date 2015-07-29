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
 * @file Link layer definitions
 */

#ifndef LINK_LAYER_H_
#define LINK_LAYER_H_

#include <stdbool.h>
#include <stdint.h>
#include "ble.h"
#include "nrf52-radio.h"

/** @brief Set the advertising transmission power in dBm
 * 	@param pwr The tx power ranging from -40 to 4
 */
inline void ll_set_adv_tx_power(int8_t pwr);

/** @brief Get the advertising transmission power in dBm
 * 	@return The tx power used
 */
inline int8_t ll_get_adv_tx_power(void);

/** @brief Start advertising based on the parameters set
 */
void ll_start_adv(void);

/** @brief Stop advertising
 */
void ll_stop_adv(void);

/** @brief Set the advertising data
 * 	@param len Length of the advertising data
 * 	@param data_ptr	Pointer to the buffer containing the data_ptr
 */
inline void ll_set_adv_data(uint8_t len, uint8_t* data_ptr);

/***** Defines for advertising parameters (BLE Spec 4.2 Vol 2, Part E, 7.8.5 Page No:1277)  *****/

typedef enum {
	 ADV_IND_PARAM, 			// Connectable undirected advertising
	 ADV_DIRECT_IND_PARAM, 		// Connectable high duty cycle directed advertising (high duty cycle)
	 ADV_SCAN_IND_PARAM, 		// Scannable undirected advertising
	 ADV_NONCONN_IND_PARAM 		// Non connectable undirected advertising
	 //ADV_DIRECT_IND 	 		// Connectable low duty cycle directed advertising(low duty cycle)
}hci_adv_type_t;

typedef enum {
	CH_37_PARAM = 1,
	CH_38_PARAM,
	CH_37_38_PARAM,
	CH_39_PARAM,
	CH_37_39_PARAM,
	CH_38_39_PARAM,
	CH_ALL_PARAM
}ll_adv_ch_map_t;

typedef struct {
	/* Range: 0x0020 to 0x4000; Time = N * 0.625 msec; Time Range: 20 ms to 10.24 sec */
	uint16_t adv_intvl;
	/* @ref adv_type_param */
	hci_adv_type_t adv_type;
	/* @ref adrs_type_param*/
	adrs_type_t own_adrs_type;
//	Peer_Address_Type,
//	Peer_Address,
	/* @ref ch_map_param.  */
	ll_adv_ch_map_t adv_ch_map;
//	Advertising_Filter_Policy;
} adv_param_t;

/** @brief Set the advertising parameters to be used
 *  @param adv_param Pointer to the structure containing the parameters
 */
void ll_set_adv_param(adv_param_t * adv_param);

/** @brief Set the random address to be used
 *  @param rand_adrs Pointer to the buffer containing the random address containing
 *  				@ref ADRS_LEN number of octets
 */
inline void ll_set_random_adrs(uint8_t * rand_adrs);

/** @brief Initialize the link layer
 */
//void ll_init(void);

inline int8_t ll_get_adv_tx_power(void){
	return radio_get_adv_tx_power();
}

inline void ll_set_adv_tx_power(int8_t pwr){
	radio_set_adv_tx_power(pwr);
}

inline void ll_set_random_adrs(uint8_t * rand_adrs){
	radio_set_random_adrs(rand_adrs);
}

inline void ll_set_adv_data(uint8_t len, uint8_t* data_ptr){
	radio_set_adv_data(len, data_ptr);
}

inline void ll_set_scan_rsp_data(uint8_t len, uint8_t* data_ptr){
	radio_set_scan_rsp_data(len, data_ptr);
}

#endif /* LINK_LAYER_H_ */
