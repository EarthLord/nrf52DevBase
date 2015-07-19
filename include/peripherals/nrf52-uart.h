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
 * @defgroup uart-driver UART Driver
 * UART Driver for the nrf52832 SoC
 * supports single character printing, stream to printf and a character stream receiver handler
 * @{
 *
 * @file
 * This file contains the UART Driver declarations for the nrf52832 SoC
 * @author
 * Prithvi
 */

#ifndef NRF52_UART_H
#define NRF52_UART_H

#include <stdbool.h>
#include <stdint.h>

/** Baudrate to be used by the UART module. Used by @ref uart_init.
 * In PCA10036 the maximum baudrate is 1Mbps */
#define UARTE_BAUDRATE 			UARTE_BAUDRATE_BAUDRATE_Baud1M

/**
 * Function to initialize the parameters of UART based on the configurations in @ref board.h
 */
void uart_init(void);

#endif /* NRF52_UART_H */
/**
 * @}
 * @}
 */
