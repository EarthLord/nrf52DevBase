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
 * @defgroup board-config Board Configuration
 * Configuration for different boards
 * @{
 *
 * @file
 * This file contains all the board specific configurations, PCA10036 in this case.
 * PCA10036's LED and UART pin configurations are present.
 *
 * @author
 * Prithvi
 */

#ifndef BOARD_H
#define BOARD_H

 
#ifdef BOARD_PCA10036
/** @anchor led-pin-definitions
 * @name Definitions for the LEDs pins in PCA10036
 * Used in @ref led-driver module
 * @{*/
/** LED1 pin number */
#define LED1  17
/** LED2 pin number */
#define LED2  18
/** LED3 pin number */
#define LED3  19
/** LED4 pin number */
#define LED4  20

/** @} */

/** @anchor button-pin-definitions
 * @name Definitions for the buttons pins in PCA10036
 * @{*/
/** BUTTON1 pin number */
#define BUTTON1  13
/** BUTTON2 pin number */
#define BUTTON2  14
/** BUTTON3 pin number */
#define BUTTON3  15
/** BUTTON4 pin number */
#define BUTTON4  16

/** @} */

/** @anchor uart-pin-definitions
 * @name Definitions for the UART pins in PCA10036
 * Used in @ref uart-driver module
 * @{*/
/** Rx pin number */
#define RX_PIN_NUMBER  8
/** Tx pin number */
#define TX_PIN_NUMBER  6
/** @brief CTS pin number.
 * Used if @ref HWFC is true */
#define CTS_PIN_NUMBER 7
/** @brief RTS pin number.
 * Used if @ref HWFC is true */
#define RTS_PIN_NUMBER 5
/** @brief Specify with a boolean value if Hardware Flow Control is required.
 *  PCA1000 cannot work without HWFC */
#define HWFC           true
/** @}
 */

#elif ANOTHER_BOARD
/*	Define board specific pins and configurations here */
#endif  /* BOARD_PCA10036 */

#endif
/** @}
 */
