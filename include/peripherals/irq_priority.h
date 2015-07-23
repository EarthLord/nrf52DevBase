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
 * @defgroup irq-priority Interrupt Priority
 * Interrupt priority definitions for all the peripherals.
 * Having all the definitions at one place provides a single glance understanding
 * of the priority levels used by various peripherals
 * @{
 *
 * @file
 * This file contains the Interrupt priority definitions for all the peripherals
 *
 * @author
 * Prithvi
 */

#ifndef IRQ_PRIORITY_H_
#define IRQ_PRIORITY_H_

/** @anchor irq-priority-defs
 * @name Definitions of the interrupt priority of all the peripherals
 * If a peripheral does not use interrupts, its priority is defined without a value.
 * Used in @ref uart-driver, @ref rtc-timer and @ref hf-timer module
 * @{*/

#define PRIORITY_POWER_CLOCK_IRQn
#define PRIORITY_RADIO_IRQn									1
#define PRIORITY_UARTE0_UART0_IRQn							2
#define PRIORITY_SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn
#define PRIORITY_SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn
#define PRIORITY_NFCT_IRQn
#define PRIORITY_GPIOTE_IRQn
#define PRIORITY_SAADC_IRQn
#define PRIORITY_TIMER0_IRQn
#define PRIORITY_TIMER1_IRQn								4
#define PRIORITY_TIMER2_IRQn
#define PRIORITY_RTC0_IRQn
#define PRIORITY_TEMP_IRQn
#define PRIORITY_RNG_IRQn									3
#define PRIORITY_ECB_IRQn
#define PRIORITY_CCM_AAR_IRQn
#define PRIORITY_WDT_IRQn
#define PRIORITY_RTC1_IRQn									4
#define PRIORITY_QDEC_IRQn
#define PRIORITY_COMP_LPCOMP_IRQn
#define PRIORITY_SWI0_EGU0_IRQn
#define PRIORITY_SWI1_EGU1_IRQn
#define PRIORITY_SWI2_EGU2_IRQn
#define PRIORITY_SWI3_EGU3_IRQn
#define PRIORITY_SWI4_EGU4_IRQn
#define PRIORITY_SWI5_EGU5_IRQn
#define PRIORITY_TIMER3_IRQn
#define PRIORITY_TIMER4_IRQn
#define PRIORITY_PWM0_IRQn
#define PRIORITY_PDM_IRQn
#define PRIORITY_MWU_IRQn
#define PRIORITY_PWM1_IRQn
#define PRIORITY_PWM2_IRQn
#define PRIORITY_SPIM2_SPIS2_SPI2_IRQn
#define PRIORITY_RTC2_IRQn
#define PRIORITY_I2S_IRQn

/** @} */

#endif /* IRQ_PRIORITY_H_ */
/**
 * @}
 * @}
 */
