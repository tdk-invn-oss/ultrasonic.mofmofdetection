/*
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/**
 * @file usr_bsp.h
 * @version $Id: usr_bsp.h $ $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2021 TDK Corporation All rights reserved.
 *
 * @brief BSP interface for RP2040(Raspberry Pi Pico)
 *        External structure and macro definition.
 */

#ifndef _USR_BSP_H_
#define _USR_BSP_H_

#include "ch_types.h"

/*
 * Defines
 */


/*
 * Structure definition
 */
/* none */


/*
 * External public function prototype declaration
 */
void USR_TimerEnable(uint16_t sample_interval);
void USR_TimerDisable(void);
uint8_t USR_TimerCallbackOccurred(void);

void USR_IntrEnable(uint8_t pin);
void USR_IntrDisable(uint8_t pin);
uint8_t USR_IntrCallbackOccured(void);
void USR_IntrCallbackRefresh(void);

void USR_GPIO_Init(void);
void USR_GPIO_Set(uint8_t pin, uint8_t level);
uint8_t USR_GPIO_Get(uint8_t pin);
void USR_GPIO_SetDir(uint8_t pin, uint8_t dir);
void USR_GPIO_SetPullUp(uint8_t pin);
void USR_GPIO_SetPullDown(uint8_t pin);

void USR_WaitMS(uint16_t t_ms);
void USR_WaitUS(uint16_t t_us);


#endif /* _USR_BSP_H_ */
