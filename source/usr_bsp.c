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
 * @file usr_bsp.c
 * @version $Id: usr_bsp.c $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2022 TDK Corporation All rights reserved.
 *
 * @brief  BSP interface for RP2040(Raspberry Pi Pico)
 * 
 **/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "class/cdc/cdc_device.h"

#include "constant.h"
#include "usr_def.h"

/*
 * External variable declaration
 */
struct repeating_timer sPicoTimer;
static uint8_t sTimerOccuredCount = 0;
static uint8_t sTimerAcceptCount = 0;
static uint8_t sIRQFlag = 0;

/*
 * Prototype declaration
 */
bool USR_TimerCallback(struct repeating_timer *t);
void USR_IntrCallback(uint gpio, uint32_t events);
#define UNUSED_VARIABLE(x) (void)(x)

/*=========================================================================================
 * \brief	Periodic timer callback routine
 * \param	t
 * \return	
============================================================================================*/
bool USR_TimerCallback(struct repeating_timer *t)
{
	UNUSED_VARIABLE(t);
	sTimerOccuredCount++;
	return(true);
}

/*=========================================================================================
 * \brief	Enable timer to start measurement
 * \param	sample_interval		measurement interval
 * \return	none
============================================================================================*/
void USR_TimerEnable(uint16_t sample_interval)
{
	(void)add_repeating_timer_ms((int32_t)sample_interval, USR_TimerCallback,  NULL, &sPicoTimer); 
}

/*=========================================================================================
 * \brief	Disable timer to stop measurement
 * \param	none
 * \return	none
============================================================================================*/
void USR_TimerDisable(void)
{
	(void)cancel_repeating_timer (&sPicoTimer);
}


/*=========================================================================================
 * \brief	Get timer status flag
 * \param	none
 * \return	timer finished or not
============================================================================================*/
uint8_t USR_TimerCallbackOccurred(void)
{
	if (sTimerAcceptCount != sTimerOccuredCount) {
		sTimerAcceptCount = sTimerOccuredCount;
		return RET_FINISH;
	}
	
	return RET_NOT_FINISH;
}

/*=========================================================================================
 * \brief	GPIO interrupts callback routine
 * \param	gpio	GPIO pin where the interrupt occurred
 *			events	events that occurred
 * \return	none
============================================================================================*/
void USR_IntrCallback(uint gpio, uint32_t events)
{
	UNUSED_VARIABLE(events);
	if (gpio == gChirpDev.int_pin) {
		sIRQFlag= 1;
	}
}

/*=========================================================================================
 * \brief	Enable GPIO interrupts
 * \param	pin		GPIO pin
 * \return	none
============================================================================================*/
void USR_IntrEnable(uint8_t pin)
{
	static uint8_t sPassed = 0;
	
	if ((int8_t)sPassed == 0) {
		sPassed = 1;
		gpio_set_irq_enabled_with_callback(pin, 0x8u, true, USR_IntrCallback);		/* posedge */
	} else {
	    gpio_set_irq_enabled(pin, 0x8u, true);		/* posedge */
	}
}

/*=========================================================================================
 * \brief	Disable GPIO interrupts
 * \param	pin		GPIO pin
 * \return	none
============================================================================================*/
void USR_IntrDisable(uint8_t pin)
{
    gpio_set_irq_enabled(pin, 0x8u, false);
}

/*=========================================================================================
 * \brief	Get interrupt status flag
 * \param	none
 * \return	status flag
============================================================================================*/
uint8_t USR_IntrCallbackOccured(void)
{
	uint8_t ret;

	ret = sIRQFlag;

	return ret;
}

/*=========================================================================================
 * \brief	Clear interrupt status flag
 * \param	none
 * \return	none
============================================================================================*/
void USR_IntrCallbackRefresh(void)
{
	/* Interrupt_disable */
	USR_IntrDisable(gChirpDev.int_pin);

	sIRQFlag = 0;

	/* Interrupt_enable */
	USR_IntrEnable(gChirpDev.int_pin);
}

/*=========================================================================================
 * \brief	Initialize  I/O pins
 * \param	none
 * \return	none
============================================================================================*/
void USR_GPIO_Init(void)
{
//	uint8_t i;
	
	gpio_init(RST_PIN);
	gpio_set_dir(RST_PIN, true/*GPIO_OUT*/);
	gpio_put(RST_PIN, false/*LOW*/);

	
	gpio_init(gChirpDev.prog_pin);
	gpio_set_dir(gChirpDev.prog_pin, true/*GPIO_OUT*/);
	gpio_put(gChirpDev.prog_pin, false/*LOW*/);

	gpio_init(gChirpDev.int_pin);
	gpio_set_dir(gChirpDev.int_pin, true/*GPIO_OUT*/);
	gpio_put(gChirpDev.int_pin, false/*LOW*/);

	gpio_init(gChirpDev.dir_pin);
	gpio_set_dir(gChirpDev.dir_pin, true/*GPIO_OUT*/);
	gpio_put(gChirpDev.dir_pin, true/*DIR_HOST_TO_SENSOR*/);
	
}

/*=========================================================================================
 * \brief	Set GPIO pin to High or Low
 * \param	pin		GPIO pin
 *			level	HIGH or LOW
 * \return	none
============================================================================================*/
void USR_GPIO_Set(uint8_t pin, uint8_t level)
{
	bool val=false;
	if (level!=0u)
	{
		val=true;
	}
	else
	{
		val=false;
	}
	gpio_put(pin, val);
}

/*=========================================================================================
 * \brief	Get the level of the GPIO pin
 * \param	pin		GPIO pin
 * \return	the level of the GPIO pin
============================================================================================*/
uint8_t USR_GPIO_Get(uint8_t pin)
{
	uint8_t ret=0;
	if ( gpio_get(pin) == true )
	{
		ret=1;
	}
	else
	{
		ret=0;
	}
	return ret;
}

/*=========================================================================================
 * \brief	Set the direction of GPIO pin
 * \param	pin		GPIO pin
 *			dir		IN or OUT
 * \return	none
============================================================================================*/
void USR_GPIO_SetDir(uint8_t pin, uint8_t dir)
{
	bool val=false;
	if (dir==(uint8_t)GPIO_IN)
	{
		val = false;
	}
	else
	{
		val = true;
	} 
	gpio_set_dir(pin, val);
}

/*=========================================================================================
 * \brief	Set pull up of GPIO pin
 * \param	pin		GPIO pin
 * \return	none
============================================================================================*/
void USR_GPIO_SetPullUp(uint8_t pin)
{
	gpio_pull_up(pin);
}

/*=========================================================================================
 * \brief	Set pull down of GPIO pin
 * \param	pin		GPIO pin
 * \return	none
============================================================================================*/
void USR_GPIO_SetPullDown(uint8_t pin)
{
	gpio_pull_down(pin);
}

/*=========================================================================================
 * \brief	Wait time
 * \param	time	wait time(msec)
 * \return	none
============================================================================================*/
void USR_WaitMS(uint16_t t_ms)
{
	busy_wait_ms(t_ms);
}

/*=========================================================================================
 * \brief	Wait time
 * \param	time	wait time(usec)
 * \return	none
============================================================================================*/
void USR_WaitUS(uint16_t t_us)
{
	busy_wait_us(t_us);
}
