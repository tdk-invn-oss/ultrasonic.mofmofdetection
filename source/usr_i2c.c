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
 * @file usr_i2c.c
 * @version $Id: usr_i2c.c $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2022 TDK Corporation All rights reserved.
 *
 * @brief  BSP interface of I2C communication for RP2040(Raspberry Pi Pico)
 * 
 **/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "constant.h"

/*
 * Defines
 */
#define I2C_PORT             i2c1
#define I2C_TIME_OUT        (1000 * 1000)	// msec

#define PIN_SDA     (6)
#define PIN_SCL     (7)

/*
 * External variable declaration
 */
/* none */

/*
 * Prototype declaration
 */
/* none */


/*=========================================================================================
 * \brief	Configurate I2C communication
 * \param	speed	I2C speed
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t USR_I2C_Config(uint64_t speed)
{
    static int8_t set_flag = 0;
    uint tmp;

    if ( set_flag != 0 ) {
        i2c_deinit(I2C_PORT);
    }

    tmp = i2c_init(I2C_PORT, (uint)speed);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PIN_SDA, PIN_SCL, GPIO_FUNC_I2C));

    set_flag = 1;

    return RET_OK;
}

/*=========================================================================================
 * \brief	Writing process of I2C communication
 * \param	i2c_addr	i2c address
 * 			buffer		pointer to data to write
 * 			len			data length to write
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t USR_I2C_Write(uint8_t i2c_addr, const uint8_t *buffer, uint16_t len)
{
	int16_t num;

	num = (int16_t)i2c_write_timeout_us(I2C_PORT, i2c_addr, buffer, len, false, I2C_TIME_OUT);
	if (num != (int16_t)len) {
		return RET_NG;
	}

	return RET_OK;
}

/*=========================================================================================
 * \brief	Reading process of I2C communication
 * \param	i2c_addr	i2c address
 * 			wr_buffer		pointer to data to write
 * 			wr_len			data length to write
 * 			rd_buffer		pointer to data to read
 * 			rd_len			data length to read
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t USR_I2C_Read(uint8_t i2c_addr, uint8_t *wr_buffer, uint16_t wr_len, uint8_t *rd_buffer, uint16_t rd_len)
{
	int16_t num;

	if ((int8_t)wr_len!=0) {
		/* i2c_write_blocking(): 5th parameter = true: Do not send 'stop condition' */
		num = (int16_t)i2c_write_blocking(I2C_PORT, i2c_addr, wr_buffer, wr_len, true);
		if (num != (int16_t)wr_len){
			return RET_NG;
		}
	}

	num = (int16_t)i2c_read_blocking(I2C_PORT, i2c_addr, rd_buffer, rd_len, false);
	if (num != (int16_t)rd_len){
		return RET_NG;
	}

	return RET_OK;
}
