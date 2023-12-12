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
 * @file usr_i2c.h
 * @version $Id: usr_i2c.h $ $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2021 TDK Corporation All rights reserved.
 *
 * @brief BSP interface of I2C communication for RP2040(Raspberry Pi Pico).
 *        External structure and macro definition.
 */

#ifndef _USR_I2C_H_
#define _USR_I2C_H_

#include "ch_types.h"

/*
 * Structure definition
 */
/* none */

/*
 * External public function prototype declaration
 */
int8_t USR_I2C_Config(uint64_t speed);
int8_t USR_I2C_Write(uint8_t i2c_addr, const uint8_t *buffer, uint16_t len);
int8_t USR_I2C_Read(uint8_t i2c_addr, uint8_t *wr_buffer, uint16_t wr_len, uint8_t *rd_buffer, uint16_t rd_len);


#endif /* _USR_I2C_H_ */
