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
 * @file ch_com.h
 * @version $Id: ch_com.h $ $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2021 TDK Corporation All rights reserved.
 *
 * @brief The ultrasonic sensor system with I2C communication.
 *        External structure and macro definition.
 */

#ifndef _CH_COM_H_
#define _CH_COM_H_

#include "ch_types.h"

/*
 * Structure definition
 */
/* none */


/*
 * External public function prototype declaration
 */
int8_t COM_Config(uint64_t speed);
int8_t COM_WriteRegProg(uint8_t reg_addr, uint16_t data);
int8_t COM_WriteRegApp(uint8_t i2c_addr, uint8_t reg_addr, uint16_t data, uint16_t nbytes);
int8_t COM_WriteMem(uint16_t mem_addr, uint16_t data, uint16_t nbytes);
int8_t COM_WriteMemBurst(uint16_t mem_addr, const uint8_t* buffer, uint16_t nbytes);
int8_t COM_ReadReg(uint8_t i2c_addr, uint8_t reg_addr, uint16_t *read_val, uint16_t nbytes);
int8_t COM_ReadMemBurst(uint16_t mem_addr, uint8_t *read_val, uint16_t nbytes);


#endif /* _CH_COM_H_ */
