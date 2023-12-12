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
 * @file ch_com.c
 * @version $Id: ch_com.c $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2022 TDK Corporation All rights reserved.
 *
 * @brief The ultrasonic sensor system with I2C communication.
 * 
 **/

//#include "ch_types.h"
#include "constant.h"
#include "usr_i2c.h"
#include "usr_def.h"
#include "ch_com.h"
/*
 * External variable declaration
 */
/* none */

/*
 * Prototype declaration
 */
/* none */


/*=========================================================================================
 * \brief	Configurate I2C speed
 * \param	speed	I2C speed
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t COM_Config(uint64_t speed)
{
    int8_t ret = RET_NG;

	ret = USR_I2C_Config(speed);

    return ret;
}

/*=========================================================================================
 * \brief	Write to a sensor programming register.
 * \param	reg_addr 	sensor programming register address.
 * 			data 		data to transmit.
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t COM_WriteRegProg(uint8_t reg_addr, uint16_t data)
{
	uint8_t message[3];
	int8_t ret = RET_NG;

	/* Set register address write bit */
	reg_addr |= (uint8_t)0x80;
	
	/* Write the register address, followed by the value to be written */
	message[0] = reg_addr;
	message[1] = (uint8_t)data;
	message[2] = (uint8_t)(data >> 8);

	/* For the 2-byte registers, we also need to also write MSB after the LSB */
	if ((reg_addr & 0x40u) != 0u) {
		ret = USR_I2C_Write(CHIRP_I2C_ADDR_PROG, message, 2);	/* 8-bit data */
	} else {
		ret = USR_I2C_Write(CHIRP_I2C_ADDR_PROG, message, 3);	/* 16-bit data */
	}

	return ret;
}

/*=========================================================================================
 * \brief	Write to a sensor application register.
 * \param	i2c_addr	application i2c address
 * 			reg_addr	sensor register address
 * 			data		data to transmit
 * 			nbytes		number of bytes to write
 * \return 	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t COM_WriteRegApp(uint8_t i2c_addr, uint8_t reg_addr, uint16_t data, uint16_t nbytes)
{
	uint8_t message[4];
	int8_t ret = RET_NG;
	uint16_t len = nbytes;

	if (((int16_t)nbytes == 1) ||( (int16_t)nbytes == 2)) {
		// Place byte count (2) in first byte of message
		// Sensor is little-endian, so LSB goes in at the lower address
		message[0] = reg_addr;
		message[1] = (uint8_t)nbytes;
		message[2] = (uint8_t)data;
		message[3] = (uint8_t)(data >> 8);
		len = len+(uint16_t)2;
		ret = USR_I2C_Write(i2c_addr, message, len );
	}

	return ret;
}

/*=========================================================================================
 * \brief	Write 1 or 2 bytes of data to sensor memory.
 * \param	mem_addr	memory address of the sensor
 * 			data		data to transmit
 * 			nbytes		number of bytes to write
 * \return 	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t COM_WriteMem(uint16_t mem_addr, uint16_t data, uint16_t nbytes)
{
	uint16_t opcode;
	int8_t ret = RET_NG;

	if (((int16_t)nbytes == 1) || ((int16_t)nbytes == 2)) {
		ret = COM_WriteRegProg(CH_PROG_REG_ADDR, mem_addr);

		if (ret == RET_OK) {
			ret = COM_WriteRegProg(CH_PROG_REG_DATA, data);
		}

		if ((int16_t)nbytes == 1) {
			opcode = 0x0B;
		} else {
			opcode = 0x03;
		}
		if (ret == RET_OK) {
			ret = COM_WriteRegProg(CH_PROG_REG_CTL, opcode);
		}
	}
	
	return ret;
}

/*=========================================================================================
 * \brief	Write burst data to sensor memory.
 * \param	mem_addr	memory address of the sensor
 * 			buffer		pointer to data to transmit
 * 			nbytes		number of bytes to write
 * \return 	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t COM_WriteMemBurst(uint16_t mem_addr, const uint8_t* buffer, uint16_t nbytes)
{
	uint8_t burst_hdr[2] = {0xC4, 0x0B};
	int8_t ret = RET_NG;

	ret = COM_WriteRegProg(CH_PROG_REG_ADDR, mem_addr);

	if (ret == RET_OK) {
		ret = COM_WriteRegProg(CH_PROG_REG_CNT, (nbytes - (uint16_t)1));
	}
	if (ret == RET_OK) {
		ret = USR_I2C_Write(CHIRP_I2C_ADDR_PROG, burst_hdr, sizeof(burst_hdr));
	}
	if (ret == RET_OK) {
		ret = USR_I2C_Write(CHIRP_I2C_ADDR_PROG, buffer, nbytes);
	}

	return ret;
}

/*=========================================================================================
 * \brief	Read from a sensor application register.
 * \param	i2c_addr	sensor i2c address
 * 			reg_addr	sensor register address
 * 			read_val	read data value 
 * 			nbytes		number of bytes to read
 * \return 	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t COM_ReadReg(uint8_t i2c_addr, uint8_t reg_addr, uint16_t *read_val, uint16_t nbytes)
{
	uint8_t wr_buf;
	uint8_t rd_buf[2];
	int8_t ret = RET_NG;

	if (((int16_t)nbytes == 1) || ((int16_t)nbytes == 2)) {
		wr_buf = reg_addr;
		ret = USR_I2C_Read(i2c_addr, &wr_buf, 1, rd_buf, nbytes);

		if (ret == RET_OK){
			if (nbytes == (uint16_t)1) {
				*read_val = (uint16_t)rd_buf[0];
			} else {
				*read_val = (uint16_t)rd_buf[0] | ((uint16_t)rd_buf[1] << 8);
			}
		}
	}

	return ret;
}

#ifdef _NO_USE_ // no use
/*=========================================================================================
 * \brief	Read burst data from sensor memory.
 * \param	mem_addr	memory address of the sensor
 * 			read_val	read data value 
 * 			nbytes		number of bytes to read
 * \return 	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t COM_ReadMemBurst(uint16_t mem_addr, uint8_t *read_val, uint16_t nbytes)
{
	uint8_t message[2] = { ((uint8_t)0x80 | (uint8_t)CH_PROG_REG_CTL), (uint8_t)0x09 };		/* read burst command */
	uint16_t num_transfers;
	uint16_t bytes_left;
	uint16_t bytes_to_read;
	uint16_t xfer;
	int8_t ret = RET_NG;

	num_transfers = (nbytes + (uint16_t)(CH_PROG_XFER_SIZE - 1)) / (uint16_t)CH_PROG_XFER_SIZE;
	bytes_left = nbytes;
	
	for(xfer = 0; xfer < num_transfers; xfer++) {
		if (bytes_left > (uint16_t)CH_PROG_XFER_SIZE) {
			bytes_to_read = (uint16_t)CH_PROG_XFER_SIZE;
			bytes_left -= (uint16_t)CH_PROG_XFER_SIZE;
		} else {
			bytes_to_read = bytes_left;
			bytes_left = 0;
		}

		ret = COM_WriteRegProg(CH_PROG_REG_ADDR, (mem_addr + (xfer * (uint16_t)CH_PROG_XFER_SIZE)) );

		if (ret == RET_OK) {
			ret = COM_WriteRegProg(CH_PROG_REG_CNT, (bytes_to_read - (uint16_t)1) );
		}
		if (ret == RET_OK) {
			ret = USR_I2C_Write(CHIRP_I2C_ADDR_PROG, message, sizeof(message) );
		}
		if (ret == RET_OK) {
			ret = USR_I2C_Read(CHIRP_I2C_ADDR_PROG, NULL, 0, (read_val + (xfer * (uint16_t)CH_PROG_XFER_SIZE)), bytes_to_read);
		}
		
		if (ret == RET_NG) {
			return ret;
		}
	}

	return ret;
}
#endif
