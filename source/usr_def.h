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
 * @file usr_def.h
 * @version $Id: usr_def.h $ $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2021 TDK Corporation All rights reserved.
 *
 * @brief User Defined Ultrasonic Sensor System
 *        External structure and macro definition.
 */

#ifndef _USR_DEF_H_
#define _USR_DEF_H_

#include <stdio.h>
#include "ch_types.h"
#include <stdbool.h>

/*
 * Defines
 */
/* System parameter */
#define NUM_SENSORS				(1)			// number of sensors on the system
#define CHIRP_I2C_SPEED			(100000)	// speed of i2c communication (100kHz)
#define CHIRP_I2C_ADDR_PROG		(0x45) 		/*!< I2C address of sensor programming interface. */

/* Host pin level that determines the direction of the level shifter */
#define DIR_HOST_TO_SENSOR		(1)
#define DIR_SENSOR_TO_HOST		(0)

/* Debug output definition */
#define _DEBUG_PRINT_

#ifdef _DEBUG_PRINT_
 #define DEBUG_PRINT_STR(str)		(void)printf(str);
 #define DEBUG_PRINTF(fmt,...)		(void)printf(fmt, ##__VA_ARGS__)
#else
 #define DEBUG_PRINT_STR(str)		
 #define DEBUG_PRINTF(fmt,...)		
#endif

/* Pin */
#define HIGH		(1)
#define LOW			(0)

#ifndef GPIO_OUT
 #define GPIO_OUT	(1)
#endif
#ifndef GPIO_IN
 #define GPIO_IN	(0)
#endif

#define RST_PIN		(0)		/* Reset pin of Chirp sensor */

/*
 * Structure definition
 */
typedef struct {
	uint8_t     mode;
	uint16_t	interval;
	uint16_t	max_range;
	uint8_t		i2c_reso;
	uint8_t		i2c_addr;
	uint8_t		prog_pin;
	uint8_t		int_pin;
	uint8_t		dir_pin;
	uint8_t		reserved1;
	uint16_t	reserved2;
} chirp_dev_t;

/*
 * External public function prototype declaration
 */
/* none */

/*
 * External variable declaration
 */
extern chirp_dev_t gChirpDev101;
extern chirp_dev_t gChirpDev201;
extern chirp_dev_t gChirpDev;


#endif /* _USR_DEF_H_ */
