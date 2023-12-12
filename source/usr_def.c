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
 * @file usr_def.c
 * @version $Id: usr_def.c $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2022 TDK Corporation All rights reserved.
 *
 * @brief  User Defined Ultrasonic Sensor System
 * 
 **/

#include "pico/binary_info.h"

#include "usr_def.h"
#include "constant.h"

/*
 * External variable declaration
 */
/* Setting information for each sensor */
/* Note: If multiple sensors are connected to a level shifter,
   the same DIR pin definition is acceptable. */
                       //mode             intval	range	reso	addr	prog	int		dir		rsv1	rsv2
chirp_dev_t gChirpDev101 = {CH_MODE_FREERUN,     500,	750,	1,	   0x30,	1,		2,		3,		0,		0};

chirp_dev_t gChirpDev201 = {CH_MODE_FREERUN,     500,	1500,	1,	   0x30,	1,		2,		3,		0,		0};

chirp_dev_t gChirpDev;

/*
 * Prototype declaration
 */
/* none */


