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
 * @file ch_drv.h
 * @version $Id: ch_drv.h $ $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2021 TDK Corporation All rights reserved.
 *
 * @brief Module to control the ultrasonic sensor system
 *        External structure and macro definition.
 */

#ifndef _CH_DRV_CH101_H_
#define _CH_DRV_CH101_H_

#include "ch_types.h"
#include "constant.h"
/*
 * Structure definition
 */
/* none */


/*
 * External public function prototype declaration
 */

int8_t DRV_FWLoad_CH101(void);
int8_t DRV_WaitForLock_CH101(uint16_t timeout_ms);
int8_t DRV_MeasRTC_CH101(void);
int8_t DRV_SetMode_CH101(uint8_t mode);
int8_t DRV_SetMaxRange_CH101(uint16_t range);
int8_t DRV_SetStaticRange_CH101(uint16_t samples);
int8_t DRV_SetSampleInterval_CH101(uint16_t interval);
uint32_t DRV_GetRange_CH101(void);
uint16_t DRV_GetAmplitude_CH101(void);

void DRV_GenPulseUS_CH101(uint16_t pulse_width);



#endif /* _CH_DRV_CH101_H_ */
