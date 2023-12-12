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
 * @file ch_lib.h
 * @version $Id: ch_lib.h $ $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2021 TDK Corporation All rights reserved.
 *
 * @brief API of the ultrasonic sensor system
 *        External structure and macro definition.
 */

#ifndef _CH_LIB_H_
#define _CH_LIB_H_

#include "ch_types.h"

/*
 * Structure definition
 */
/* none */


/*
 * External public function prototype declaration
 */

int8_t CH_API_Init(uint16_t type);
int8_t CH_API_Config(uint8_t mode, uint16_t range, uint16_t interval);
void CH_API_MeasStart(void);
void CH_API_MeasStop(void);
int8_t CH_API_GetResult(chirp_result_t *p_result);


#endif /* _CH_LIB_H_ */
