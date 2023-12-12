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
 * @file ch101_gpr.h
 * @version $Id: ch101_gpr.h $ $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2021 TDK Corporation All rights reserved.
 *
 * @brief  External structure and macro definition.
 */

#ifndef _CH101_GPR_H_
#define _CH101_GPR_H_

#include "constant.h"

/*
 * Defines
 */
#define CH101_RAM_INIT_ADDRESS			(2420)
#define CH101_RAM_INIT_WRITE_SIZE		(13)

extern const unsigned char ram_ch101_gpr_init[CH101_RAM_INIT_WRITE_SIZE];
extern const unsigned char ch101_gpr_fw[CH101_FW_SIZE];
#endif /* _CH101_GPR_H_ */
