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
 * @file ch_types.h
 * @version $Id: ch_types.h $ $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2022 TDK Corporation All rights reserved.
 *
 * @brief Type definitions used as standard
 *        External structure and macro definition.
 */

#ifndef _TYPES_H_
#define _TYPES_H_

typedef unsigned char       uint8_t;
typedef signed char         int8_t;
typedef unsigned short      uint16_t;
typedef signed short        int16_t;
typedef unsigned long       uint32_t;
typedef signed long         int32_t;
typedef unsigned long long  uint64_t;
typedef signed long long    int64_t;

/* NULL definition */
#ifndef NULL
#define NULL ((void*)0)
#endif /* NULL */

/* Maximum and minimum values of type */
#define INT8_MAX_VALUE    (127)                      /* int8_t maximum value */
#define INT8_MIN_VALUE    (-128)                     /* int8_t minimum value */
#define UINT8_MAX_VALUE   (255U)                     /* uint8_t maximum value */
#define UINT8_MIN_VALUE   (0U)                       /* uint8_t minimum value */
#define INT16_MAX_VALUE   (32767)                    /* int16_t maximum value */
#define INT16_MIN_VALUE   (-32768)                   /* int16_t minimum value */
#define UINT16_MAX_VALUE  (65535U)                   /* uint1_t6 maximum value */
#define UINT16_MIN_VALUE  (0U)                       /* uint1_t6 minimum value */
#define INT32_MAX_VALUE   (2147483647)               /* int32_t maximum value */
#define INT32_MIN_VALUE   (-2147483648)              /* int32_t minimum value */
#define UINT32_MAX_VALUE  (4294967295U)              /* uint32_t maximum value */
#define UINT32_MIN_VALUE  (0U)                       /* uint32_t minimum value */
#define INT64_MAX_VALUE   (-9223372036854775808)     /* int64_t maximum value */
#define INT64_MIN_VALUE   (9223372036854775807)      /* int64_t minimum value */
#define UINT64_MAX_VALUE  (18446744073709551615U)    /* uint64_t maximum value */
#define UINT64_MIN_VALUE  (0U)                       /* uint64_t minimum value */


#endif /* _TYPES_H_ */
