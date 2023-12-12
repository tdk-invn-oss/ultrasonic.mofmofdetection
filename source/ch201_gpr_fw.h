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
 * @file ch201_gpr_fw.h
 * @version $Id: ch201_gpr_fw.h $ $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2021-2023 TDK Corporation All rights reserved.
 *
 * @brief Firmware of the ultrasonic sensor system
 *        External structure and macro definition.
 *        Chirp Microsystems Firmware Header Generator v2.0 (Python 2.7.15)
 *        File generated from ch201_gprmt_v10a.hex at 2020-11-13 18:19:55.828000 by klong
 */

#ifndef _CH201_GPR_FW_H_
#define _CH201_GPR_FW_H_

#include "ch_types.h"

/*
 * Defines
 */
#define CH201_RAM_INIT_ADDRESS			(2392)
#define CH201_RAM_INIT_WRITE_SIZE		(28)

const char * ch201_gprmt_version = "gprmt_gprmt-201_v10a";
const char * ch201_gprmt_gitsha1 = "247eb617b50e896de61a12488571555935b91867";

const unsigned char ram_ch201_gprmt_init[CH201_RAM_INIT_WRITE_SIZE] = {
0x88, 0x13, 0xD0, 0x07, 0x20, 0x03, 0x90, 0x01, 0xFA, 0x00, 0xAF, 0x00, 0x06, 0x00, 0x00, 0x00, 
0x00, 0xFA, 0x00, 0x00, 0x64, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x01, 0x00, };

const unsigned char ch201_gprmt_fw[] = {
0x0a, 0x12, 0x09, 0x12, 0x08, 0x12, 0x07, 0x12, 0x06, 0x12, 0x05, 0x12, 0x04, 0x12, 0x31, 0x80, 
0x0c, 0x00, 0x81, 0x4d, 0x0a, 0x00, 0x91, 0x42, 0x16, 0x02, 0x08, 0x00, 0x56, 0x42, 0x08, 0x02, 
0x05, 0x43, 0x81, 0x43, 0x02, 0x00, 0x09, 0x43, 0x0a, 0x43, 0x0c, 0x93, 0x6a, 0x24, 0x81, 0x4c, 
0x00, 0x00, 0x08, 0x43, 0x47, 0x43, 0x0f, 0x48, 0x1e, 0x41, 0x0a, 0x00, 0x4e, 0x93, 0x5c, 0x20, 
0x0a, 0x96, 0x1d, 0x20, 0x57, 0x53, 0x57, 0x93, 0x12, 0x24, 0x67, 0x93, 0x0d, 0x24, 0x77, 0x90, 
0x03, 0x00, 0x07, 0x24, 0x67, 0x92, 0x02, 0x24, 0x06, 0x43, 0x0c, 0x3c, 0x5e, 0x42, 0x15, 0x02, 
0x08, 0x3c, 0x5e, 0x42, 0x0d, 0x02, 0x05, 0x3c, 0x5e, 0x42, 0x0c, 0x02, 0x02, 0x3c, 0x5e, 0x42, 
0x09, 0x02, 0x06, 0x5e, 0x4e, 0x47, 0x0e, 0x5e, 0x91, 0x4e, 0x16, 0x02, 0x08, 0x00, 0x3e, 0x40, 
0x28, 0x02, 0x04, 0x4f, 0x14, 0x53, 0x04, 0x54, 0x04, 0x5e, 0x0f, 0x5f, 0x0f, 0x5e, 0x0b, 0x4f, 
0x2c, 0x4f, 0x2d, 0x44, 0xb0, 0x12, 0xec, 0xfe, 0x3a, 0x90, 0x09, 0x00, 0x0c, 0x2c, 0x0f, 0x4a, 
0x0f, 0x5f, 0x3f, 0x50, 0x30, 0x09, 0x2e, 0x4f, 0x8f, 0x4c, 0x00, 0x00, 0x0c, 0x8e, 0x0c, 0x93, 
0x02, 0x34, 0x3c, 0xe3, 0x1c, 0x53, 0x81, 0x93, 0x02, 0x00, 0x1e, 0x20, 0x5f, 0x42, 0x11, 0x02, 
0x0f, 0x5f, 0x0f, 0x9a, 0x19, 0x2c, 0x05, 0x93, 0x06, 0x20, 0x81, 0x9c, 0x08, 0x00, 0x14, 0x2c, 
0x81, 0x4a, 0x06, 0x00, 0x15, 0x43, 0x2c, 0x4b, 0x2d, 0x44, 0xb0, 0x12, 0xdc, 0xfd, 0x0c, 0x99, 
0x02, 0x28, 0x09, 0x4c, 0x09, 0x3c, 0x0f, 0x4a, 0x1f, 0x81, 0x06, 0x00, 0x1f, 0x83, 0x81, 0x4f, 
0x04, 0x00, 0x05, 0x43, 0x91, 0x43, 0x02, 0x00, 0x28, 0x53, 0x1a, 0x53, 0x91, 0x83, 0x00, 0x00, 
0x9a, 0x23, 0x05, 0x93, 0x06, 0x20, 0x81, 0x93, 0x02, 0x00, 0x07, 0x20, 0xb2, 0x43, 0x24, 0x02, 
0x40, 0x3c, 0x1a, 0x81, 0x06, 0x00, 0x81, 0x4a, 0x04, 0x00, 0x82, 0x49, 0x26, 0x02, 0x0c, 0x49, 
0x12, 0xc3, 0x09, 0x10, 0x16, 0x41, 0x04, 0x00, 0x1a, 0x41, 0x06, 0x00, 0x0a, 0x56, 0x36, 0x90, 
0xfd, 0xff, 0x1a, 0x38, 0x3f, 0x40, 0x28, 0x02, 0x07, 0x4a, 0x07, 0x57, 0x17, 0x53, 0x07, 0x57, 
0x07, 0x5f, 0x08, 0x4a, 0x08, 0x58, 0x08, 0x58, 0x08, 0x5f, 0x26, 0x42, 0x16, 0x51, 0x04, 0x00, 
0x05, 0x4c, 0x2c, 0x48, 0x2d, 0x47, 0xb0, 0x12, 0xdc, 0xfd, 0x0c, 0x99, 0x05, 0x28, 0x27, 0x82, 
0x28, 0x82, 0x1a, 0x83, 0x16, 0x83, 0xf4, 0x23, 0x09, 0x8c, 0x09, 0x59, 0x05, 0x8c, 0x3e, 0x42, 
0x4f, 0x43, 0x4f, 0x5f, 0x05, 0x99, 0x02, 0x2c, 0x09, 0x85, 0x5f, 0x53, 0x09, 0x59, 0x1e, 0x83, 
0xf8, 0x23, 0x0c, 0x4a, 0xb0, 0x12, 0xdc, 0xfe, 0x4f, 0x4f, 0x0f, 0x11, 0x0c, 0xdf, 0x82, 0x4c, 
0x24, 0x02, 0x31, 0x50, 0x0c, 0x00, 0x30, 0x40, 0x6c, 0xff, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 
0x0c, 0x12, 0x0b, 0x12, 0x0a, 0x12, 0x3a, 0x40, 0x77, 0x01, 0x82, 0x4a, 0xa6, 0x01, 0xd2, 0xc3, 
0x6a, 0x09, 0xc2, 0x93, 0x14, 0x02, 0x3a, 0x20, 0x1b, 0x43, 0x1c, 0x42, 0x3a, 0x02, 0x1d, 0x42, 
0x38, 0x02, 0xb0, 0x12, 0xec, 0xfe, 0x1c, 0x92, 0x6e, 0x09, 0x19, 0x28, 0x1f, 0x42, 0x3a, 0x02, 
0x0f, 0x11, 0x1f, 0x82, 0x38, 0x02, 0x1f, 0x93, 0x02, 0x38, 0x3f, 0x43, 0x01, 0x3c, 0x1f, 0x43, 
0xc2, 0x93, 0x70, 0x09, 0x07, 0x24, 0x5e, 0x42, 0x70, 0x09, 0x8e, 0x11, 0x0f, 0x9e, 0x02, 0x24, 
0x0b, 0x43, 0x02, 0x3c, 0x82, 0x5f, 0x6c, 0x09, 0xc2, 0x4f, 0x70, 0x09, 0x0f, 0x3c, 0xb2, 0x50, 
0x14, 0x00, 0x6c, 0x09, 0xb2, 0x90, 0x2d, 0x01, 0x6c, 0x09, 0x06, 0x28, 0xb2, 0x80, 0xc8, 0x00, 
0x6c, 0x09, 0x12, 0xc3, 0x12, 0x10, 0x6e, 0x09, 0xc2, 0x43, 0x70, 0x09, 0x0b, 0x93, 0x3c, 0x20, 
0xd2, 0x43, 0x14, 0x02, 0xb2, 0x40, 0x1e, 0x3f, 0x50, 0x09, 0x36, 0x3c, 0xd2, 0x93, 0x14, 0x02, 
0x31, 0x20, 0xf2, 0x90, 0x03, 0x00, 0x56, 0x09, 0x0a, 0x24, 0xc2, 0x93, 0x56, 0x09, 0x04, 0x20, 
0xb2, 0x40, 0x58, 0x18, 0x42, 0x09, 0x15, 0x3c, 0xd2, 0x83, 0x56, 0x09, 0x12, 0x3c, 0x1c, 0x42, 
0x3a, 0x02, 0x1d, 0x42, 0x38, 0x02, 0xb0, 0x12, 0xec, 0xfe, 0x82, 0x9c, 0x68, 0x09, 0x05, 0x28, 
0x82, 0x4c, 0x68, 0x09, 0x92, 0x53, 0x64, 0x09, 0x04, 0x3c, 0xe2, 0x43, 0x56, 0x09, 0x92, 0x83, 
0x64, 0x09, 0xe2, 0x93, 0x56, 0x09, 0x0b, 0x24, 0xc2, 0x93, 0x56, 0x09, 0x0d, 0x20, 0xe2, 0x43, 
0x14, 0x02, 0xe2, 0xd3, 0x6a, 0x09, 0xb2, 0x40, 0x80, 0x10, 0xd0, 0x01, 0x05, 0x3c, 0xd2, 0x43, 
0x01, 0x02, 0x02, 0x3c, 0x82, 0x43, 0xf0, 0x01, 0xf2, 0x90, 0x03, 0x00, 0x56, 0x09, 0x07, 0x2c, 
0x5c, 0x42, 0x07, 0x02, 0x0c, 0x5c, 0x5d, 0x42, 0x56, 0x09, 0xb0, 0x12, 0x00, 0xf8, 0xe2, 0x93, 
0x14, 0x02, 0x0d, 0x28, 0xd2, 0xd3, 0xe0, 0x01, 0xd2, 0xc3, 0xe0, 0x01, 0xb2, 0x40, 0x77, 0x06, 
0xa6, 0x01, 0x3c, 0x42, 0xb0, 0x12, 0x8a, 0xff, 0x82, 0x4a, 0xa6, 0x01, 0x05, 0x3c, 0x5c, 0x43, 
0xb0, 0x12, 0xfc, 0xfb, 0xa2, 0xc2, 0x92, 0x01, 0xa2, 0xd2, 0x92, 0x01, 0xd2, 0x42, 0x54, 0x09, 
0xe0, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x0c, 0x00, 0x3a, 0x41, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 
0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0a, 0x12, 0xb2, 0x40, 0x80, 0x5a, 0x20, 0x01, 0xe2, 0x42, 
0xe0, 0x01, 0xd2, 0x43, 0xe2, 0x01, 0xf2, 0x40, 0x40, 0x00, 0x01, 0x02, 0xf2, 0x40, 0x3c, 0x00, 
0x07, 0x02, 0xf2, 0x40, 0x06, 0x00, 0x04, 0x02, 0xf2, 0x40, 0x09, 0x00, 0x00, 0x02, 0xf2, 0x40, 
0x1a, 0x00, 0x08, 0x02, 0xf2, 0x40, 0x0d, 0x00, 0x09, 0x02, 0xf2, 0x40, 0x11, 0x00, 0x0c, 0x02, 
0xf2, 0x40, 0x17, 0x00, 0x0d, 0x02, 0xf2, 0x40, 0x28, 0x00, 0x15, 0x02, 0xf2, 0x40, 0x1e, 0x00, 
0x10, 0x02, 0x3f, 0x40, 0x16, 0x02, 0x3d, 0x40, 0x06, 0x00, 0x3e, 0x40, 0x58, 0x09, 0x2f, 0x53, 
0xbf, 0x4e, 0xfe, 0xff, 0x1d, 0x83, 0xfb, 0x23, 0xd2, 0x43, 0x05, 0x02, 0xc2, 0x43, 0x11, 0x02, 
0xb2, 0x40, 0x80, 0x00, 0x02, 0x02, 0xf2, 0x40, 0x03, 0x00, 0xc2, 0x01, 0xb2, 0x40, 0x00, 0x02, 
0xa6, 0x01, 0xb2, 0x40, 0x00, 0x06, 0xa6, 0x01, 0xb2, 0x40, 0x28, 0x02, 0xb0, 0x01, 0xb2, 0x40, 
0x12, 0x00, 0xb2, 0x01, 0xb2, 0x40, 0x77, 0x01, 0xa6, 0x01, 0xb2, 0x40, 0x80, 0x00, 0x90, 0x01, 
0xb2, 0x40, 0x07, 0x00, 0x92, 0x01, 0x0a, 0x43, 0x02, 0x3c, 0x32, 0xd0, 0x58, 0x00, 0x5f, 0x42, 
0x01, 0x02, 0x0a, 0x9f, 0x23, 0x24, 0x5a, 0x42, 0x01, 0x02, 0x0f, 0x4a, 0x3f, 0x80, 0x10, 0x00, 
0x1b, 0x24, 0x3f, 0x80, 0x10, 0x00, 0x18, 0x24, 0x3f, 0x80, 0x20, 0x00, 0x10, 0x20, 0xc2, 0x43, 
0x14, 0x02, 0xe2, 0x42, 0x56, 0x09, 0xb2, 0x40, 0x1e, 0x18, 0x50, 0x09, 0x1f, 0x42, 0x6c, 0x09, 
0x3f, 0x50, 0x00, 0x10, 0x82, 0x4f, 0xf0, 0x01, 0x5c, 0x43, 0xb0, 0x12, 0xfc, 0xfb, 0xe2, 0x42, 
0x54, 0x09, 0xe2, 0xc3, 0xe0, 0x01, 0x02, 0x3c, 0xe2, 0xd3, 0xe0, 0x01, 0x32, 0xc2, 0x03, 0x43, 
0xc2, 0x93, 0x6a, 0x09, 0xd2, 0x27, 0x32, 0xd0, 0x18, 0x00, 0xd1, 0x3f, 0xd2, 0xd3, 0x6a, 0x09, 
0x1f, 0x42, 0x6c, 0x09, 0x3f, 0x50, 0x00, 0x10, 0x82, 0x4f, 0xf0, 0x01, 0xf2, 0x90, 0x40, 0x00, 
0x01, 0x02, 0x49, 0x24, 0xd2, 0x92, 0x07, 0x02, 0x66, 0x09, 0x31, 0x24, 0xd2, 0x42, 0x07, 0x02, 
0x66, 0x09, 0x5f, 0x42, 0x04, 0x02, 0x0f, 0x5f, 0x3f, 0x80, 0x0b, 0x00, 0x5e, 0x42, 0x07, 0x02, 
0x0e, 0x5e, 0x0e, 0x8f, 0x3e, 0x80, 0x0b, 0x00, 0xc2, 0x93, 0x56, 0x09, 0x04, 0x20, 0xb2, 0x40, 
0x58, 0x18, 0x42, 0x09, 0x03, 0x3c, 0xb2, 0x40, 0x58, 0x24, 0x42, 0x09, 0x0f, 0x5f, 0x0f, 0x5f, 
0x0f, 0x5f, 0x3f, 0x50, 0x00, 0x2c, 0x82, 0x4f, 0x44, 0x09, 0x3b, 0x40, 0xf8, 0x4f, 0x3d, 0x40, 
0x46, 0x09, 0x6f, 0x43, 0x3e, 0xb0, 0x80, 0xff, 0x17, 0x20, 0x0e, 0x5e, 0x0e, 0x5e, 0x0e, 0x5e, 
0x3e, 0x50, 0x00, 0x4c, 0x8d, 0x4e, 0x00, 0x00, 0x5f, 0x53, 0xc2, 0x4f, 0x57, 0x09, 0x4c, 0x93, 
0x04, 0x20, 0xb2, 0x40, 0x82, 0x10, 0xa2, 0x01, 0x19, 0x3c, 0xb2, 0x40, 0x86, 0x10, 0xa2, 0x01, 
0x92, 0x42, 0x50, 0x09, 0xa0, 0x01, 0x12, 0x3c, 0x2d, 0x53, 0x8d, 0x4b, 0xfe, 0xff, 0x5f, 0x53, 
0x3e, 0x80, 0x7f, 0x00, 0xdf, 0x3f, 0xb2, 0x40, 0x40, 0x20, 0x42, 0x09, 0xd2, 0x43, 0x57, 0x09, 
0x92, 0x42, 0x50, 0x09, 0xa0, 0x01, 0xb2, 0x40, 0x86, 0x10, 0xa2, 0x01, 0x5f, 0x42, 0x57, 0x09, 
0x0f, 0x93, 0x06, 0x24, 0x3e, 0x40, 0x42, 0x09, 0xb2, 0x4e, 0xa4, 0x01, 0x1f, 0x83, 0xfc, 0x23, 
0xc2, 0x93, 0x14, 0x02, 0x03, 0x24, 0xb2, 0xd0, 0x18, 0x00, 0xa2, 0x01, 0x92, 0x43, 0xae, 0x01, 
0xa2, 0x43, 0xae, 0x01, 0x30, 0x41, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 
0x0a, 0x12, 0x1d, 0x42, 0x02, 0x02, 0x0a, 0x4d, 0xe2, 0x93, 0x01, 0x02, 0x1d, 0x20, 0xd2, 0x83, 
0x72, 0x09, 0x1a, 0x20, 0x5e, 0x42, 0x05, 0x02, 0xc2, 0x4e, 0x72, 0x09, 0x5c, 0x42, 0x07, 0x02, 
0x5f, 0x42, 0x07, 0x02, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x8c, 0x6e, 0x93, 0x08, 0x28, 
0x0a, 0x9f, 0x06, 0x2c, 0x0a, 0x5d, 0xd2, 0x83, 0x72, 0x09, 0xe2, 0x93, 0x72, 0x09, 0xf8, 0x2f, 
0x5c, 0x43, 0xb0, 0x12, 0xfc, 0xfb, 0x09, 0x3c, 0xb2, 0x40, 0x77, 0x06, 0xa6, 0x01, 0x3c, 0x42, 
0xb0, 0x12, 0x8a, 0xff, 0xb2, 0x40, 0x77, 0x01, 0xa6, 0x01, 0x82, 0x4a, 0x90, 0x01, 0xb1, 0xc0, 
0xf0, 0x00, 0x0c, 0x00, 0x3a, 0x41, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 
0x00, 0x13, 0x0f, 0x12, 0x5f, 0x42, 0x73, 0x09, 0x0f, 0x93, 0x15, 0x24, 0x1f, 0x83, 0x26, 0x24, 
0x1f, 0x83, 0x29, 0x20, 0xb2, 0x90, 0x22, 0x00, 0x52, 0x09, 0x07, 0x2c, 0x1f, 0x42, 0x52, 0x09, 
0xdf, 0x42, 0xc1, 0x01, 0x00, 0x02, 0x92, 0x53, 0x52, 0x09, 0xd2, 0x83, 0x55, 0x09, 0x1b, 0x20, 
0xc2, 0x43, 0x73, 0x09, 0x18, 0x3c, 0x5f, 0x42, 0xc1, 0x01, 0x82, 0x4f, 0x52, 0x09, 0xd2, 0x43, 
0x73, 0x09, 0xd2, 0x4f, 0x00, 0x02, 0xc0, 0x01, 0x3f, 0x90, 0x06, 0x00, 0x0c, 0x20, 0xf2, 0x40, 
0x24, 0x00, 0xe0, 0x01, 0xb2, 0x40, 0x03, 0x00, 0xd8, 0x01, 0x05, 0x3c, 0xd2, 0x42, 0xc1, 0x01, 
0x55, 0x09, 0xe2, 0x43, 0x73, 0x09, 0xf2, 0xd0, 0x10, 0x00, 0xc2, 0x01, 0xf2, 0xd0, 0x20, 0x00, 
0xc2, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x02, 0x00, 0x3f, 0x41, 0x00, 0x13, 0x0a, 0x12, 0x1d, 0x93, 
0x03, 0x34, 0x3d, 0xe3, 0x1d, 0x53, 0x02, 0x3c, 0x3c, 0xe3, 0x1c, 0x53, 0x0e, 0x4d, 0x0f, 0x4c, 
0x0e, 0x11, 0x0f, 0x11, 0x0b, 0x43, 0x0c, 0x4e, 0x0d, 0x4b, 0xb0, 0x12, 0x94, 0xfe, 0x0a, 0x4c, 
0x0c, 0x4f, 0x0d, 0x4b, 0xb0, 0x12, 0x94, 0xfe, 0x1f, 0x93, 0x03, 0x34, 0x0e, 0x8c, 0x0f, 0x5a, 
0x02, 0x3c, 0x0e, 0x5c, 0x0f, 0x8a, 0x1b, 0x53, 0x2b, 0x92, 0xed, 0x3b, 0x0c, 0x4e, 0x3a, 0x41, 
0x30, 0x41, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0xe2, 0xb3, 0xe0, 0x01, 
0x12, 0x24, 0xd2, 0x42, 0xe0, 0x01, 0x54, 0x09, 0xe2, 0xc3, 0xe0, 0x01, 0xa2, 0xc2, 0x92, 0x01, 
0x4c, 0x43, 0xf2, 0x90, 0x20, 0x00, 0x01, 0x02, 0x01, 0x24, 0x5c, 0x43, 0xb0, 0x12, 0xfc, 0xfb, 
0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 
0x00, 0x13, 0x0f, 0x12, 0xc2, 0x43, 0x73, 0x09, 0x92, 0x53, 0x52, 0x09, 0xb2, 0x90, 0x30, 0x07, 
0x52, 0x09, 0x03, 0x28, 0x82, 0x43, 0x52, 0x09, 0x05, 0x3c, 0x1f, 0x42, 0x52, 0x09, 0xd2, 0x4f, 
0x00, 0x02, 0xc0, 0x01, 0xf2, 0xd0, 0x20, 0x00, 0xc2, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x02, 0x00, 
0x3f, 0x41, 0x00, 0x13, 0x3d, 0xf0, 0x0f, 0x00, 0x3d, 0xe0, 0x0f, 0x00, 0x0d, 0x5d, 0x00, 0x5d, 
0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 
0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x30, 0x41, 
0x3d, 0xf0, 0x0f, 0x00, 0x3d, 0xe0, 0x0f, 0x00, 0x0d, 0x5d, 0x00, 0x5d, 0x0c, 0x5c, 0x0c, 0x5c, 
0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 
0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x30, 0x41, 0x1c, 0x93, 0x02, 0x34, 
0x3c, 0xe3, 0x1c, 0x53, 0x0f, 0x4c, 0x1d, 0x93, 0x02, 0x34, 0x3d, 0xe3, 0x1d, 0x53, 0x0c, 0x4d, 
0x0c, 0x9f, 0x03, 0x2c, 0x0e, 0x4c, 0x0c, 0x4f, 0x0f, 0x4e, 0x12, 0xc3, 0x0f, 0x10, 0x0f, 0x11, 
0x0c, 0x5f, 0x30, 0x41, 0x0f, 0x12, 0xb2, 0xf0, 0xef, 0xff, 0xa2, 0x01, 0x3f, 0x40, 0x00, 0x28, 
0x1f, 0x52, 0x64, 0x09, 0x82, 0x4f, 0xa0, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x02, 0x00, 0x3f, 0x41, 
0x00, 0x13, 0x92, 0x42, 0xda, 0x01, 0x0a, 0x02, 0x82, 0x43, 0xd8, 0x01, 0xe2, 0x42, 0xe0, 0x01, 
0xb1, 0xc0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x13, 0x31, 0x40, 0x00, 0x0a, 0xb0, 0x12, 0x98, 0xff, 
0x0c, 0x43, 0xb0, 0x12, 0xf6, 0xfa, 0xb0, 0x12, 0x9c, 0xff, 0xe2, 0xc3, 0x6a, 0x09, 0x92, 0x42, 
0xd2, 0x01, 0x22, 0x02, 0xb1, 0xc0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x13, 0x34, 0x41, 0x35, 0x41, 
0x36, 0x41, 0x37, 0x41, 0x38, 0x41, 0x39, 0x41, 0x3a, 0x41, 0x30, 0x41, 0xb2, 0x40, 0x77, 0x13, 
0xa6, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x13, 0x1c, 0x83, 0x03, 0x43, 0xfd, 0x23, 
0x30, 0x41, 0x32, 0xd0, 0x10, 0x00, 0xfd, 0x3f, 0x1c, 0x43, 0x30, 0x41, 0x03, 0x43, 0xff, 0x3f, 
0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x62, 0xfe, 0x62, 0xfd, 0xa0, 0xff, 0x32, 0xff, 0x22, 0xfe, 0x00, 0x00, 0x92, 0xff, 0x9a, 0xf9, 
0x14, 0xff, 0x7c, 0xff, 0x92, 0xff, 0x00, 0x00, 0x5a, 0xff, 0xe6, 0xfc, 0x92, 0xff, 0x48, 0xff, 
};

#endif /* _CH201_GPR_FW_H_ */
