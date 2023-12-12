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
 * @file constant.h
 * @version $Id: constant.h $ $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2021 TDK Corporation All rights reserved.
 *
 * @brief Common Definitions of the ultrasonic sensor system
 *        External structure and macro definition.
 */

#ifndef _CONSTANT_H_
#define _CONSTANT_H_

#include "ch_types.h"

/*
 * Defines
 */
/* common */
#define RET_OK             (0)
#define RET_NG             (-1)
//#define RET_NG_PARAM       (-2u)

#define RET_FINISH			(1)
#define RET_NOT_FINISH		(0)

/* sensor */
#define CHIRP_TYPE_CH101	(1u)
#define CHIRP_TYPE_CH201	(2u)

#define CH_SIG_BYTE_0		(0x0au)			/*!< Signature byte in sensor (1 of 2). */
#define CH_SIG_BYTE_1		(0x02u)			/*!< Signature byte in sensor (2 of 2). */
#define CH_SIG_WORD		    ( (((uint16_t)CH_SIG_BYTE_1) << 8 ) | ((uint16_t)CH_SIG_BYTE_0) )

#define CHDRV_FREQLOCK_TIMEOUT_MS 	(100u) 		/*!< Time to wait in chdrv_group_start() for sensor */
#define RTC_CAL_PULSE_MS            (100u)

#define CH_NUM_THRESHOLDS			(6u)				/*!< Number of internal detection thresholds (CH201 only). */
#define CH_SPEEDOFSOUND_MPS			(343u) 			/*!< Speed of sound, in meters per second. */

/* Misc definitions */
#define CH_NO_TARGET		(0xFFFFFFFFu)	/*!< Range value returned if no target was detected. */

#define CH101_DATA_MEM_SIZE			(0x800u)	
#define CH101_DATA_MEM_ADDR			(0x0200u)
#define CH101_PROG_MEM_SIZE			(0x800u)	
#define CH101_PROG_MEM_ADDR			(0xF800u)
#define	CH101_FW_SIZE				CH101_PROG_MEM_SIZE

#define CH201_DATA_MEM_SIZE			(0x800u)	
#define CH201_DATA_MEM_ADDR			(0x0200u)
#define CH201_PROG_MEM_SIZE			(0x800u)	
#define CH201_PROG_MEM_ADDR			(0xF800u)
#define	CH201_FW_SIZE				CH201_PROG_MEM_SIZE

/* CH-101 common definitions */
#define CH101_COMMON_REG_OPMODE 			(0x01u)
#define CH101_COMMON_REG_TICK_INTERVAL 		(0x02u)
#define CH101_COMMON_REG_PERIOD 			(0x05u)
#define CH101_COMMON_REG_CAL_TRIG 			(0x06u)
#define CH101_COMMON_REG_MAX_RANGE 			(0x07u)
#define CH101_COMMON_REG_TIME_PLAN 			(0x09u)
#define CH101_COMMON_REG_CAL_RESULT 		(0x0Au)
#define CH101_COMMON_REG_RX_HOLDOFF			(0x11u)
#define CH101_COMMON_REG_STAT_RANGE 		(0x12u)
#define CH101_COMMON_REG_STAT_COEFF			(0x13u)
#define CH101_COMMON_REG_READY 				(0x14u)
#define CH101_COMMON_REG_TOF_SF 			(0x16u)
#define CH101_COMMON_REG_TOF 				(0x18u)
#define CH101_COMMON_REG_AMPLITUDE			(0x1Au)
#define CH101_COMMON_REG_DATA 				(0x1Cu)

#define CH101_COMMON_I2CREGS_OFFSET			(0u)
#define CH101_COMMON_READY_FREQ_LOCKED		(0x02u)
#define CH101_COMMON_FREQCOUNTERCYCLES  	(128u)

#define	CH101_COMMON_STAT_COEFF_DEFAULT		(6u)			// default value for stationary target coefficient
#define CH101_COMMON_NUM_THRESHOLDS	    	(6u)			// total number of thresholds

/* GPR firmware registers */
#define CH101_GPR_OPEN_REG_OPMODE 			(0x01u)
#define CH101_GPR_OPEN_REG_TICK_INTERVAL 	(0x02u)
#define CH101_GPR_OPEN_REG_PERIOD 			(0x05u)
#define CH101_GPR_OPEN_REG_CAL_TRIG 		(0x06u)
#define CH101_GPR_OPEN_REG_MAX_RANGE 		(0x07u)
#define CH101_GPR_OPEN_REG_CALC				(0x08u)
#define CH101_GPR_OPEN_REG_CAL_RESULT 		(0x0Au)
#define CH101_GPR_OPEN_REG_ST_RANGE 		(0x12u)
#define CH101_GPR_OPEN_REG_READY 			(0x14u)
#define CH101_GPR_OPEN_REG_TOF_SF 			(0x16u)
#define CH101_GPR_OPEN_REG_TOF 				(0x18u)
#define CH101_GPR_OPEN_REG_AMPLITUDE 		(0x1Au)
#define CH101_GPR_OPEN_REG_DATA 			(0x1Cu)

#define	CH101_GPR_OPEN_CTR					(0x2B368u) //177000
#define CH101_GPR_OPEN_MAX_SAMPLES			(150u)

/* CH-201 common definitions */
#define CH201_COMMON_REG_OPMODE 		(0x01u)
#define CH201_COMMON_REG_TICK_INTERVAL 	(0x02u)
#define CH201_COMMON_REG_PERIOD 		(0x05u)
#define CH201_COMMON_REG_CAL_TRIG 		(0x06u)
#define CH201_COMMON_REG_MAX_RANGE 		(0x07u)
#define CH201_COMMON_REG_THRESH_LEN_0 	(0x08u)
#define CH201_COMMON_REG_THRESH_LEN_1 	(0x09u)
#define CH201_COMMON_REG_CAL_RESULT		(0x0Au)
#define CH201_COMMON_REG_THRESH_LEN_2 	(0x0Cu)
#define CH201_COMMON_REG_THRESH_LEN_3 	(0x0Du)
#define CH201_COMMON_REG_RX_HOLDOFF		(0x11u)
#define CH201_COMMON_REG_ST_RANGE 		(0x12u)
#define CH201_COMMON_REG_READY 			(0x14u)
#define CH201_COMMON_REG_THRESH_LEN_4 	(0x15u)
#define CH201_COMMON_REG_THRESHOLDS		(0x16u)	// start of array of six 2-byte threshold levels
#define CH201_COMMON_REG_TOF_SF 		(0x22u)
#define CH201_COMMON_REG_TOF 			(0x24u)
#define CH201_COMMON_REG_AMPLITUDE 		(0x26u)
#define CH201_COMMON_REG_DATA 			(0x28u)

#define CH201_COMMON_I2CREGS_OFFSET		(0u)
#define CH201_COMMON_READY_FREQ_LOCKED	(0x02u)		// XXX need more values (?)
#define CH201_COMMON_FREQCOUNTERCYCLES	(128u)
#define CH201_COMMON_NUM_THRESHOLDS		(6u)		// total number of thresholds

/* GPR with multi thresholds firmware registers */
#define CH201_GPRMT_REG_OPMODE 			(0x01u)
#define CH201_GPRMT_REG_TICK_INTERVAL 	(0x02u)
#define CH201_GPRMT_REG_PERIOD 			(0x05u)
#define CH201_GPRMT_REG_CAL_TRIG 		(0x06u)
#define CH201_GPRMT_REG_MAX_RANGE 		(0x07u)
#define CH201_GPRMT_REG_THRESH_LEN_0 	(0x08u)
#define CH201_GPRMT_REG_THRESH_LEN_1 	(0x09u)
#define CH201_GPRMT_REG_CAL_RESULT 		(0x0Au)
#define CH201_GPRMT_REG_THRESH_LEN_2 	(0x0Cu)
#define CH201_GPRMT_REG_THRESH_LEN_3 	(0x0Du)
#define CH201_GPRMT_REG_ST_RANGE 		(0x12u)
#define CH201_GPRMT_REG_READY 			(0x14u)
#define CH201_GPRMT_REG_THRESH_LEN_4 	(0x15u)
#define CH201_GPRMT_REG_THRESHOLDS		(0x16u)	// start of array of six 2-byte threshold levels
#define CH201_GPRMT_REG_TOF_SF 			(0x22u)
#define CH201_GPRMT_REG_TOF 			(0x24u)
#define CH201_GPRMT_REG_AMPLITUDE 		(0x26u)
#define CH201_GPRMT_REG_DATA 			(0x28u)

#define CH201_GPRMT_MAX_SAMPLES			(450u)	// max number of samples
#define CH201_GPRMT_NUM_THRESHOLDS		(6u)		// total number of thresholds

/* Programming interface register addresses */
#define CH_PROG_REG_PING 		(0x00u)			/*!< Read-only register used during device discovery. */
#define CH_PROG_REG_CPU 		(0x42u)			/*!< Processor control register address. */
#define CH_PROG_REG_STAT 		(0x43u)			/*!< Processor status register address. */
#define CH_PROG_REG_CTL 		(0x44u)			/*!< Data transfer control register address. */
#define CH_PROG_REG_ADDR 		(0x05u)			/*!< Data transfer starting address register address. */
#define CH_PROG_REG_CNT 		(0x07u)			/*!< Data transfer size register address. */
#define CH_PROG_REG_DATA 		(0x06u)			/*!< Data transfer value register address. */

#define CH_PROG_XFER_SIZE		(256u)			/*!< max size of a read operation via programming interface */

/* Sensor operating modes */
#define	CH_MODE_IDLE                (0x00u)
#define	CH_MODE_FREERUN   			(0x02u)
#define	CH_MODE_TRIGGERED_TX_RX 	(0x10u)
#define	CH_MODE_TRIGGERED_RX_ONLY   (0x20u)


/*
 * Structure definition
 */
typedef enum {
	CH_RANGE_ECHO_ONE_WAY 		= 0,		/*!< One way - gets full pulse/echo distance & divides by 2. */
	CH_RANGE_ECHO_ROUND_TRIP	= 1,		/*!< Round trip - full pulse/echo distance. */
	CH_RANGE_DIRECT				= 2			/*!< Direct - for receiving node in pitch-catch mode. */
} eRangeType_t;

typedef struct {
	uint32_t	range;				// from ch_get_range()
	uint16_t	amplitude;			// from ch_get_amplitude()
	uint16_t	reserved;
} chirp_result_t;

typedef struct{
	uint32_t 	freq; 				/*!< Operating frequency for the sensor. */
	uint16_t 	rtc_result; 		/*!< Real-time clock calibration result for the sensor. */
	uint16_t 	scale_factor; 		/*!< Scale factor for the sensor. */
} chirp_calib_t;

typedef struct {
	uint8_t		mode;				/*!< operating mode */
	uint16_t	max_range;			/*!< maximum range, in mm */
	uint16_t	static_range;		/*!< static target rejection range, in mm (0 if unused) */
	uint16_t 	sample_interval;	/*!< sample interval, only used if in free-running mode */
} stConfig_t;

//! Detection threshold value (CH201 only).
typedef struct {
	uint16_t		start_sample;
	uint16_t		level;
} ch_thresh_t;

//! Multiple detection threshold structure (CH201 only).
typedef struct {
	ch_thresh_t		threshold[CH_NUM_THRESHOLDS];
} ch_thresholds_t;


#endif /* _CONSTANT_H_ */
