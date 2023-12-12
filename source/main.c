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
 * @file main.c
 * @version $Id: main.c $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2022 TDK Corporation All rights reserved.
 *
 * @brief Main routine of the ultrasonic sensor system
 * 
 **/

#include "pico/stdlib.h"
#include <stdio.h>
#include "class/cdc/cdc_device.h"

#include "constant.h"
#include "ch_lib.h"
#include "usr_def.h"

/*
 * External variable declaration
 */
/* none */

/*
 * Prototype declaration
 */
/* buffer size definition */
#define MEAS_BUF_SIZE            (10u)	/* Median calculate buffer */
#define TOKEN_BUF_SIZE			(256u)	/* command token buffer */

#define	NO_TOKEN				(-1)	/* token is not received properly */

#define SENSOR_TYPE         (CHIRP_TYPE_CH201)
#define RANGE_LIMIT_UPPER   (320.0)
#define RANGE_LIMIT_LOWER   (280.0)
#define AMP_MAX             (30000u)

/* command code definition */
#define	SEN_CMDCODE_STOP	(0U)
#define	SEN_CMDCODE_SET		(1U)
#define	SEN_CMDCODE_MOF		(2U)
#define	SEN_CMDCODE_DIST	(3U)
#define	SEN_CMDCODE_CLR		(4U)
#define	SEN_CMDCODE_UNKNOWN	(5U)

typedef float float32_t;
typedef struct {
	const uint8_t *cmdstr;	/* command (string) */
	uint8_t cmdlen;			/* command length */
	uint8_t cmdcode;		/* command code */
	int16_t reserved;	    /* alignment */
}stCmdDefineTable_t;

typedef struct {
	float32_t   range;				// from ch_get_range()
	uint16_t	amplitude;			// from ch_get_amplitude()
	uint16_t	reserved;			// alignment */
} stMeasResult_t;

/* command definition table */
const stCmdDefineTable_t sCDTable[]={
	{ "\0",   1U, SEN_CMDCODE_STOP, 0},	/* stop string , strlen, cmdcode, reserve */
	{ "set",  4U, SEN_CMDCODE_SET,  0},	/* set string ,  strlen, cmdcode, reserve */
	{ "mof",  4U, SEN_CMDCODE_MOF,  0},	/* mof string ,  strlen, cmdcode, reserve */
	{ "dist", 5U, SEN_CMDCODE_DIST, 0},	/* dist string , strlen, cmdcode, reserve */
	{ "clear",6U, SEN_CMDCODE_CLR,  0},	/* clear string ,strlen, cmdcode, reserve */
	{ "\0",   0U, SEN_CMDCODE_UNKNOWN,0}	/* terminate area */
};

/* Variable definitons indicating measurement status */
#define	MMODE_MOFMOF		(0U)
#define	MMODE_DISTANCE		(1U)
static	uint8_t	sMeasureMode = MMODE_MOFMOF;

stMeasResult_t	sMdn_buf[MEAS_BUF_SIZE];
uint8_t			sMdn_widx;
uint8_t			sMdn_cnt;

static	uint8_t	sToken[TOKEN_BUF_SIZE];
static	uint8_t sToken_idx = 0;


int8_t	get_token( uint8_t *cmd_buf );
uint8_t	get_cmdcode( uint8_t *cmd_buf, uint8_t len );
void	execute_cmd( uint8_t cmdcode );

void	measure_mof( chirp_result_t *ch_result );
void	measure_dist( chirp_result_t *ch_result );

void	clrMedian( void );
int8_t	calcMedian(stMeasResult_t *onemeas, float32_t *range, uint16_t *amp);


/*=========================================================================================
 * \brief	Main routine
 * \param	none
 * \return	zero
============================================================================================*/
int main(void)
{
    chirp_result_t  ch_result;

    uint8_t         cmd_buf[TOKEN_BUF_SIZE]   = {0};
	uint8_t			cmdcode;

	int8_t			ret_len;
    int8_t       	ret_ch = RET_NG;

    (void)stdio_init_all();
	
    while (true) {
		ret_len = get_token( cmd_buf );
		if ( ret_len == NO_TOKEN ){
			/* no input */
		}
		else{
			/* token exist */
			cmdcode = get_cmdcode( cmd_buf, (uint8_t)ret_len );
			execute_cmd( cmdcode );
		}

        //Get result if data is ready
        ret_ch = CH_API_GetResult(&ch_result);

		if ( ret_ch != RET_FINISH ){
			continue;
		} else { /* else nothing */ }

		if ( ch_result.range == CH_NO_TARGET ){
			DEBUG_PRINT_STR("no target found\n");
			continue;
		} else { /* else nothing */ }

		switch( sMeasureMode ){
		case MMODE_MOFMOF:
			measure_mof( &ch_result );
			break;
		case MMODE_DISTANCE:
		default:
			measure_dist( &ch_result );
			break;
		}
	}/* while(true) */
}

/*=========================================================================================
 * \brief	get tokent 
 * \param	uint8_t *cmd_buf
 * \return	int8_t  length of the string including the null character
============================================================================================*/
/* Returns the length of the string including the null character */
int8_t get_token( uint8_t* cmd_buf )
{
	uint8_t ch;
	int8_t	ret;

	if ( tud_cdc_available() > 0u ){
		/* avoid buffer overflow */
		if ( sToken_idx >= TOKEN_BUF_SIZE ){
			sToken_idx = 0;
		}

		ch = (uint8_t)tud_cdc_read_char( );

		if ( ch == (uint8_t)'\r' ){ 
			sToken[sToken_idx] = (uint8_t)'\0';
			sToken_idx ++;
			(void)memcpy(cmd_buf, sToken, sToken_idx );
			ret = (int8_t)sToken_idx;
			sToken_idx = 0;
		}
		else{
			sToken[sToken_idx] = ch;
			sToken_idx ++;
			ret = NO_TOKEN;
		}
	}
	else{
		ret = NO_TOKEN;
	}
	return( ret );
}

/*=========================================================================================
 * \brief	get command
 * \param	uint8_t *cmd_buf
 *          uint8_t  len
 * \return	void
============================================================================================*/
uint8_t get_cmdcode( uint8_t *cmd_buf, uint8_t len )
{
	uint8_t i;
	uint8_t ret_cmp;
	uint8_t ret_cmdcode = SEN_CMDCODE_UNKNOWN;

	for( i = 0 ; i < SEN_CMDCODE_UNKNOWN ; i ++ ){
		if ( len == sCDTable[i].cmdlen ) {
			ret_cmp = (uint8_t)memcmp( cmd_buf, sCDTable[i].cmdstr, len );
			if ( ret_cmp == 0u ){
				ret_cmdcode = sCDTable[i].cmdcode;
				break;
			}
			else{
				/* continue */
			}
		}
		else{
			/* continue */
		}
	}
	if ( i >= SEN_CMDCODE_UNKNOWN ){
		DEBUG_PRINTF("%s : no such command\n", cmd_buf);
	}
	return ( ret_cmdcode );
}

/*=========================================================================================
 * \brief	command execution
 * \param	uint8_t cmdcode 
 * \return	void
============================================================================================*/
void execute_cmd( uint8_t cmdcode )
{
	int8_t	ret;

	switch( cmdcode ){
	case SEN_CMDCODE_STOP: /* \r */
		DEBUG_PRINT_STR("---------------------------------------\n");
	    CH_API_MeasStop();
		break;
	case SEN_CMDCODE_SET: /* set */
		DEBUG_PRINT_STR("---------------------------------------\n");
        (void)CH_API_Init(SENSOR_TYPE);
		break;
	case SEN_CMDCODE_MOF: /* mof */
		sMeasureMode = MMODE_MOFMOF;
		ret = CH_API_Config(CH_MODE_FREERUN, 500, 400);
        if (ret == RET_OK) {
        	DEBUG_PRINT_STR("\nConfiguration is succeeded\n");
		} else {
			DEBUG_PRINT_STR("\nConfiguration is failed\n");
		}
		CH_API_MeasStart();
		break;
	case SEN_CMDCODE_DIST:	/* dist */
		sMeasureMode = MMODE_DISTANCE;
		ret = CH_API_Config(CH_MODE_FREERUN, 500, 400);
		if (ret == RET_OK) {
			DEBUG_PRINT_STR("\nConfiguration is succeeded\n");
		} else {
			DEBUG_PRINT_STR("\nConfiguration is failed\n");
		}
		CH_API_MeasStart();
		break;
	case SEN_CMDCODE_CLR:
		clrMedian( );
        DEBUG_PRINT_STR("Data Buffer is cleared\n");
		break;
	default: /* include SEN_CMDCODE_UNKNOWN */
		break;
	}
	return;
}

/*=========================================================================================
 * \brief	measure softness
 * \param	chirp_result_t *   measurement result 
 * \return	void
============================================================================================*/
void measure_mof( chirp_result_t *ch_result )
{
    float32_t meas_range   = 0.0f;
    float32_t median_range = -1.0f;
    uint16_t  median_amp   = 0;
    float32_t softness     = 0.0f;

	stMeasResult_t	onemeas;
	int8_t	data_count;			

	meas_range = ((float32_t)ch_result->range)/32.0f;
	if(meas_range >= RANGE_LIMIT_UPPER){
		DEBUG_PRINT_STR("too far \n");	/* Not included in median calculation */
	}
	else if (meas_range < RANGE_LIMIT_LOWER){
		DEBUG_PRINT_STR("too near\n");	/* Not included in median calculation */
	}
	else{
		onemeas.range = meas_range;
		onemeas.amplitude = ((ch_result->amplitude > AMP_MAX) ? AMP_MAX : (ch_result->amplitude) );
		data_count = calcMedian( &onemeas , &median_range, &median_amp );
		if ( data_count < 0 ){
			DEBUG_PRINTF("%d data is measured, need %d more data\n", (data_count*(-1)), ((int8_t)MEAS_BUF_SIZE - (data_count*(-1))) );
		}else{
			softness = ( ((float32_t)AMP_MAX - (float32_t)median_amp)/(float32_t)AMP_MAX ) * (float32_t)100;
			DEBUG_PRINTF("Range: %0.1f mm  Amp: %u  Softness: %0.1f%%\n", median_range, median_amp, softness);
		}
	}
}

/*=========================================================================================
 * \brief	printf measurement result
 * \param	chirp_result_t *   measurement result 
 * \return	void
============================================================================================*/
void measure_dist( chirp_result_t *ch_result )
{
	DEBUG_PRINTF("Range: %5.1f mm  Amp: %5u\n", ((float32_t)ch_result->range)/32.0f, ch_result->amplitude);
}

/*=========================================================================================
 * \brief	calculate median value
 * \param	stMeasResult_t*     new data for median value calculation
 *          *range   calculated range
 *          *amp     calculated amp 
 * \return	zero
============================================================================================*/
int8_t calcMedian(stMeasResult_t *onemeas, float32_t *range, uint16_t *amp) 
{
	uint8_t	i;
	uint8_t	j;
    int8_t ret_code;
	stMeasResult_t tmp;
	stMeasResult_t sort_buf[MEAS_BUF_SIZE];
	uint8_t cnt_meas_buf;

	sMdn_buf[sMdn_widx] = *onemeas;
	sMdn_widx ++;

	if ( sMdn_widx >= MEAS_BUF_SIZE ){
		sMdn_widx = 0;
	}
	if ( sMdn_cnt < MEAS_BUF_SIZE ){
		sMdn_cnt ++;
		ret_code = (int8_t)(sMdn_cnt) * -1;
	}
	else{ /* ( sMdn_cnt == MEAS_BUF_SIZE ) */
		ret_code = (int8_t)sMdn_cnt;

		/* Copy to buffer for sorting  */
		(void)memcpy( sort_buf , sMdn_buf, sizeof(sMdn_buf) );
		
		/* sort by amplitude */
		for (i=0; i<MEAS_BUF_SIZE; ++i) {
			for (j=i+1u; j< MEAS_BUF_SIZE; ++j) {
				if (sort_buf[i].amplitude > sort_buf[j].amplitude) {
					tmp =  sort_buf[i];
					sort_buf[i] = sort_buf[j];
					sort_buf[j] = tmp;
				}
			}
		}
		cnt_meas_buf = MEAS_BUF_SIZE;
		if ((cnt_meas_buf % 2u) == 0u) {
			*range = (sort_buf[MEAS_BUF_SIZE / 2u].range + sort_buf[(MEAS_BUF_SIZE / 2u) - 1u].range) / 2.0f;
			*amp   = (sort_buf[MEAS_BUF_SIZE / 2u].amplitude + sort_buf[(MEAS_BUF_SIZE / 2u) - 1u].amplitude) / 2u;
		} else {
			*range = sort_buf[MEAS_BUF_SIZE / 2u].range;
			*amp   = sort_buf[MEAS_BUF_SIZE / 2u].amplitude;
		}
	}
	return ( ret_code );
}

/*=========================================================================================
 * \brief	clear buffer for median value calculation
 * \param	void
 * \return	void
============================================================================================*/
void clrMedian( void  )
{
	sMdn_widx = 0;
	sMdn_cnt = 0;
	return;
}

