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
 * @file ch_lib.c
 * @version $Id: ch_lib.c $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2022 TDK Corporation All rights reserved.
 *
 * @brief API of the ultrasonic sensor system
 * 
 **/

//#include "ch_types.h"
#include "constant.h"
#include "ch_com.h"
#include "ch_drv_ch101.h"
#include "ch_drv_ch201.h"
#include "usr_bsp.h"
#include "usr_def.h"
#include "ch_lib.h"

/*
 * External variable declaration
 */
static stConfig_t sConfig;
static uint16_t sChirpType;

/* Detection level settings - for CH201 sensors only */
/* Each threshold entry includes the starting sample number & threshold level. */
#if 0
static ch_thresholds_t sThresholds = {
	.threshold = {
		{0, 	5000},		/* threshold 0 */
		{26,	2000},		/* threshold 1 */
		{39,	800},		/* threshold 2 */
		{56,	400},		/* threshold 3 */
		{79,	250},		/* threshold 4 */
		{89,	175}		/* threshold 5 */
	}
};
#endif
static ch_thresholds_t sThresholds = {
	.threshold = {
		{0, 	400},		/* threshold 0 */
		{12,	5000},		/* threshold 1 */
		{24,	3000},		/* threshold 2 */
		{36,	30000},		/* threshold 3 */
		{48,	30000},		/* threshold 4 */
		{60,	30000}		/* threshold 5 */
	}
};


/*
 * Prototype declaration
 */
/* none */


/*=========================================================================================
 * \brief	Initialize the sensor.
 * \param	type  CHIRP_TYPE_CH101/CHIRP_TYPE_CH201
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t CH_API_Init(uint16_t type)
{
	static uint8_t sPassed = 0;
	uint16_t sig_word;
	int8_t ret = RET_NG;

	/*Specify sensor type */
	sChirpType = type;
	if (type == CHIRP_TYPE_CH201) 
	{
		gChirpDev = gChirpDev201;
	}
	else if (type == CHIRP_TYPE_CH101)
	{
		gChirpDev = gChirpDev101;
	}
	else
	{
		//do nothing
	}

	/* Initialize variable */
	sConfig.mode = gChirpDev.mode;
	sConfig.max_range = gChirpDev.max_range;
	sConfig.sample_interval = gChirpDev.interval;
	sConfig.static_range = 0;

	/* Initialize I/O pin */
	USR_GPIO_Init();
	
	/* Reset all sensors */
	USR_GPIO_Set(RST_PIN, LOW);
	USR_WaitMS(1);
	USR_GPIO_Set(RST_PIN, HIGH);

	/* Configure I2C */
	if (sPassed == 0u) {
		sPassed = 1;
		(void)COM_Config(CHIRP_I2C_SPEED);
	}

	/* Assert PROG pin */
	USR_GPIO_Set(gChirpDev.prog_pin, HIGH);
	
	/* Detect the sensor */
	ret = COM_ReadReg(CHIRP_I2C_ADDR_PROG, 0x00, &sig_word, 2);
	if ((ret == RET_OK) && (sig_word == CH_SIG_WORD))
	{
		ret = RET_OK;
		DEBUG_PRINT_STR("Sensor : Connected\n");
	}
	else
	{
		ret = RET_NG;
		DEBUG_PRINT_STR("Sensor : Not Connected\n");
	}
	
	/* De-assert PROG pin */
	USR_GPIO_Set(gChirpDev.prog_pin, LOW);

	if(sChirpType == CHIRP_TYPE_CH201)
	{
		/* Load firmware */
		if (ret == RET_OK) {
			ret = DRV_FWLoad_CH201();
		}
		/* Check the sensor ready */
		if (ret == RET_OK) {
			ret = DRV_WaitForLock_CH201(CHDRV_FREQLOCK_TIMEOUT_MS);
		}
		/* Calibrate the sensor RTC */
		if (ret == RET_OK) {
			USR_WaitMS(1);
			ret = DRV_MeasRTC_CH201();
		}
	}
	else if (sChirpType == CHIRP_TYPE_CH101)
	{
		/* Load firmware */
		if (ret == RET_OK) {
			ret = DRV_FWLoad_CH101();
		}
		/* Check the sensor ready */
		if (ret == RET_OK) {
			ret = DRV_WaitForLock_CH101(CHDRV_FREQLOCK_TIMEOUT_MS);
		}
		/* Calibrate the sensor RTC */
		if (ret == RET_OK) {
			USR_WaitMS(1);
			ret = DRV_MeasRTC_CH101();
		}
	}
	else
	{
		//do nothing
	}
	return ret;
}

/*=========================================================================================
 * \brief	Configure sensor settings.
 * \param	range		measurement range
 *			interval	measurement interval
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t CH_API_Config(uint8_t mode, uint16_t range, uint16_t interval)
{
	uint16_t set_range;
	uint16_t set_interval;
	uint8_t  set_mode;
	int8_t   ret = RET_NG;

	if( (mode == CH_MODE_IDLE)
	 || (mode == CH_MODE_FREERUN)
	 || (mode == CH_MODE_TRIGGERED_TX_RX)
	 || (mode == CH_MODE_TRIGGERED_RX_ONLY) )
	{
		set_mode = mode;
	}
	else
	{
		set_mode = sConfig.mode;
	}

	if ( ((sChirpType == CHIRP_TYPE_CH101) && ((range > 1200u)||(range < 40u)))
	 ||  ((sChirpType == CHIRP_TYPE_CH201) && ((range > 5000u)||(range < 200u))) )
	{
		set_range = sConfig.max_range;	/* Use the last setting */
	}
	else 
	{
		set_range = range;				/* Reconfigure */
	}
	set_interval = interval;

	/* Stores configurations */
	sConfig.mode			= set_mode;
	sConfig.max_range		= set_range;
	sConfig.sample_interval = set_interval;
	sConfig.static_range	= 0;

	if (sChirpType == CHIRP_TYPE_CH201)
	{
		/* Set mode */
		ret = DRV_SetMode_CH201(set_mode);
		/* Set max range */
		if (ret == RET_OK) {
			ret = DRV_SetMaxRange_CH201(set_range);
		}
		/* Set sample interval */
		if (ret == RET_OK) {
			ret = DRV_SetSampleInterval_CH201(set_interval);
		}
		/* Set multiple thresholds */
		if (ret == RET_OK) {
			ret = DRV_SetThresholds_CH201(&sThresholds);
		}
	}
	else if (sChirpType == CHIRP_TYPE_CH101)
	{
		/* Set mode */
		ret = DRV_SetMode_CH101(set_mode);
		/* Set max range */
		if (ret == RET_OK) {
			ret = DRV_SetMaxRange_CH101(set_range);
		}
		/* Set static range */
		if (ret == RET_OK) {
			ret = DRV_SetStaticRange_CH101(0);
		}
		/* Set sample interval */
		if (ret == RET_OK) {
			ret = DRV_SetSampleInterval_CH101(set_interval);
		}
	}
	else
	{
		//do nothing
	}
	return ret;
}

/*=========================================================================================
 * \brief	Start measurement.
 * \param	none
 * \return	none
============================================================================================*/
void CH_API_MeasStart(void)
{
	if (sConfig.mode == CH_MODE_FREERUN)
	{
		/* INT pin is the input setting. At the same time, the DIR pin is also controlled. */
		USR_GPIO_Set(gChirpDev.dir_pin, DIR_SENSOR_TO_HOST);
		USR_GPIO_SetDir(gChirpDev.int_pin, GPIO_IN);
		USR_GPIO_SetPullDown(gChirpDev.int_pin);
		/* Enable GPIO interrupt */
		USR_IntrEnable(gChirpDev.int_pin);
	}
	else if (sConfig.mode == CH_MODE_TRIGGERED_TX_RX)
	{
		USR_TimerEnable(sConfig.sample_interval);
	}
	else
	{
		// do noting
	}
}

/*=========================================================================================
 * \brief	Stop measurement.
 * \param	none
 * \return	none
============================================================================================*/
void CH_API_MeasStop(void)
{
	if (sConfig.mode == CH_MODE_FREERUN)
	{
		/* Disable GPIO interrupt */
		USR_IntrDisable(gChirpDev.int_pin);
	}
	else if (sConfig.mode == CH_MODE_TRIGGERED_TX_RX)
	{	
		USR_TimerDisable();
	}
	else
	{
		// do nothing
	}
	/* Transition to IDLE mode */
	if (sChirpType == CHIRP_TYPE_CH201)
	{
		(void)DRV_SetMode_CH201(CH_MODE_IDLE);
	}
	else if (sChirpType == CHIRP_TYPE_CH101)
	{
		(void)DRV_SetMode_CH101(CH_MODE_IDLE);
	}
	else
	{
		//do nothing
	}
}

/*=========================================================================================
 * \brief	Get measurement results.
 * \param	p_result	measurement result
 * \return	measurement finish or not
============================================================================================*/
int8_t CH_API_GetResult(chirp_result_t *p_result)
{
	chirp_result_t meas_data;
	int8_t ret = RET_NOT_FINISH;

	if (p_result == NULL)
	{
		return RET_NOT_FINISH;
	}
	if (sConfig.mode == CH_MODE_TRIGGERED_TX_RX)
	{
		if (USR_TimerCallbackOccurred() != 0u)
		{
			if (sChirpType == CHIRP_TYPE_CH201)
			{
				DRV_GenPulseUS_CH201(5);
			}
			else if (sChirpType == CHIRP_TYPE_CH101)
			{
				DRV_GenPulseUS_CH101(5);
			}
			else
			{
				// do noting
			}
			USR_WaitMS(10);
			/* Enable GPIO interrupt */
			USR_IntrEnable(gChirpDev.int_pin);
		}
		/* Get measurement results */
		if (USR_IntrCallbackOccured() != 0u)
		{
			ret = RET_FINISH;
			USR_IntrDisable(gChirpDev.int_pin);
			USR_IntrCallbackRefresh();

			if (sChirpType == CHIRP_TYPE_CH201)
			{
				/* Get range */
				meas_data.range = DRV_GetRange_CH201();		// range_type = CH_RANGE_ECHO_ONE_WAY
				/* Get amplitude */
				if (meas_data.range == CH_NO_TARGET)
				{
					meas_data.amplitude = 0;
				}
				else
				{
					meas_data.amplitude = DRV_GetAmplitude_CH201();
				}
				*p_result = meas_data;
			}
			else if (sChirpType == CHIRP_TYPE_CH101)
			{
				/* Get range */
				meas_data.range = DRV_GetRange_CH101();		// range_type = CH_RANGE_ECHO_ONE_WAY
				/* Get amplitude */
				if (meas_data.range == CH_NO_TARGET)
				{
					meas_data.amplitude = 0;
				}
				else
				{
					meas_data.amplitude = DRV_GetAmplitude_CH101();
				}
				*p_result = meas_data;
			}
			else
			{
				//do nothing
			}
		}
	}
	else if (sConfig.mode == CH_MODE_FREERUN)
	{
		/* Get measurement results */
		if (USR_IntrCallbackOccured() != 0u)
		{
			ret = RET_FINISH;
			USR_IntrCallbackRefresh();
			if (sChirpType == CHIRP_TYPE_CH201)
			{
				/* Get range */
				meas_data.range = DRV_GetRange_CH201();		// range_type = CH_RANGE_ECHO_ONE_WAY
				/* Get amplitude */
				if (meas_data.range == CH_NO_TARGET)
				{
					meas_data.amplitude = 0;
				}
				else
				{
					meas_data.amplitude = DRV_GetAmplitude_CH201();
				}
				*p_result = meas_data;
			}
			else if (sChirpType == CHIRP_TYPE_CH101)
			{
				/* Get range */
				meas_data.range = DRV_GetRange_CH101();		// range_type = CH_RANGE_ECHO_ONE_WAY

				/* Get amplitude */
				if (meas_data.range == CH_NO_TARGET) 
				{
					meas_data.amplitude = 0;
				}
				else
				{
					meas_data.amplitude = DRV_GetAmplitude_CH101();
				}
				*p_result = meas_data;
			}
			else
			{
				//do nothing
			}
		}
	}
	else
	{
		// do nothing
	}
	return ret;
}
