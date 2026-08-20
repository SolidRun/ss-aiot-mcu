/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */
/* Written to RTC_BKP_DR1 once the calendar has been set from GNSS. It lives in
 * the backup domain next to the DR0 calendar-valid marker, so it survives a
 * reset and is lost only when VBAT is lost - which is exactly when the calendar
 * itself becomes meaningless. Without it gps_time_synced is a RAM flag that
 * reads false after every reset, and the SOM is told the time is unverified
 * when it is in fact still good. */
#define RTC_GPS_SYNC_MAGIC   0x6B1DU

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == 0x32F2U)
  {
    return;
  }
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 1;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
                              |RTC_ALARMMASK_MINUTES;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2U);
  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_TAMP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_TAMP_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
    __HAL_RCC_RTCAPB_CLK_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_TAMP_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void rtc_updeteTime(uint8_t Hours, uint8_t Minutes , uint8_t Seconds)
{
	  RTC_TimeTypeDef s_Time = {0};
	  HAL_RTC_GetTime(&hrtc, &s_Time, RTC_FORMAT_BIN);
	  s_Time.Hours = Hours;
	  s_Time.Minutes = Minutes;
	  s_Time.Seconds = Seconds;
	  if (HAL_RTC_SetTime(&hrtc, &s_Time, RTC_FORMAT_BIN) != HAL_OK)
	  {

	  }
}

void rtc_updeteDate(uint8_t Month, uint8_t Date , uint8_t Year)
{

	  RTC_DateTypeDef s_Date = {0};

	  HAL_RTC_GetDate(&hrtc, &s_Date, RTC_FORMAT_BIN);
	  s_Date.Month = Month;
	  s_Date.Date = Date;
	  s_Date.Year = Year;

	  if (HAL_RTC_SetDate(&hrtc, &s_Date, RTC_FORMAT_BIN) != HAL_OK)
	  {

	  }
}

void rtc_getTime(RTC_TimeTypeDef* s_Time)
{
	  HAL_RTC_GetTime(&hrtc, s_Time, RTC_FORMAT_BIN);
}

/* Did the calendar get set from GNSS at some point, and has it been kept alive
 * since? Both markers have to be present: DR0 says the calendar is running,
 * DR1 says GNSS is where its value came from. */
bool rtc_gpsSyncIsValid(void)
{
  return ((HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == 0x32F2U) &&
          (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == RTC_GPS_SYNC_MAGIC));
}

void rtc_markGpsSynced(void)
{
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_GPS_SYNC_MAGIC);
}

void rtc_getDate(RTC_DateTypeDef* s_Date)
{
	  HAL_RTC_GetDate(&hrtc, s_Date, RTC_FORMAT_BIN);
}
/* USER CODE END 1 */
