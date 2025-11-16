/*
 * rtc.c
 *
 *  Created on: Sep 30, 2025
 *      Author: User
 */
#include "rtc.h"

extern RTC_HandleTypeDef hrtc;

//void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
//  RTC_AlarmTypeDef sAlarm;
//  HAL_RTC_GetAlarm(hrtc,&sAlarm,RTC_ALARM_A,FORMAT_BIN);
//  if(sAlarm.AlarmTime.Seconds>58) {
//    sAlarm.AlarmTime.Seconds=0;
//  }else{
//    sAlarm.AlarmTime.Seconds=sAlarm.AlarmTime.Seconds+1;
//  }
//    while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}
//    HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
//}

void rtc_updeteTime(uint8_t Hours, uint8_t Minutes , uint8_t Seconds)
{
	  RTC_TimeTypeDef s_Time = {0};
	  HAL_RTC_GetTime(&hrtc, &s_Time, RTC_FORMAT_BCD);
	  s_Time.Hours = Hours;
	  s_Time.Minutes = Minutes;
	  s_Time.Seconds = Seconds;
	  if (HAL_RTC_SetTime(&hrtc, &s_Time, RTC_FORMAT_BCD) != HAL_OK)
	  {

	  }
}

void rtc_updeteDate(uint8_t Month, uint8_t Date , uint8_t Year)
{

	  RTC_DateTypeDef s_Date = {0};

	  HAL_RTC_GetDate(&hrtc, &s_Date, RTC_FORMAT_BCD);
	  s_Date.Month = Month;
	  s_Date.Date = Date;
	  s_Date.Year = Year;

	  if (HAL_RTC_SetDate(&hrtc, &s_Date, RTC_FORMAT_BCD) != HAL_OK)
	  {

	  }
}

void rtc_getTime(RTC_TimeTypeDef* s_Time)
{
	  HAL_RTC_GetTime(&hrtc, s_Time, RTC_FORMAT_BCD);
}

void rtc_getDate(RTC_DateTypeDef* s_Date)
{
	  HAL_RTC_GetDate(&hrtc, s_Date, RTC_FORMAT_BCD);
}
