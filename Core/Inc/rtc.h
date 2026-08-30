/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stm32u0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_RTC_Init(void);

/* USER CODE BEGIN Prototypes */
void rtc_updeteTime(uint8_t Hours, uint8_t Minutes , uint8_t Seconds);
void rtc_updeteDate(uint8_t Month, uint8_t Date , uint8_t Year);
void rtc_getTime(RTC_TimeTypeDef* s_Time);
void rtc_getDate(RTC_DateTypeDef* s_Date);
bool rtc_gpsSyncIsValid(void);
bool rtc_setDailyAlarm(uint8_t Hours, uint8_t Minutes, uint8_t Seconds);
void rtc_cancelAlarm(void);
bool rtc_getAlarm(uint8_t *Hours, uint8_t *Minutes, uint8_t *Seconds);
void rtc_alarmInit(void);
void rtc_markGpsSynced(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */

