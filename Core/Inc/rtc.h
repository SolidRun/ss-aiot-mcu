/*
 * rtc.h
 *
 *  Created on: Sep 30, 2025
 *      Author: User
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "stm32u0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>


void rtc_updeteTime(uint8_t Hours, uint8_t Minutes , uint8_t Seconds);
void rtc_updeteDate(uint8_t Month, uint8_t Date , uint8_t Year);
void rtc_getTime(RTC_TimeTypeDef* s_Time);
void rtc_getDate(RTC_DateTypeDef* s_Date);
#endif /* INC_RTC_H_ */
