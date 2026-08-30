/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* Interrupt sources, reported as a bitfield in data[0] of the interrupt
 * read (0x12 0x07). A source occupies its bit whether or not it currently
 * has a detail byte attached. */
#define INT_SRC_MCU      0x01U   /* the firmware itself, no detail byte yet */
#define INT_SRC_IR       0x02U   /* STHS34PF80,  detail in data[1]          */
#define INT_SRC_ACCEL    0x04U   /* ISM330DHCX,  detail in data[2]          */
#define INT_SRC_RTC      0x08U   /* reserved, never set yet                 */
#define INT_SRC_CHARGER  0x10U   /* reserved, never set yet                 */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SomEnable(void);
void SomDisable(void);
void somSetInt(uint8_t source);
void somTakeInterrupts(uint8_t *mcu, uint8_t *ir, uint8_t *acc);
void resetI2C2(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IR_SENS_INT_Pin GPIO_PIN_0
#define IR_SENS_INT_GPIO_Port GPIOA
#define IR_SENS_INT_EXTI_IRQn EXTI0_1_IRQn
#define _6AX_INT_Pin GPIO_PIN_1
#define _6AX_INT_GPIO_Port GPIOA
#define _6AX_INT_EXTI_IRQn EXTI0_1_IRQn
#define GNSS_GPS1PPS_Pin GPIO_PIN_0
#define GNSS_GPS1PPS_GPIO_Port GPIOB
#define GPS_INT_Pin GPIO_PIN_1
#define GPS_INT_GPIO_Port GPIOB
#define GPS_RSTN_Pin GPIO_PIN_2
#define GPS_RSTN_GPIO_Port GPIOB
#define BATT_INT_Pin GPIO_PIN_10
#define BATT_INT_GPIO_Port GPIOB
#define BATT_INT_EXTI_IRQn EXTI4_15_IRQn
#define BATT_CE_Pin GPIO_PIN_11
#define BATT_CE_GPIO_Port GPIOB
#define BATT_QON_Pin GPIO_PIN_12
#define BATT_QON_GPIO_Port GPIOB
#define GNSS_PWR_EN_Pin GPIO_PIN_8
#define GNSS_PWR_EN_GPIO_Port GPIOA
#define i2c2_SCL_Pin GPIO_PIN_9
#define i2c2_SCL_GPIO_Port GPIOA
#define i2c2_SDA_Pin GPIO_PIN_10
#define i2c2_SDA_GPIO_Port GPIOA
#define LED_MCU_Pin GPIO_PIN_12
#define LED_MCU_GPIO_Port GPIOA
#define MCU_INT_Pin GPIO_PIN_15
#define MCU_INT_GPIO_Port GPIOA
#define SOM_EN_Pin GPIO_PIN_5
#define SOM_EN_GPIO_Port GPIOB
#define BATT_STAT_Pin GPIO_PIN_8
#define BATT_STAT_GPIO_Port GPIOB
#define BATT_PG_Pin GPIO_PIN_9
#define BATT_PG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
