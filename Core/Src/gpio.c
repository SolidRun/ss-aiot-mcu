/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "stdbool.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC14-OSC32_IN   ------> RCC_OSC32_IN
     PC15-OSC32_OUT   ------> RCC_OSC32_OUT
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPS_RSTN_Pin|BATT_QON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BATT_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GNSS_PWR_EN_Pin|MCU_INT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IR_SENS_INT_Pin _6AX_INT_Pin */
  GPIO_InitStruct.Pin = IR_SENS_INT_Pin|_6AX_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GNSS_GPS1PPS_Pin GPS_INT_Pin SOM_EN_Pin BATT_STAT_Pin */
  GPIO_InitStruct.Pin = GNSS_GPS1PPS_Pin|GPS_INT_Pin|SOM_EN_Pin|BATT_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_RSTN_Pin BATT_CE_Pin BATT_QON_Pin */
  GPIO_InitStruct.Pin = GPS_RSTN_Pin|BATT_CE_Pin|BATT_QON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BATT_INT_Pin */
  GPIO_InitStruct.Pin = BATT_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BATT_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GNSS_PWR_EN_Pin LED_MCU_Pin */
  GPIO_InitStruct.Pin = GNSS_PWR_EN_Pin|LED_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_INT_Pin */
  GPIO_InitStruct.Pin = MCU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BATT_PG_Pin */
  GPIO_InitStruct.Pin = BATT_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BATT_PG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 2 */

volatile bool sensors_ready = false;

void GPIO_EnableSensorInterrupts(void)
{
  sensors_ready = true;
  __HAL_GPIO_EXTI_GENERATE_SWIT(IR_SENS_INT_Pin|_6AX_INT_Pin);
}

void GPIO_InitSomEnablePin(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_PinState state = HAL_GPIO_ReadPin(SOM_EN_GPIO_Port, SOM_EN_Pin);

    /* on init logical state follows electrical state (i.e. follow assembled pull-up/-down) */
    HAL_GPIO_WritePin(SOM_EN_GPIO_Port, SOM_EN_Pin, state);

    /* set output */
    GPIO_InitStruct.Pin   = SOM_EN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SOM_EN_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE END 2 */
