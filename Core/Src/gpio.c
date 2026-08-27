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

  /*Configure GPIO pin Output Level Low */
  HAL_GPIO_WritePin(GPIOA, LED_MCU_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, BATT_CE_Pin|SOM_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level High */
  HAL_GPIO_WritePin(GPIOA, GNSS_PWR_EN_Pin|MCU_INT_Pin, GPIO_PIN_SET);

  /* IR_SENS_INT_Pin and _6AX_INT_Pin are deliberately NOT configured here.
   * They are left in their reset state until GPIO_EnableSensorInterrupts()
   * is called, once the sensor drivers are up - see below. */

  /*Configure GPIO pins : GNSS_GPS1PPS_Pin GPS_INT_Pin BATT_STAT_Pin */
  GPIO_InitStruct.Pin = GNSS_GPS1PPS_Pin|GPS_INT_Pin|BATT_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_RSTN_Pin BATT_CE_Pin BATT_QON_Pin SOM_EN_Pin */
  GPIO_InitStruct.Pin = GPS_RSTN_Pin|BATT_CE_Pin|BATT_QON_Pin|SOM_EN_Pin;
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
  /* EXTI0_1 belongs to the sensor lines and is set up together with them in
   * GPIO_EnableSensorInterrupts(), not here. */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 2 */

/**
  * @brief  Configure and enable the sensor interrupt lines, PA0 (IR_SENS_INT)
  *         and PA1 (_6AX_INT).
  *
  * Kept out of MX_GPIO_Init() on purpose. Both pins share the EXTI0_1 vector,
  * and an edge arriving before the corresponding driver has registered its
  * bus IO used to call through a NULL function pointer and HardFault the MCU.
  * The sensors keep their configuration across an MCU reset, so a device
  * already asserting INT hits that window on every boot.
  *
  * Call once ACC_Init() and IR_SENSOR_InitCtx() have completed. Until then the
  * pins stay in their reset state (analog, input buffer off) and the EXTI mux,
  * trigger and mask registers are all still at their reset values, so no edge
  * can be detected, latched or delivered.
  */
void GPIO_EnableSensorInterrupts(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Sets the EXTI mux (EXTICR), the rising trigger (RTSR1) and unmasks the
   * lines (IMR1) for both pins in one go. */
  GPIO_InitStruct.Pin = IR_SENS_INT_Pin|_6AX_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* Both sensors drive their INT pin as a level and latch the source until it
   * is read, so a condition asserted before the lines came up produces no
   * rising edge and would never be reported. Raise one in software: SWIER1
   * sets the rising-pending bit, so each sensor is sampled by its normal
   * handler, in interrupt context, exactly as a real edge would. */
  __HAL_GPIO_EXTI_GENERATE_SWIT(IR_SENS_INT_Pin|_6AX_INT_Pin);
}

/* USER CODE END 2 */
