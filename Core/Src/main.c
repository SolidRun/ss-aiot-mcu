/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "flash.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "stdio.h"
#include "ir_sensor.h"
#include "acc_sensor.h"
#include "ublox.h"
#include "bq25638.h"
#include "rtc.h"
#include "i2c_slave.h"
#include "nmea.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t IR_INT;
bool ACC_INT;
bool MCU_INT;
bool gps_time_synced = false;      // GPS time was successfully synchronized
bool gps_time_sync_request = false; // Request to try GPS time synchronization
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RTC_RESYNC_MS 3600000U   /* re-sync the calendar once an hour */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IR_SENS_INT_Pin){
    	IR_HandleInt();
    }
    if (GPIO_Pin == _6AX_INT_Pin){
    	ACC_HandleInt();
    }

}


// Enable VIN_SOM_EN
void SomEnable(void) {
    //HAL_GPIO_WritePin(SOM_EN_GPIO_Port, SOM_EN_Pin, GPIO_PIN_SET);
    I2C_Slave_Init();

}

// Disable VIN_SOM_EN
void SomDisable(void) {
    HAL_GPIO_WritePin(SOM_EN_GPIO_Port, SOM_EN_Pin, GPIO_PIN_RESET);
}

void somSetInt(void) {
	/* assert: drive the line low */
	HAL_GPIO_WritePin(MCU_INT_GPIO_Port, MCU_INT_Pin, GPIO_PIN_RESET);
	MCU_INT = 1;
}

uint8_t somGetInt(void) {
	return MCU_INT;
}

void somClearINT(void) {
	MCU_INT = 0;
	/* deassert: release the line back to external pull-up */
	HAL_GPIO_WritePin(MCU_INT_GPIO_Port, MCU_INT_Pin, GPIO_PIN_SET);
}

void resetI2C2(void){
	HAL_I2C_DeInit(&hi2c2);
	MX_I2C2_Init();
	SomEnable();
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FLASH_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1500);

  ACC_Init();

  IR_SENSOR_InitCtx();
  IR_SENSOR_StartContinuous(STHS34PF80_ODR_AT_1Hz);

  BQ25638_Init();

  SomEnable();
  HAL_TIM_Base_Start_IT(&htim6);
  UBlox_Init();          /* discard the GNSS backlog */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  static uint16_t led_flag=0;
  static uint32_t last_pump = 0;
  static uint32_t last_sync = 0;

  /* A reset does not invalidate the calendar. Believe the backup domain rather
   * than the RAM flag, so the SOM is not told the time is unverified when the
   * cell has been keeping it right all along. */
  gps_time_synced = rtc_gpsSyncIsValid();
  while (1)
  {
	 led_flag++;
	 if(led_flag>100){
		 HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
		 led_flag = 0;
	 }

	 /* Drain the GNSS on a fixed cadence. Measured output is ~370 B/s with no
	  * fix and two to three times that with one, so a 20 ms gap accumulates a
	  * few dozen bytes at most and the module's buffer never builds up. */
	 if ((HAL_GetTick() - last_pump) >= 20U) {
		 last_pump = HAL_GetTick();
		 UBlox_Pump();
	 }
	 ////////////////////////
	 /* Keep the calendar right from GNSS, with no help from the SOM. RMC gives
	  * a fresh time every second once there is a fix; the RTC only needs it
	  * once, and then once an hour to stay inside the LSE's 20 ppm. */
	 {
		 nmea_time_t t;
		 if ((!gps_time_synced ||
		      (HAL_GetTick() - last_sync) >= RTC_RESYNC_MS) &&
		     NMEA_GetTime(&t)) {
			 rtc_updeteTime(t.hour, t.min, t.sec);
			 rtc_updeteDate(t.month, t.day, t.year);
			 rtc_markGpsSynced();
			 gps_time_synced = true;
			 last_sync = HAL_GetTick();
		 }
	 }

	 /* The SOM's "sync now" command just clears the flag; the block above does
	  * the work on the next RMC. */
	 if (gps_time_sync_request){
		 gps_time_synced = false;
		 gps_time_sync_request = false;
	 }

	 HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int fd, char * ptr, int len)
{
  HAL_UART_Transmit(&huart3, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
