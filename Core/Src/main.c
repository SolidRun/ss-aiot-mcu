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
#include "lptim.h"
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
#include "timebase.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern volatile bool sensors_ready;
volatile uint8_t IR_INT;
volatile uint8_t ACC_INT;
volatile uint8_t MCU_INT;
volatile uint8_t RTC_INT;
volatile bool gps_time_synced = false;      // GPS time was successfully synchronized
volatile bool gps_time_sync_request = false; // Request to try GPS time synchronization
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

/* SoM power-off internal state, accessed from i2c irq and main */
static volatile bool     som_off_pending  = false;
static volatile uint32_t som_off_deadline = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if (!sensors_ready) { return; }

    if (GPIO_Pin == IR_SENS_INT_Pin){
    	IR_HandleInt();
    }
    if (GPIO_Pin == _6AX_INT_Pin){
    	ACC_HandleInt();
    }

}

// Enable VIN_SOM_EN
void SomEnable(void) {
    /* regulator enable pin is active-low, low = on */
    HAL_GPIO_WritePin(SOM_EN_GPIO_Port, SOM_EN_Pin, GPIO_PIN_RESET);
}

// Disable VIN_SOM_EN
void SomDisable(void) {
    /* regulator enable pin is active-low, high = off */
    HAL_GPIO_WritePin(SOM_EN_GPIO_Port, SOM_EN_Pin, GPIO_PIN_SET);
}

/*
 * Delayed power-off for SoM:
 *
 * Pass delay in ms, cannot be canceled, processed in main.
 */
void SomScheduleOff(uint16_t delay_ms) {
  som_off_deadline = HAL_GetTick() + delay_ms;
  som_off_pending  = true;
}


/* Interrupt reporting configuration, in the wire order of 0x13 0x07.
 *
 * en_sources gates whole sources and pwr_sources selects which of the sources
 * that got through power the SOM up, so the two source masks sit together;
 * en_ir/en_acc/en_rtc gate the individual events within a source, in the order
 * of the detail bytes of the interrupt status read.
 *
 * The defaults report everything and reproduce the power behaviour the
 * firmware had before the command existed: a sensor or alarm event brings the
 * SOM up, the firmware's own restart notification does not.
 */
static volatile struct {
	uint8_t en_sources;
	uint8_t pwr_sources;
	uint8_t en_ir;
	uint8_t en_acc;
	uint8_t en_rtc;
} int_cfg = {
	0xFFU,
	INT_SRC_IR | INT_SRC_ACCEL | INT_SRC_RTC,
	0xFFU, 0xFFU, 0xFFU,
};

/* Report the interrupt configuration, INT_CONFIG_LEN bytes in wire order.
 */
void somGetIntConfig(uint8_t *out)
{
	out[0] = int_cfg.en_sources;
	out[1] = int_cfg.pwr_sources;
	out[2] = int_cfg.en_ir;
	out[3] = int_cfg.en_acc;
	out[4] = int_cfg.en_rtc;
}

/* Replace the interrupt configuration, INT_CONFIG_LEN bytes in wire order.
 *
 * Applied as one indivisible operation, so an event in flight is judged
 * against either the old configuration or the new one, never a mix.
 */
void somSetIntConfig(const uint8_t *in)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();

	int_cfg.en_sources  = in[0];
	int_cfg.pwr_sources = in[1];
	int_cfg.en_ir       = in[2];
	int_cfg.en_acc      = in[3];
	int_cfg.en_rtc      = in[4];

	__set_PRIMASK(primask);
}

/* Record an interrupt source together with its detail bits, and assert the
 * line to the SOM. The source bit and its detail byte are set as one
 * indivisible operation, so a read of the interrupt status never sees one
 * without the other.
 *
 * A zero detail byte means there is nothing to report and the call does
 * nothing. INT_SRC_MCU carries no detail byte in the response, so it passes
 * any non-zero value to raise the source on its own.
 */
void somSetInt(uint8_t source, uint8_t detail)
{
	uint32_t primask;

	/* keep only the events this source is configured to report */
	switch (source) {
	case INT_SRC_IR:    detail &= int_cfg.en_ir;  break;
	case INT_SRC_ACCEL: detail &= int_cfg.en_acc; break;
	case INT_SRC_RTC:   detail &= int_cfg.en_rtc; break;
	default: break;      /* INT_SRC_MCU has no events to select from */
	}

	/* nothing to report */
	if (detail == 0)
		return;

	/* the source itself may be switched off */
	if ((source & int_cfg.en_sources) == 0U)
		return;

	/* the SOM has to be powered to receive what it asked to be woken for */
	if ((source & int_cfg.pwr_sources) != 0U)
		SomEnable();

	primask = __get_PRIMASK();

	__disable_irq();

	MCU_INT |= source;

	switch (source) {
	case INT_SRC_IR:    IR_INT  |= detail; break;
	case INT_SRC_ACCEL: ACC_INT |= detail; break;
	case INT_SRC_RTC:   RTC_INT |= detail; break;
	default: break;      /* INT_SRC_MCU, the source bit is the whole report */
	}

	/* assert: drive the line low */
	HAL_GPIO_WritePin(MCU_INT_GPIO_Port, MCU_INT_Pin, GPIO_PIN_RESET);

	__set_PRIMASK(primask);
}

/* Snapshot and clear all three interrupt latches, and release the line to the
 * SOM, as one indivisible operation.
 */
void somTakeInterrupts(uint8_t *mcu, uint8_t *ir, uint8_t *acc, uint8_t *rtc)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();

	*mcu = (uint8_t)MCU_INT;
	*ir  = (uint8_t)IR_INT;
	*acc = (uint8_t)ACC_INT;
	*rtc = (uint8_t)RTC_INT;

	MCU_INT = 0;
	IR_INT  = 0;
	ACC_INT = 0;
	RTC_INT = 0;

	/* deassert: release the line back to the external pull-up */
	HAL_GPIO_WritePin(MCU_INT_GPIO_Port, MCU_INT_Pin, GPIO_PIN_SET);

	__set_PRIMASK(primask);
}

void resetI2C2(void){
	HAL_I2C_DeInit(&hi2c2);
	MX_I2C2_Init();
	/* re-arm slave after deinit&init */
	I2C_Slave_Init();
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
  MX_LPTIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Start the tick counter before anything that timestamps with it. */
  Timebase_Init();

  /* change som-enable pin from input to output following initial electrical state */
  GPIO_InitSomEnablePin();

  /* Alarm A can be armed from a previous power cycle - by this firmware, or by
   * the CubeMX block in MX_RTC_Init() on the board's very first boot. Sort out
   * which before anything can raise an interrupt. */
  rtc_alarmInit();

  HAL_Delay(1500);

  ACC_Init();

  IR_SENSOR_InitCtx();
  IR_SENSOR_StartContinuous(STHS34PF80_ODR_AT_1Hz);

  BQ25638_Init();

  /* Both sensor drivers are up, so it is now safe to bring up the interrupt
   * lines they share. Doing this here rather than in MX_GPIO_Init() is what
   * keeps an edge from reaching a driver that has no bus IO registered yet. */
  GPIO_EnableSensorInterrupts();

  /* Arm the i2c slave */
  I2C_Slave_Init();

  // mcu (re-)start should notify SoM, but not modify current power-state
  somSetInt(INT_SRC_MCU, 0x01U);
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

	 /* Per-driver high-priority interrupt post-processing. A driver handles its
	  * interrupt on the mcu side and returns the part of it, if any, that the
	  * SOM is to be notified about. */
	 {
		 uint8_t detail;

		 ACC_ProcessInt(&detail);
		 somSetInt(INT_SRC_ACCEL, detail);

		 IR_ProcessInt(&detail);
		 somSetInt(INT_SRC_IR, detail);
	 }

	 /* Per-driver processing, once per iteration. Each decides for itself whether there is work to do. */
	 ACC_Process();
	 IR_Process();
	 BQ25638_Process();

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

    /* process scheduled SoM power-off */
    if (som_off_pending) {
      /* distance in ms between current time and deadline: negative = deadline in future, positive = deadline in past */
      int32_t distance = HAL_GetTick() - som_off_deadline;
      if (distance >= 0) {
        som_off_pending = false;
        SomDisable();
      }
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
