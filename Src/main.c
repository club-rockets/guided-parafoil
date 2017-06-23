/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "fatfs.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "SGP_Control.h"
#include "motorcmd.h"
#include "SD_save.h"
#include "GPS.h"
#include "serial_com.h"
#include "mti.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//temp
uint8_t temp_val = 0;
// buffer pour comm CANbus
CanTxMsgTypeDef CanTx_msg;
CanRxMsgTypeDef CanRx_msg;
CAN_FilterConfTypeDef CAN_FilterStruct;

extern FATFS fs;         // Work area (file system object) for logical drive
extern FIL data_file;  // file objects for data logging
extern FIL error_file;  // file objects for error logging
//FRESULT res;        // FatFs function common result code
extern uint32_t byteread, bytewritten;         // File R/W count

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
HAL_StatusTypeDef result;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN2_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART6_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */

	hcan2.pTxMsg = &CanTx_msg;
	hcan2.pRxMsg = &CanRx_msg;

	CAN_FilterStruct.FilterNumber = 14; //Specifies the filter which will be initialized.
	CAN_FilterStruct.FilterMode = CAN_FILTERMODE_IDMASK; //Specifies the filter mode to be initialized.
														 //CAN_FILTERMODE_IDLIST : Identifier list mode
	CAN_FilterStruct.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterStruct.FilterIdHigh = 0; //Specifies the filter identification number (MSBs for a 32-bit configuration, first one for a 16-bit configuration).
	CAN_FilterStruct.FilterIdLow = 0;
	CAN_FilterStruct.FilterMaskIdHigh = 0;
	CAN_FilterStruct.FilterMaskIdLow = 0;
	CAN_FilterStruct.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_FilterStruct.FilterActivation = ENABLE; /* Enable this filter */
	CAN_FilterStruct.BankNumber = 14;

	HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterStruct);

	hcan2.pTxMsg->StdId = 0x244;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->DLC = 1;

	//init main handlers and stuff
	SD_Save_Init();
	Motor_Init();
	GPS_Init();
	SGP_Control_Init();

	//MTi release reset
	HAL_GPIO_WritePin(MTi_RST_GPIO_Port, MTi_RST_Pin, GPIO_PIN_RESET);

	HAL_Delay(1000);

	//start uart interrupt for GPS, cte lenght rx

	//TIMER START
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim6);

	//MTi release reset
	HAL_GPIO_WritePin(MTi_RST_GPIO_Port, MTi_RST_Pin, GPIO_PIN_SET);

	//CAN START
	HAL_GPIO_WritePin(CAN_STANDBY_GPIO_Port, CAN_STANDBY_Pin, GPIO_PIN_RESET);
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);

	PolarCoordinate_t polar_dest;

	//Landing zone Spaceport
	polar_dest.latitude = 32.947222;
	polar_dest.longitude = -106.915226;

	Set_GPSDestination(polar_dest);

	vector_2D_t uWindVector;

	uWindVector.X = 1.0;
	uWindVector.Y = 0.0;

	Set_uWindVector(uWindVector);

	//TODO: IMPORTANT TO REMOVE THAT LINE!
	//Set_RocketState(MAIN_DESCENT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		serial_menu();


	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* CAN2_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* CAN2_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* OTG_FS_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/* USER CODE BEGIN 4 */

/* All peripheral callback are here */
//TIMER
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	static int tim3Counter = 1;
	static float altitude = 900.0;

	if (htim->Instance == TIM3) {

		MotorCMD_Loop();

		if (!(tim3Counter % 4)) {
			GPS_Read_Data();
		}

		if (tim3Counter == 19) {
			CANBUS_LaunchDataRead();
		}

		if (tim3Counter == 20) {
			SGP_Control_Loop();

			//TODO: To remove
			Set_Altitude(altitude);

			altitude += 30.0;
		}

		tim3Counter++;
		if (tim3Counter > 20)
			tim3Counter = 1;
	}

//  if (htim->Instance == TIM4)
//  {
//    SGP_Control_Loop();
//  }

	if (htim->Instance == TIM6) {
		//SD_Save_Loop();

	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == SD_DETECT_Pin) {
	  uint8_t Save_String[512];


	  //recall all init process
	  //pretty slow, but works quite well

	  if (BSP_SD_Init() == 0) {
		f_mount(&fs, (TCHAR const*) SD_Path, 1);
		f_open(&data_file, FILENAME, FA_OPEN_ALWAYS | FA_WRITE);
		f_lseek(&data_file, f_size(&data_file));

		//Log file header
		strcpy((char*) Save_String, DATA_LOG_HEADER);
		f_puts((char*) Save_String, &data_file);

		//column title
		strcpy((char*) Save_String, DATA_LOG_COL_NAME);
		f_puts((char*) Save_String, &data_file);

		//write file to sd
		f_close(&data_file);
	  }
	}
	//else if (GPIO_Pin == GPIO_PIN_4) {
	else if (0) {
		//HAL_GPIO_TogglePin(GPIOD, LED4_Pin);
		mti_receive_message();
	}

}

/* peripheral callback END */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
