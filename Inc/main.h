/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define NOT_USED_PE2_Pin GPIO_PIN_2
#define NOT_USED_PE2_GPIO_Port GPIOE
#define NOT_USED_PE3_Pin GPIO_PIN_3
#define NOT_USED_PE3_GPIO_Port GPIOE
#define NOT_USED_PE4_Pin GPIO_PIN_4
#define NOT_USED_PE4_GPIO_Port GPIOE
#define NOT_USED_PE5_Pin GPIO_PIN_5
#define NOT_USED_PE5_GPIO_Port GPIOE
#define NOT_USED_PE6_Pin GPIO_PIN_6
#define NOT_USED_PE6_GPIO_Port GPIOE
#define NOT_USED_PC13_Pin GPIO_PIN_13
#define NOT_USED_PC13_GPIO_Port GPIOC
#define NOT_USED_PC0_Pin GPIO_PIN_0
#define NOT_USED_PC0_GPIO_Port GPIOC
#define NOT_USED_PC1_Pin GPIO_PIN_1
#define NOT_USED_PC1_GPIO_Port GPIOC
#define NOT_USED_PC2_Pin GPIO_PIN_2
#define NOT_USED_PC2_GPIO_Port GPIOC
#define NOT_USED_PC3_Pin GPIO_PIN_3
#define NOT_USED_PC3_GPIO_Port GPIOC
#define RIGHT_CHA_Pin GPIO_PIN_0
#define RIGHT_CHA_GPIO_Port GPIOA
#define RIGHT_CHB_Pin GPIO_PIN_1
#define RIGHT_CHB_GPIO_Port GPIOA
#define NOT_USED_PA2_Pin GPIO_PIN_2
#define NOT_USED_PA2_GPIO_Port GPIOA
#define NOT_USED_PA3_Pin GPIO_PIN_3
#define NOT_USED_PA3_GPIO_Port GPIOA
#define MTi_CS_Pin GPIO_PIN_4
#define MTi_CS_GPIO_Port GPIOA
#define MTi_SCK_Pin GPIO_PIN_5
#define MTi_SCK_GPIO_Port GPIOA
#define MTi_MISO_Pin GPIO_PIN_6
#define MTi_MISO_GPIO_Port GPIOA
#define MTi_MOSI_Pin GPIO_PIN_7
#define MTi_MOSI_GPIO_Port GPIOA
#define MTi_SYNC_IN_Pin GPIO_PIN_5
#define MTi_SYNC_IN_GPIO_Port GPIOC
#define NPWM_LEFT_Pin GPIO_PIN_0
#define NPWM_LEFT_GPIO_Port GPIOB
#define NPWM_RIGHT_Pin GPIO_PIN_1
#define NPWM_RIGHT_GPIO_Port GPIOB
#define NOT_USED_PB2_Pin GPIO_PIN_2
#define NOT_USED_PB2_GPIO_Port GPIOB
#define NOT_USED_PE7_Pin GPIO_PIN_7
#define NOT_USED_PE7_GPIO_Port GPIOE
#define MTi_RST_Pin GPIO_PIN_8
#define MTi_RST_GPIO_Port GPIOE
#define NOT_USED_PE9_Pin GPIO_PIN_9
#define NOT_USED_PE9_GPIO_Port GPIOE
#define NOT_USED_PE10_Pin GPIO_PIN_10
#define NOT_USED_PE10_GPIO_Port GPIOE
#define PWM_LEFT_Pin GPIO_PIN_11
#define PWM_LEFT_GPIO_Port GPIOE
#define NOT_USED_PE12_Pin GPIO_PIN_12
#define NOT_USED_PE12_GPIO_Port GPIOE
#define PWM_RIGHT_Pin GPIO_PIN_13
#define PWM_RIGHT_GPIO_Port GPIOE
#define NOT_USED_PE14_Pin GPIO_PIN_14
#define NOT_USED_PE14_GPIO_Port GPIOE
#define NOT_USED_PE15_Pin GPIO_PIN_15
#define NOT_USED_PE15_GPIO_Port GPIOE
#define NOT_USED_PB10_Pin GPIO_PIN_10
#define NOT_USED_PB10_GPIO_Port GPIOB
#define CAN_STANDBY_Pin GPIO_PIN_11
#define CAN_STANDBY_GPIO_Port GPIOB
#define NOT_USED_PB14_Pin GPIO_PIN_14
#define NOT_USED_PB14_GPIO_Port GPIOB
#define NOT_USED_PB15_Pin GPIO_PIN_15
#define NOT_USED_PB15_GPIO_Port GPIOB
#define GPIO_ext1_Pin GPIO_PIN_8
#define GPIO_ext1_GPIO_Port GPIOD
#define GPIO_ext2_Pin GPIO_PIN_9
#define GPIO_ext2_GPIO_Port GPIOD
#define GPIO_ext3_Pin GPIO_PIN_10
#define GPIO_ext3_GPIO_Port GPIOD
#define GPIO_ext4_Pin GPIO_PIN_11
#define GPIO_ext4_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOD
#define LEFT_CHB_Pin GPIO_PIN_15
#define LEFT_CHB_GPIO_Port GPIOA
#define NOT_USED_PD0_Pin GPIO_PIN_0
#define NOT_USED_PD0_GPIO_Port GPIOD
#define GPS_D_SEL_Pin GPIO_PIN_1
#define GPS_D_SEL_GPIO_Port GPIOD
#define SD_DETECT_Pin GPIO_PIN_3
#define SD_DETECT_GPIO_Port GPIOD
#define GPS_INT_Pin GPIO_PIN_4
#define GPS_INT_GPIO_Port GPIOD
#define GPS_RX_Pin GPIO_PIN_5
#define GPS_RX_GPIO_Port GPIOD
#define GPS_TX_Pin GPIO_PIN_6
#define GPS_TX_GPIO_Port GPIOD
#define GPS_PULSE_Pin GPIO_PIN_7
#define GPS_PULSE_GPIO_Port GPIOD
#define LEFT_CHA_Pin GPIO_PIN_3
#define LEFT_CHA_GPIO_Port GPIOB
#define NOT_USED_PB4_Pin GPIO_PIN_4
#define NOT_USED_PB4_GPIO_Port GPIOB
#define NOT_USED_PB5_Pin GPIO_PIN_5
#define NOT_USED_PB5_GPIO_Port GPIOB
#define NOT_USED_PB6_Pin GPIO_PIN_6
#define NOT_USED_PB6_GPIO_Port GPIOB
#define NOT_USED_PB7_Pin GPIO_PIN_7
#define NOT_USED_PB7_GPIO_Port GPIOB
#define NOT_USED_PB8_Pin GPIO_PIN_8
#define NOT_USED_PB8_GPIO_Port GPIOB
#define GPS_RESET_Pin GPIO_PIN_9
#define GPS_RESET_GPIO_Port GPIOB
#define NOT_USED_PE0_Pin GPIO_PIN_0
#define NOT_USED_PE0_GPIO_Port GPIOE
#define TRANS_OE_Pin GPIO_PIN_1
#define TRANS_OE_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#define GPS_FRAME_LENGTH 100

typedef enum Rocket_State_m {
  INITIALISATION = 0,
  STANDBY_ON_PAD,
  LAUNCH,
  POWERED_ASCENT,
  ENGINE_BURNOUT,
  COASTING_ASCENT,
  APOGEE_REACHED,
  DROGUE_DEPLOYMENT,
  DROGUE_DESCENT,
  MAIN_DEPLOYMENT,
  MAIN_DESCENT,
  LANDING,
  RECOVERY,
  PICKEDUP
} Rocket_State_t;

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
