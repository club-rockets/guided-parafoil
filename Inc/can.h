/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "SGP_Control.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

//sent by SGP
#define CAN_ROCKET_STATE_REQ_ID      0x001
#define CAN_ROCKET_ALTITUDE_REQ_ID   0x002

//sent to SGP
#define CAN_ROCKET_STATE_ID       	0x011
#define CAN_ROCKET_ALTITUDE_ID      0x012

#define CAN_GPS_LONGITUDE_ID      	0x211
#define CAN_GPS_LATITUDE_ID       	0x212
#define CAN_GPS_ALTITUDE_ID      	0x213
#define CAN_GPS_FIX_TYPE_ID       	0x214
#define CAN_GPS_N_SATELLITE_ID    	0x215

#define CAN_ACCELERATION_X_ID 	  	0x221
#define CAN_ACCELERATION_Y_ID     	0x222
#define CAN_ACCELERATION_Z_ID     	0x223

#define CAN_GYRO_YIELD_ID         	0x231
#define CAN_GYRO_YAW_ID           	0x232
#define CAN_GYRO_ROLL_ID          	0x233

#define CAN_SGP_STATE				0x300
#define CAN_SGP_DESCENTTIME			0x301
#define CAN_SGP_HORZSPEED			0x302
#define CAN_SGP_VERTSPEED			0x303
#define CAN_SGP_POSTTRACKING		0x304

#define CAN_BEZIER_TIP_X			0x310
#define CAN_BEZIER_TIP_Y			0x311
#define CAN_BEZIER_FAP_X			0x312
#define CAN_BEZIER_FAP_Y			0x313
#define CAN_BEZIER_CP1_X			0x314
#define CAN_BEZIER_CP1_Y			0x315
#define CAN_BEZIER_CP2_X			0x316
#define CAN_BEZIER_CP2_Y			0x317

#define CAN_MOTORCMD_LEFT			0x400
#define CAN_MOTORCMD_RIGHT			0x401
#define CAN_MOTORPOS_LEFT			0x402
#define CAN_MOTORPOS_RIGHT			0x403


/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef CANBUS_LaunchDataRead(void);
HAL_StatusTypeDef Send_CAN_Message(uint8_t _data[8], uint32_t _StdId);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
