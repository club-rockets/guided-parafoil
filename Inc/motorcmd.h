/*********************************************************************************************
 ___                  _      ___   _____  ___
 |  _`\               ( )    (  _`\(_   _)(  _`\
 | (_) )   _      ___ | |/') | (_(_) | |  | (_(_)
 | ,  /  /'_`\  /'___)| , <  |  _)_  | |  `\__ \
 | |\ \ ( (_) )( (___ | |\`\ | (_( ) | |  ( )_) |
 (_) (_)`\___/'`\____)(_) (_)(____/' (_)  `\____)
 Copyright (c) 2016-2017
 All rights reserved
 RockETS, Montreal
 Ecole de Technologies Superieures
 *********************************************************************************************/

#ifndef MOTORCMD_H_
#define MOTORCMD_H_

#include "string.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_tim.h"

#include "tim.h"
#include "rtc.h"
#include "SD_save.h"

/******************************************************************************/
/*                                Define                                      */
/******************************************************************************/
#define PGAIN             4
#define POS_TOLERANCE     10
#define CMD_STATURATION   12// +/- 12V


/******************************************************************************/
/*                             Function prototype                             */
/******************************************************************************/
void Motor_Init();
void MotorCMD_Loop();
void Set_Direction_Error(int _Direction_Error);

#endif /* MOTORCMD_H_ */
