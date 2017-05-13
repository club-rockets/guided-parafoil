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

#ifndef DIRECTION_ERROR_CALCULATOR_H_
#define DIRECTION_ERROR_CALCULATOR_H_

#include "string.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#include "rtc.h"
#include "tim.h"
#include "SD_save.h"

/******************************************************************************/
/*                             Global variable                                */
/******************************************************************************/
//RTC time and date handler
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
RTC_AlarmTypeDef sAlarm;

/******************************************************************************/
/*                             Function prototype                             */
/******************************************************************************/
void Direction_Error_Calculator_Loop();

#endif /* DIRECTION_ERROR_CALCULATOR_H_ */
