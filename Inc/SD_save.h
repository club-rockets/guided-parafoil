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

#ifndef SD_SAVE_H_
#define SD_SAVE_H_

#include "string.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#include "fatfs.h"
#include "sdio.h"
#include "rtc.h"

/******************************************************************************/
/*                                Define                                      */
/******************************************************************************/
#define FILENAME "SGP.csv"//Must be less than 8 characters!!!
#define DATA_LOG_COL_NAME "RTC Time,Mission Time (ms)\n"
#define DATA_LOG_HEADER "\nRockETS Data Logger Version 2.0\n"

/******************************************************************************/
/*                             Function prototype                             */
/******************************************************************************/
void SD_Save_Loop();
void SD_Save_Data(char *_Save_String);
void SD_Save_Init();
void log_message(char *str);

#endif /* SD_SAVE_H_ */
