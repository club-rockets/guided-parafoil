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

/******************************************************************************/
/*                                Define                                      */
/******************************************************************************/
#define FILENAME "RockETS_SGP.csv"
#define DATA_LOG_COL_NAME "RTC Time,Rocket State,Mission Time (ms), Main Loop Use (%),Main Detect,Main Fire,Drogue Detect,Drogue Fire,Barometer Temperature (degC),Air Density (Kg/m^3),Barometer Altitude (m),AGL Altitude (m),Estimated Altitude (m),Estimated Vertical Speed (m/s),Estimated Vertical Acceleration (m/s/s)\n"
#define DATA_LOG_HEADER "\nRockETS Data Logger Version 2.0\n"

/******************************************************************************/
/*                             Function prototype                             */
/******************************************************************************/
void SD_Save_Loop();

#endif /* SD_SAVE_H_ */
