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
 this file contain the main program
 every ISR is called here and passes global variable
 *********************************************************************************************/

#include "direction_error_calculator.h"

/******************************************************************************/
/*                             Global variable                                */
/******************************************************************************/
// buffer pour sauvegarder des donnees
static uint8_t Save_String[512];

void Direction_Error_Calculator_Loop() {
  HAL_GPIO_TogglePin(GPIOD, LED2_Pin);

  /***************************************************
   * update real time clock variable before save
   ***************************************************/
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  /***************************************************
   * SD save in buffer
   ***************************************************/
  sprintf((char*) (Save_String), "20%02d-%02d-%02dT%02d:%02d:%02d,%i\n",
          sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes,
          sTime.Seconds, 100);

  SD_Save_Data(Save_String);
}
