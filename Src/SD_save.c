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

#include "SD_save.h"

/******************************************************************************/
/*                             Global variable                                */
/******************************************************************************/
//FATFS parameters
FATFS fs;         // Work area (file system object) for logical drive
FIL data_file;  // file objects for data logging
FIL error_file;  // file objects for error logging
//FRESULT res;        // FatFs function common result code
uint32_t byteread, bytewritten;         // File R/W count

// buffer pour sauvegarder des donnees
static uint8_t Save_String[512];

void SD_Save_Loop()
{
  static uint32_t save_counter;
  //turn on led if the data as been saved
  if (f_close(&data_file) == FR_OK) {
    HAL_GPIO_WritePin(GPIOD, LED3_Pin, GPIO_PIN_SET);
  }
  //if error while writing, reset led
  else {
    HAL_GPIO_WritePin(GPIOD, LED3_Pin, GPIO_PIN_RESET);
  }

  f_open(&data_file, FILENAME, FA_OPEN_EXISTING | FA_WRITE);
  f_lseek(&data_file, f_size(&data_file));

  save_counter++;
}

void SD_Save_Init()
{
  //init de fatfs et de la SD
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

void SD_Save_Data(uint8_t *_Save_String)
{
  //RTC time and date handler
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  // buffer pour sauvegarder des donnees
  static uint8_t Save_String[512];

  //SD card write
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  /***************************************************
   * SD save in buffer
   ***************************************************/
  sprintf((char*) (Save_String), "20%02d-%02d-%02dT%02d:%02d:%02d,%s\n",
          sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes,
          sTime.Seconds, _Save_String);

  f_puts((char*) Save_String, &data_file);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static int i;
  i++;
  //HAL_GPIO_TogglePin(GPIOD, LED4_Pin);
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

void log_message(char *str){
//  char buff[strlen(str)+3];
//  strcat(buff, str);
//  strcat(buff, "\r\n");
//
//  WriteInFile(buff, strlen(buff), FILE_NAME_DEBUG);
//
//  OSTimeDlyHMSM(0, 0, 0, 1, OS_OPT_TIME_HMSM_STRICT, &err);
}
