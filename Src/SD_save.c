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
FRESULT res;        // FatFs function common result code
uint32_t byteread, bytewritten;         // File R/W count



void SD_Save_Loop()
{
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

  //save user setting to Backup SRAM
  //this ram sector wont erase upon power cycling
  Save_Backup_SRAM(&Backup_Settings);

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static int i;
  i++;
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
