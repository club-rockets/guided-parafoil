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

uint16_t test_var = 0;

void Direction_Error_Calculator_Loop() {
  // buffer pour sauvegarder des donnees
  uint8_t Save_String[512];
  static uint32_t led_counter;

  HAL_GPIO_TogglePin(GPIOD, LED2_Pin);

  if (led_counter < 7) {
    led_counter++;
  } else {
    led_counter = 0;
    if (test_var != 0)
    {
      Set_Direction_Error(-1400);
      test_var = 0;
    }
    else
    {
      Set_Direction_Error(1400);
      test_var = 1;
    }
  }

  /***************************************************
   * SD save in buffer
   ***************************************************/
  sprintf((char*) (Save_String), "%s,%i", "SGP_Control", 100);

  SD_Save_Data(Save_String);
}
