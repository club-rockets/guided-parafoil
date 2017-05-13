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

#include "motorcmd.h"

/******************************************************************************/
/*                             Global variable                                */
/******************************************************************************/
uint16_t MotorLeft_PosCmd, MotorRight_PosCmd;

/******************************************************************************/
/*                             Function prototype                             */
/******************************************************************************/
void Set_MotorLeftPWM(uint16_t _MotorLeftPWM);
void Set_MotorRightPWM(uint16_t _MotorRightPWM);

uint16_t Get_MotorLeftPosition();
uint16_t Get_MotorRightPosition();
uint16_t Get_MotorLeftSpeed();
uint16_t Get_MotorRightSpeed();

void MotorCMD_Loop() {
  static uint32_t led_counter;
  static float previous_MotorLeft_SpeedErr, previous_MotorRight_SpeedErr;  //Previous speed error
  static float previous_MotorLeft_Cmd, previous_MotorRight_Cmd;  //Previous motor command

  uint16_t MotorLeft_PosErr, MotorRight_PosErr;     //Axial positionning error
  uint16_t MotorLeft_SpeedCmd, MotorRight_SpeedCmd;  //Speed command
  float MotorLeft_SpeedErr, MotorRight_SpeedErr;    //Axial speed error
  float MotorLeft_Cmd, MotorRight_Cmd;              //Motor command

  //Calculation of the motor axial position error
  MotorLeft_PosErr = MotorLeft_PosCmd - Get_MotorLeftPosition();
  MotorRight_PosErr = MotorRight_PosCmd - Get_MotorRightPosition();

  //Calculation of the speed command
  MotorLeft_SpeedCmd = 1.374 * MotorLeft_PosErr;
  MotorRight_SpeedCmd = 1.374 * MotorRight_PosErr;

  //Calculation of the axial speed error
  MotorLeft_SpeedErr = MotorLeft_SpeedCmd - Get_MotorLeftSpeed();
  MotorRight_SpeedErr = MotorRight_SpeedCmd - Get_MotorRightSpeed();

  //Calculation of the motor command
  MotorLeft_Cmd = 0.001941 * MotorLeft_SpeedErr
      + 0.001941 * previous_MotorLeft_SpeedErr + previous_MotorLeft_Cmd;
  MotorRight_Cmd = 0.001941 * MotorRight_SpeedErr
      + 0.001941 * previous_MotorRight_SpeedErr + previous_MotorRight_Cmd;

  //Set previous value for next command calculation
  previous_MotorLeft_SpeedErr = MotorLeft_SpeedErr;
  previous_MotorRight_SpeedErr = MotorRight_SpeedErr;
  previous_MotorLeft_Cmd = MotorLeft_Cmd;
  previous_MotorRight_Cmd = MotorRight_Cmd;

  if (led_counter < 20) {
    led_counter++;
  } else {
    led_counter = 0;
    HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
  }

}

void Set_Direction_Error(uint16_t _Direction_Error) {
  if (_Direction_Error > 0) {
    MotorLeft_PosCmd = _Direction_Error + 300;
    MotorRight_PosCmd = 300;
  } else {
    MotorLeft_PosCmd = 300;
    MotorRight_PosCmd = -_Direction_Error + 300;
  }
}

void Set_MotorLeftPWM(uint16_t _MotorLeftPWM) {
  if (_MotorLeftPWM > 7900)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 7900);
  else if (_MotorLeftPWM < 0)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  else
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, _MotorLeftPWM);
}

void Set_MotorRightPWM(uint16_t _MotorRightPWM) {
  if (_MotorRightPWM > 7900)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 7900);
  else if (_MotorRightPWM < 0)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  else
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, _MotorRightPWM);
}

uint16_t Get_MotorLeftPosition() {
  return 0;
}

uint16_t Get_MotorRightPosition() {
  return 0;
}

uint16_t Get_MotorLeftSpeed() {
  return 0;
}

uint16_t Get_MotorRightSpeed() {
  return 0;
}
