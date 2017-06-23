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
#include "can.h"

/******************************************************************************/
/*                                Define                                      */
/******************************************************************************/
#define PGAIN             4
#define POS_TOLERANCE     10// +/- 10LSB
#define CMD_STATURATION   12// +/- 12V

#define LEFT_CABLE_LENGTH  61
#define RIGHT_CABLE_LENGTH 63
#define CM_TO_RAD          140


/******************************************************************************/
/*                             Function prototype                             */
/******************************************************************************/
void Motor_Init();
void MotorCMD_Loop();
int Get_LeftMotor_command();
int Get_RightMotor_command();
float Get_LeftMotor_position();
float Get_RightMotor_position();
void Set_Motor_Command(int _MotorLeft_PosCmd, int _MotorRight_PosCmd);
void config_Motor_Command(int _MotorLeft_ConfCmd, int _MotorRight_ConfCmd);
void MotorPos_Reset();
void Enable_MotorCMD();
void Disable_MotorCMD();
void CalibrateMotor();

#endif /* MOTORCMD_H_ */
