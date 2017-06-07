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
int MotorLeft_PosCmd, MotorRight_PosCmd = 0; // Motor position command
float MotorLeftPos = 0.0, MotorRightPos = 0.0;          // Motor position
uint8_t PWMLeftSet, PWMRightSet = 0;        // Bool PWM ON/OFF
uint8_t MotorEnabled = 0;

/******************************************************************************/
/*                             Function prototype                             */
/******************************************************************************/
float SaturateCMD(float _command);

void Motor_Init(){

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);

  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  //HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  //HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  PWMLeftSet = 0;
  PWMRightSet = 0;

  MotorLeftPos = 0.0;
  MotorRightPos = 0.0;

  MotorLeft_PosCmd = 0;
  MotorRight_PosCmd = 0;

  MotorEnabled = 0;

  __HAL_TIM_SetCounter(&htim2, 25000);
  __HAL_TIM_SetCounter(&htim5, 25000);

  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);

  //Enable voltage translator
  HAL_GPIO_WritePin(TRANS_OE_GPIO_Port, TRANS_OE_Pin, GPIO_PIN_SET);
}

void MotorCMD_Loop() {

  float MotorLeftSpeed, MotorRightSpeed;
  int LeftCnt = 0, RightCnt = 0;

  /*  Motor driver  */
  static uint32_t led_counter;
  static float previous_MotorLeft_SpeedErr, previous_MotorRight_SpeedErr;  //Previous speed error
  static float previous_MotorLeft_Cmd, previous_MotorRight_Cmd;  //Previous motor command

  float MotorLeft_PosErr, MotorRight_PosErr;     //Axial positionning error
  float MotorLeft_SpeedCmd, MotorRight_SpeedCmd;  //Speed command
  float MotorLeft_SpeedErr, MotorRight_SpeedErr;    //Axial speed error
  float MotorLeft_Cmd, MotorRight_Cmd;              //Motor command
  int MotorLeft_PWM, MotorRight_PWM;              //Motor command

  /* SD card  */
  static uint8_t Save_String[512];                  //SD card write buffer

  /* Encoder read */
  LeftCnt = __HAL_TIM_GET_COUNTER(&htim2) - 25000;
  __HAL_TIM_SetCounter(&htim2, 25000);

  MotorLeftPos += LeftCnt / 41.0;//2 pi (rad/s)/256 (pulse) = 1/41
  MotorLeftSpeed = LeftCnt / 2.0;// 2 pi (rad/s) / (256 (pulse))

  RightCnt = __HAL_TIM_GET_COUNTER(&htim5) - 25000;
  __HAL_TIM_SetCounter(&htim5, 25000);

  MotorRightPos += RightCnt / 41.0;//2 pi (rad/s)/256 (pulse) = 1/41
  MotorRightSpeed = RightCnt / 2.0;// 2 pi (rad/s) / (256 (pulse))


  /*  Motor command */
  //if (MotorEnabled != 0)
  //{

    //Calculation of the motor axial position error
    MotorLeft_PosErr = MotorLeft_PosCmd - MotorLeftPos;
    MotorRight_PosErr = MotorRight_PosCmd - MotorRightPos;

    //Calculation of the speed command
    MotorLeft_SpeedCmd = 1.374 * MotorLeft_PosErr;
    MotorRight_SpeedCmd = 1.374 * MotorRight_PosErr;

    //Calculation of the axial speed error
    MotorLeft_SpeedErr = MotorLeft_SpeedCmd - MotorLeftSpeed;
    MotorRight_SpeedErr = MotorRight_SpeedCmd - MotorRightSpeed;

    //Calculation of the motor command
    MotorLeft_Cmd = SaturateCMD(0.002 * MotorLeft_SpeedErr
        + 0.005 * previous_MotorLeft_SpeedErr + previous_MotorLeft_Cmd);//0.001941
    MotorRight_Cmd = SaturateCMD(0.002 * MotorRight_SpeedErr
        + 0.005 * previous_MotorRight_SpeedErr + previous_MotorRight_Cmd);

     MotorLeft_PWM = -341 * MotorLeft_Cmd + 4200;//(8300 - 4200)/(12 - 0) = 341
     MotorRight_PWM = 341 * MotorRight_Cmd + 4200;//(8300 - 4200)/(12 - 0) = 341

     //Left motor PWM set
     if ((MotorLeft_PosErr < POS_TOLERANCE)&&(MotorLeft_PosErr > -POS_TOLERANCE))
     {
       if (PWMLeftSet != 0)
       {
         //Deactivate PWM
         HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
         HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
         PWMLeftSet = 0;
       }
     }
     else
     {
       if (PWMLeftSet == 0)
       {
         HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
         HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
         PWMLeftSet = 1;
       }
     }
     //Set PWM
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MotorLeft_PWM);

     //Right motor PWM set
     if ((MotorRight_PosErr < POS_TOLERANCE)&&(MotorRight_PosErr > -POS_TOLERANCE))
        {
          if (PWMRightSet != 0)
          {
            //Deactivate PWM
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            PWMRightSet = 0;
          }
        }
        else
        {
          if (PWMRightSet == 0)
          {
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            PWMRightSet = 1;
          }
        }
        //Set PWM
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MotorRight_PWM);

    //Set previous value for next command calculation
    previous_MotorLeft_SpeedErr = MotorLeft_SpeedErr;
    previous_MotorRight_SpeedErr = MotorRight_SpeedErr;
    previous_MotorLeft_Cmd = MotorLeft_Cmd;
    previous_MotorRight_Cmd = MotorRight_Cmd;
  //}

  /***************************************************
   * SD save in buffer
   ***************************************************/
  sprintf((char*) (Save_String), "%s,%f,%f,%f,%f", "SGP_MD",MotorLeft_Cmd, MotorRight_Cmd, MotorLeftPos, MotorRightPos);

  SD_Save_Data(Save_String);

  if (led_counter < 19) {
    led_counter++;
  } else {
    led_counter = 0;
    HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
  }


}

void Enable_MotorCMD()
{
  MotorEnabled = 1;
}

void Disable_MotorCMD()
{
  MotorEnabled = 0;
}

int Get_LeftMotor_command() {
  return MotorLeft_PosCmd;
}

int Get_RightMotor_command() {
  return MotorRight_PosCmd;
}

float Get_LeftMotor_position() {
  return MotorLeftPos;
}

float Get_RightMotor_position() {
  return MotorRightPos;
}

void Set_Motor_Command(int _MotorLeft_PosCmd, int _MotorRight_PosCmd) {
  //
  MotorLeft_PosCmd = -_MotorLeft_PosCmd;
  MotorRight_PosCmd = -_MotorRight_PosCmd;
}

void config_Motor_Command(int _MotorLeft_ConfCmd, int _MotorRight_ConfCmd) {
  //
  MotorLeft_PosCmd += _MotorLeft_ConfCmd;
  MotorRight_PosCmd += _MotorRight_ConfCmd;
}

void MotorPos_Reset()
{
  MotorLeftPos = 0;
  MotorRightPos = 0;
  MotorLeft_PosCmd = 0;
  MotorRight_PosCmd = 0;
}

float SaturateCMD(float _command)
{
  if (_command > CMD_STATURATION)
   {
     return CMD_STATURATION;
   }
  else if (_command < -CMD_STATURATION)
   {
     return -CMD_STATURATION;
   }
  return _command;
}
