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

#ifndef SGP_CONTROL_H_
#define SGP_CONTROL_H_

#include "string.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stdlib.h"

#include "rtc.h"
#include "tim.h"
#include "SD_save.h"
#include "motorcmd.h"
#include "GPS.h"
#include "can.h"
#include "serial_com.h"

#define D1 150
#define D2 150

#define LOOP_DIV 2.1

#define PROJ_ITR 20
#define PROJECTION_DIST 100

/******************************************************************************/
/*                              Type  Prototype                               */
/******************************************************************************/

typedef enum SGP_Control_State_m {
  SGP_INIT = 0,
  SGP_STANDBY,
  INIT_CALIBRATION_PHASE,
  CALIBRATION_PHASE,
  INIT_DISTANCE_PHASE,
  DISTANCE_PHASE,
  INIT_DIRECT_APPROACH_PHASE,
  DIRECT_APPROACH_PHASE,
  INIT_LANDING_PHASE,
  LANDING_PHASE,
  ON_THE_GROUND_PHASE,
  MOTOR_TEST
} SGP_Control_State_t;

typedef struct Bezier_curve_s {
	vector_2D_t tip;
	vector_2D_t cp1;
	vector_2D_t cp2;
	vector_2D_t fap;
	vector_2D_t a;
	vector_2D_t b;
	vector_2D_t c;
} Bezier_curve_t;

typedef struct SGP_Data_s {
	SGP_Control_State_t SGP_State;
	Bezier_curve_t bezier_data;
	GPS_Data_t GPS_data;
	GPS_Data_t oldGPS_data;
	vector_2D_t uCurrentDir;
	vector_2D_t uWind;
	float Altitude;
	float oldAltitude;
	float HorzSpeed;
	float VertSpeed;
	float DescentTime;
	float PosTTracking;
	uint8_t ControlEnable;
} SGP_Data_t;



/******************************************************************************/
/*                             Global variable                                */
/******************************************************************************/

/******************************************************************************/
/*                             Function prototype                             */
/******************************************************************************/
void SGP_Control_Init();
void SGP_Control_Loop();
uint8_t Launch_MotorTest();
void Set_uWindVector(vector_2D_t _uWind);
void Set_RocketState(Rocket_State_t _Rocket_State);
void Set_Altitude(float _altitude);
Rocket_State_t Get_RocketState(void);

#endif /* SGP_CONTROL_H_ */
