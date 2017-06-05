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

#include "rtc.h"
#include "tim.h"
#include "SD_save.h"
#include "motorcmd.h"

/******************************************************************************/
/*                              Type  Prototype                               */
/******************************************************************************/
typedef enum Rocket_State_m {
  INITIALISATION = 0,
  STANDBY_ON_PAD,
  LAUNCH,
  POWERED_ASCENT,
  ENGINE_BURNOUT,
  COASTING_ASCENT,
  APOGEE_REACHED,
  DROGUE_DEPLOYMENT,
  DROGUE_DESCENT,
  MAIN_DEPLOYMENT,
  MAIN_DESCENT,
  LANDING,
  RECOVERY,
  PICKEDUP
} Rocket_State_t;

typedef enum SGP_Control_State_m {
  SGP_INIT = 0,
  CALIBRATION_PHASE,
  LOOP_PHASE,
  DIRECT_APPROACH_PHASE,
  LANDING_PHASE,
  ON_THE_GROUND_PHASE
} SGP_Control_State_t;

/******************************************************************************/
/*                             Global variable                                */
/******************************************************************************/


/******************************************************************************/
/*                             Function prototype                             */
/******************************************************************************/
void SGP_Control_Loop();

#endif /* SGP_CONTROL_H_ */
