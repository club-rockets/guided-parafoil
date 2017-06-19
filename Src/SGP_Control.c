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

#include "SGP_Control.h"

/******************************************************************************/
/*                             Global variable                                */
/******************************************************************************/

Rocket_State_t Rocket_State = INITIALISATION;
SGP_Control_State_t SGP_Control_State = SGP_INIT;

PolarCoordinate_t PolarGPS_Position = {.longitude = 0.0, .latitude = 0.0 };
//uint8_t DestinationSet = 0;

//Motor test variable
uint8_t motor_testState = 0;
uint8_t motor_testCounter = 0;

void SGP_Control_Init() {
	Rocket_State = INITIALISATION;
	SGP_Control_State = SGP_INIT;
	//DestinationSet = 0;

	motor_testState = 0;
	motor_testCounter = 0;

	//TODO: search destination in backup RAM
}

void SGP_Control_Loop() {
	// buffer pour sauvegarder des donnees
	uint8_t Save_String[512];

	int Direction_Error = 0;
	int MotorLeft_PosCmd, MotorRight_PosCmd = 0;

	HAL_GPIO_TogglePin(GPIOD, LED2_Pin);

	/* Rocket state */
	switch (Rocket_State) {

	case INITIALISATION:

		break;

	case STANDBY_ON_PAD:

		break;

	case LAUNCH:

		break;

	case POWERED_ASCENT:

		break;

	case ENGINE_BURNOUT:

		break;

	case COASTING_ASCENT:

		break;

	case APOGEE_REACHED:

		break;

	case DROGUE_DEPLOYMENT:

		break;

	case DROGUE_DESCENT:

		break;

	case MAIN_DEPLOYMENT:

		break;

	case MAIN_DESCENT:

		break;

	case LANDING:

		break;

	case RECOVERY:

		break;

	case PICKEDUP:

		break;

	default:
		Rocket_State = INITIALISATION;
		break;
	}

	/* SGP Control system state */
	switch (SGP_Control_State) {

	case SGP_INIT:
		SGP_Control_State = SGP_STANDBY;
		break;

	case SGP_STANDBY:
		if (Rocket_State == MAIN_DESCENT)
			SGP_Control_State = CALIBRATION_PHASE;
		break;

	case CALIBRATION_PHASE:

		break;

	case MOTOR_TEST:

		if (motor_testCounter == 0) {
			Enable_MotorCMD();
		}

		if (motor_testCounter < 10) {

			MotorLeft_PosCmd = 5000;
			MotorRight_PosCmd = 0;

		} else if (motor_testCounter < 20) {

			MotorLeft_PosCmd = 0;
			MotorRight_PosCmd = 5000;

		} else {

			MotorLeft_PosCmd = 0;
			MotorRight_PosCmd = 0;
			Disable_MotorCMD();
			SGP_Control_State = SGP_INIT;

		}

		Set_Motor_Command(MotorLeft_PosCmd, MotorRight_PosCmd);
		motor_testCounter++;
		break;

	default:
		SGP_Control_State = SGP_INIT;
		break;
	}

	/*
	 if (Direction_Error > 0) {
	 MotorLeft_PosCmd = PGAIN * 1400 + 300;
	 MotorRight_PosCmd = 300;
	 } else {
	 MotorLeft_PosCmd = 300;
	 MotorRight_PosCmd = PGAIN * 1400 + 300;
	 }

	 Set_Motor_Command(MotorLeft_PosCmd, MotorRight_PosCmd);
	 */
	/***************************************************
	 * SD save in buffer
	 ***************************************************/
//	if (DestinationSet != 0) {
//		sprintf((char*) (Save_String), "%s,%s,%i", "SGP_Control", "SGP Ready.",
//				Rocket_State);
//	} else {
//		sprintf((char*) (Save_String), "%s,%s,%i", "SGP_Control",
//				"Destination coordinate not set.", Rocket_State);
//	}

	SD_Save_Data(Save_String);
	//CANBUS_LaunchDataRead();
}

uint8_t Launch_MotorTest() {
	if ((Rocket_State == INITIALISATION) || (Rocket_State != STANDBY_ON_PAD)) {

		motor_testCounter = 0;
		SGP_Control_State = MOTOR_TEST;
		return 0;

	} else {

		return 1;

	}
}



void Set_RocketState(Rocket_State_t _Rocket_State) {
	Rocket_State = _Rocket_State;
}

Rocket_State_t Get_RocketState(void) {
	return Rocket_State;
}

