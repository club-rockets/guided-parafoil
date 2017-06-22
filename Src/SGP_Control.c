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
/*                             Function prototype                             */
/******************************************************************************/
void Calculate_BiezerCoef(Bezier_curve_t *_BiezerData);
float Evaluate_VertSpeed(float _diff_altitude, float _diff_time);
float Evaluate_HorzSpeed(float _dist_traveled, float _diff_time);
void Save_BezierData(Bezier_curve_t *_BezierData);

/******************************************************************************/
/*                             Global variable                                */
/******************************************************************************/

Rocket_State_t Rocket_State = INITIALISATION;

//Motor test variable
uint8_t motor_testCounter = 0;

//Path planning variable
SGP_Data_t SGP_Data;

void SGP_Control_Init() {
	Rocket_State = INITIALISATION;
	SGP_Data.SGP_State = SGP_INIT;

	SGP_Data.uWind.X = 1.0;
	SGP_Data.uWind.Y = 0.0;

	SGP_Data.PosTTracking = 0.0;

	SGP_Data.ControlEnable = 0;

	motor_testCounter = 0;

}

void SGP_Control_Loop() {
	// buffer pour sauvegarder des donnees
	//uint8_t Save_String[512];

	//Path planning variable
	float DestDist = 0.0, LoopDist = 0.0, fapDist = 0.0, h = 0.0;
	float Dist_traveled = 0.0, Height_loss = 0.0;
	vector_2D_t uHDirection, ufapDir, uMidDir;
	vector_2D_t InitialPnt;

	//Parafoil position command variable
	vector_2D_t ErrPnt;
	float ProjTCenter, ProjStep, ProjX1, ProjY1, ProjX2, ProjY2, ProjDistUp, ProjDistDown, ProjTUp, ProjTDown;


	static float DirErr = 0.0;
	int MotorLeft_PosCmd, MotorRight_PosCmd = 0;

	int i;
	static uint8_t time = 0;

	HAL_GPIO_TogglePin(GPIOD, LED2_Pin);

	SGP_Data.oldGPS_data = SGP_Data.GPS_data;
	SGP_Data.GPS_data = *GPS_GetData();

	//Horizontal parameters calculation
	//uCurrentDir not unit vector
	SGP_Data.uCurrentDir.X = SGP_Data.GPS_data.CartesianCoordinate.X - SGP_Data.oldGPS_data.CartesianCoordinate.X;
	SGP_Data.uCurrentDir.Y = SGP_Data.GPS_data.CartesianCoordinate.Y - SGP_Data.oldGPS_data.CartesianCoordinate.Y;

	Dist_traveled = sqrt(pow((SGP_Data.uCurrentDir.X), 2) + pow((SGP_Data.uCurrentDir.Y), 2));
	SGP_Data.HorzSpeed = Evaluate_HorzSpeed(Dist_traveled, 1.0);

	//Now uCurrentDir is a unit vector
	SGP_Data.uCurrentDir.X = SGP_Data.uCurrentDir.X / SGP_Data.HorzSpeed;
	SGP_Data.uCurrentDir.Y = SGP_Data.uCurrentDir.Y / SGP_Data.HorzSpeed;

	//Vertical parameters calculation
	Height_loss = SGP_Data.Altitude - SGP_Data.oldAltitude;
	SGP_Data.VertSpeed = Evaluate_VertSpeed(Height_loss, 1.0);

	SGP_Data.DescentTime = abs(SGP_Data.Altitude / SGP_Data.VertSpeed);

	/* SGP Control system state machine */
	switch (SGP_Data.SGP_State) {

	case SGP_INIT:
		SGP_Data.SGP_State = SGP_STANDBY;
		break;

	case SGP_STANDBY:
		if (Rocket_State == MAIN_DESCENT)
			SGP_Data.SGP_State = CALIBRATION_PHASE;
		break;

	case INIT_CALIBRATION_PHASE:

		CalibrateMotor();

		//Set a straight line trajectory
		SGP_Data.bezier_data.tip = SGP_Data.GPS_data.CartesianCoordinate;
		SGP_Data.bezier_data.cp1 = SGP_Data.bezier_data.tip;

		SGP_Data.bezier_data.fap.X = 1000 * SGP_Data.uCurrentDir.X + SGP_Data.bezier_data.tip.X;
		SGP_Data.bezier_data.fap.Y = 1000 * SGP_Data.uCurrentDir.Y + SGP_Data.bezier_data.tip.Y;
		SGP_Data.bezier_data.cp2 = SGP_Data.bezier_data.fap;

		//Save bezier parameter
		Save_BezierData(&SGP_Data.bezier_data);
		//Calculate
		Calculate_BiezerCoef(&SGP_Data.bezier_data);

		SGP_Data.ControlEnable = 1;//Enable position control on parafoil

		time = 0;

		SGP_Data.SGP_State = CALIBRATION_PHASE;
		break;

	case CALIBRATION_PHASE:
		if (time < 6)
		{
			time++;
		}
		else
		{
			SGP_Data.SGP_State = INIT_DISTANCE_PHASE;
		}

		break;

	case INIT_DISTANCE_PHASE:

		SGP_Data.PosTTracking = 0.0;

		//Define TIP at rocket actual position
		SGP_Data.bezier_data.tip = SGP_Data.GPS_data.CartesianCoordinate;

		//Direction from the rocket to the destination
		//Destination set at [0,0]
		DestDist = sqrt(pow(SGP_Data.bezier_data.tip.X, 2) + pow(SGP_Data.bezier_data.tip.Y, 2));
		uHDirection.X = - SGP_Data.bezier_data.tip.X / DestDist;
		uHDirection.Y = - SGP_Data.bezier_data.tip.Y / DestDist;

		//CP1 calculation
		SGP_Data.bezier_data.cp1.X = (SGP_Data.bezier_data.tip.X + D1 * SGP_Data.uCurrentDir.X);
		SGP_Data.bezier_data.cp1.Y = (SGP_Data.bezier_data.tip.Y + D1 * SGP_Data.uCurrentDir.Y);

		LoopDist = (SGP_Data.HorzSpeed * SGP_Data.DescentTime) / LOOP_DIV;

		//Initiate the distance phase sequence.
		//The destination is too near of the rocket main deployment.
		if (DestDist < LoopDist)
		{
			//Define FAP arrival direction according to actual direction
			InitialPnt.X = SGP_Data.uCurrentDir.X + SGP_Data.bezier_data.tip.X;
			InitialPnt.Y = SGP_Data.uCurrentDir.Y + SGP_Data.bezier_data.tip.Y;

			if (SGP_Data.bezier_data.tip.X * InitialPnt.Y - InitialPnt.X * SGP_Data.bezier_data.tip.Y > 0)
			{
				ufapDir.X = uHDirection.Y;
				ufapDir.Y = -uHDirection.X;
			}
			else
			{
				ufapDir.X = -uHDirection.Y;
				ufapDir.Y = uHDirection.X;
			}

			h = sqrt(pow(LoopDist,2) - pow(DestDist,2));

			SGP_Data.bezier_data.fap.X = h * ufapDir.X + SGP_Data.bezier_data.tip.X / 2.0;
			SGP_Data.bezier_data.fap.Y = h * ufapDir.Y + SGP_Data.bezier_data.tip.Y / 2.0;

			fapDist = sqrt(pow(SGP_Data.bezier_data.fap.X,2) + pow(SGP_Data.bezier_data.fap.Y,2));

			if (SGP_Data.bezier_data.tip.X * InitialPnt.Y - InitialPnt.X * SGP_Data.bezier_data.tip.Y > 0)
			{
				uMidDir.X = -SGP_Data.bezier_data.fap.Y / fapDist;
				uMidDir.Y = SGP_Data.bezier_data.fap.X / fapDist;
			}
			else
			{
				uMidDir.X = SGP_Data.bezier_data.fap.Y / fapDist;
				uMidDir.Y = -SGP_Data.bezier_data.fap.X / fapDist;
			}

			SGP_Data.bezier_data.cp2.X = SGP_Data.bezier_data.fap.X - D2 * uMidDir.X;
			SGP_Data.bezier_data.cp2.Y = SGP_Data.bezier_data.fap.Y - D2 * uMidDir.Y;

			//Save bezier parameter
			Save_BezierData(&SGP_Data.bezier_data);
			//Calculate
			Calculate_BiezerCoef(&SGP_Data.bezier_data);

			SGP_Data.ControlEnable = 1;//Enable position control on parafoil

			SGP_Data.SGP_State = DISTANCE_PHASE;
		}
		//Skip the distance phase sequence.
		//The destination is at a good distance or to far from the rocket.
		else
		{
			//FAP point to destination
			SGP_Data.bezier_data.fap.X = 0.0;
			SGP_Data.bezier_data.fap.Y = 0.0;

			SGP_Data.bezier_data.cp2.X = SGP_Data.bezier_data.fap.X - D2 * SGP_Data.uWind.X;
			SGP_Data.bezier_data.cp2.Y = SGP_Data.bezier_data.fap.Y - D2 * SGP_Data.uWind.Y;

			//Save bezier parameter
			Save_BezierData(&SGP_Data.bezier_data);
			//Calculate
			Calculate_BiezerCoef(&SGP_Data.bezier_data);

			SGP_Data.ControlEnable = 1;//Enable position control on parafoil

			SGP_Data.SGP_State = INIT_DIRECT_APPROACH_PHASE;
		}

		break;
	case DISTANCE_PHASE:

		if (SGP_Data.PosTTracking > 0.99)
		{
			SGP_Data.SGP_State = INIT_DIRECT_APPROACH_PHASE;
		}
		else
		{

			float DistHorzBank = (SGP_Data.HorzSpeed * SGP_Data.DescentTime) - 300;

			float DistToDest = sqrt(pow(SGP_Data.GPS_data.CartesianCoordinate.X, 2) + pow(SGP_Data.GPS_data.CartesianCoordinate.Y, 2));

			if (DistHorzBank < DistToDest)
			{
				SGP_Data.SGP_State = INIT_DIRECT_APPROACH_PHASE;
			}
			else
			{
				if (DirErr > 200)
					SGP_Data.SGP_State = INIT_DISTANCE_PHASE;
			}
		}
		break;

	case INIT_DIRECT_APPROACH_PHASE:

		SGP_Data.PosTTracking = 0.0;

		//Bezier point calculation
		SGP_Data.bezier_data.tip.X = SGP_Data.GPS_data.CartesianCoordinate.X;
		SGP_Data.bezier_data.tip.Y = SGP_Data.GPS_data.CartesianCoordinate.Y;

		SGP_Data.bezier_data.fap.X = 0.0;
		SGP_Data.bezier_data.fap.Y = 0.0;

		SGP_Data.bezier_data.cp1.X = SGP_Data.bezier_data.tip.X + D1 * SGP_Data.uCurrentDir.X;
		SGP_Data.bezier_data.cp1.Y = SGP_Data.bezier_data.tip.Y + D1 * SGP_Data.uCurrentDir.Y;

		SGP_Data.bezier_data.cp2.X = SGP_Data.bezier_data.fap.X + D2 * SGP_Data.uWind.X;
		SGP_Data.bezier_data.cp2.Y = SGP_Data.bezier_data.fap.Y + D2 * SGP_Data.uWind.Y;

		//Save bezier parameter
		Save_BezierData(&SGP_Data.bezier_data);
		//Calculate
		Calculate_BiezerCoef(&SGP_Data.bezier_data);

		SGP_Data.SGP_State = DIRECT_APPROACH_PHASE;

		break;

	case DIRECT_APPROACH_PHASE:
		if (SGP_Data.Altitude <= 50.0)
		{
			SGP_Data.SGP_State = INIT_LANDING_PHASE;
		}
		break;

	case INIT_LANDING_PHASE:

		time = 0;
		SGP_Data.ControlEnable = 0;

		//Break
		MotorLeft_PosCmd = 5000;
		MotorRight_PosCmd = 5000;

		SGP_Data.SGP_State = INIT_LANDING_PHASE;
		break;

	case LANDING_PHASE:

		if (time <=10)
		{
			time++;
		}
		else
		{
			SGP_Data.SGP_State = ON_THE_GROUND_PHASE;
		}

		break;

	case ON_THE_GROUND_PHASE:

		Disable_MotorCMD();

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
			SGP_Data.SGP_State = SGP_INIT;

		}

		Set_Motor_Command(MotorLeft_PosCmd, MotorRight_PosCmd);
		motor_testCounter++;
		break;

	default:
		SGP_Data.SGP_State = SGP_INIT;
		break;
	}

	/********************	PARAFOIL POSITION CONTROL *******************/
	if (SGP_Data.ControlEnable)
	{
		DirErr = 0.0;

		//We project the rocket position foward at PROJECTION_DIST meters
		//Projection point calculation
		ErrPnt.X = SGP_Data.GPS_data.CartesianCoordinate.X + PROJECTION_DIST * SGP_Data.uCurrentDir.X;
		ErrPnt.Y = SGP_Data.GPS_data.CartesianCoordinate.Y + PROJECTION_DIST * SGP_Data.uCurrentDir.Y;


		//TODO: Upgrade the projection on biezer curve algorithm
		ProjTCenter = 0.5;
		ProjStep = 0.25;

		ProjX1 = 0.0;
		ProjY1 = 0.0;
		ProjX2 = 0.0;
		ProjY2 = 0.0;
		ProjDistDown = 0.0;
		ProjTUp = 0.0;
		ProjTDown = 0.0;

		for (i = 0; i < PROJ_ITR; i++)
		{
			ProjTUp = ProjTCenter + ProjStep;
			ProjTDown = ProjTCenter - ProjStep;

			ProjX1 = SGP_Data.bezier_data.a.X * pow(ProjTUp, 3) + SGP_Data.bezier_data.b.X * pow(ProjTUp, 2) + SGP_Data.bezier_data.c.X * ProjTUp + SGP_Data.bezier_data.tip.X;
			ProjY1 = SGP_Data.bezier_data.a.Y * pow(ProjTUp, 3) + SGP_Data.bezier_data.b.Y * pow(ProjTUp, 2) + SGP_Data.bezier_data.c.Y * ProjTUp + SGP_Data.bezier_data.tip.Y;

			ProjDistUp = sqrt(pow(ErrPnt.X - ProjX1, 2) + pow(ErrPnt.Y - ProjY1, 2));

			ProjX2 = SGP_Data.bezier_data.a.X * pow(ProjTDown, 3) + SGP_Data.bezier_data.b.X * pow(ProjTDown, 2) + SGP_Data.bezier_data.c.X * ProjTDown + SGP_Data.bezier_data.tip.X;
			ProjY2 = SGP_Data.bezier_data.a.Y * pow(ProjTDown, 3) + SGP_Data.bezier_data.b.Y * pow(ProjTDown, 2) + SGP_Data.bezier_data.c.Y * ProjTDown + SGP_Data.bezier_data.tip.Y;

			ProjDistDown = sqrt(pow(ErrPnt.X - ProjX2, 2) + pow(ErrPnt.Y - ProjY2, 2));

			if (ProjDistUp < ProjDistDown)
				ProjTCenter = ProjTCenter + ProjStep;
			else
				ProjTCenter = ProjTCenter - ProjStep;

			ProjStep = ProjStep/2;
		}

		if (SGP_Data.PosTTracking < ProjTCenter)
		{
			SGP_Data.PosTTracking = ProjTCenter;
		}
		else
		{

			ProjX2 = SGP_Data.bezier_data.a.X * pow(SGP_Data.PosTTracking, 3) + SGP_Data.bezier_data.b.X * pow(SGP_Data.PosTTracking, 2) + SGP_Data.bezier_data.c.X * SGP_Data.PosTTracking + SGP_Data.bezier_data.tip.X;
			ProjY2 = SGP_Data.bezier_data.a.Y * pow(SGP_Data.PosTTracking, 3) + SGP_Data.bezier_data.b.Y * pow(SGP_Data.PosTTracking, 2) + SGP_Data.bezier_data.c.Y * SGP_Data.PosTTracking + SGP_Data.bezier_data.tip.Y;

			ProjDistDown = sqrt(pow(ErrPnt.X - ProjX2, 2) + pow(ErrPnt.Y - ProjY2, 2));
		}

		//Define which side to turn
		if (((ErrPnt.X - SGP_Data.GPS_data.CartesianCoordinate.X)*(ProjY2 - SGP_Data.GPS_data.CartesianCoordinate.Y) - (ErrPnt.Y - SGP_Data.GPS_data.CartesianCoordinate.Y)*(ProjX2 - SGP_Data.GPS_data.CartesianCoordinate.X)) > 0)
			DirErr = ProjDistDown;
		else
			DirErr = -ProjDistDown;


		 if (DirErr > 0) {
			 MotorLeft_PosCmd = PGAIN * DirErr + 300;
			 MotorRight_PosCmd = 300;
		 } else {
			 MotorLeft_PosCmd = 300;
			 MotorRight_PosCmd = PGAIN * DirErr + 300;
		 }

		 Set_Motor_Command(MotorLeft_PosCmd, MotorRight_PosCmd);
	}

	//CAN message
	uint8_t Data[8];

	//General algorithme parameter
	memcpy(Data, &SGP_Data.SGP_State, sizeof(uint8_t[8]));
	Send_CAN_Message(Data, CAN_SGP_STATE);

	memcpy(Data, &SGP_Data.DescentTime, sizeof(float));
	Send_CAN_Message(Data, CAN_SGP_DESCENTTIME);

	memcpy(Data, &SGP_Data.HorzSpeed, sizeof(float));
	Send_CAN_Message(Data, CAN_SGP_HORZSPEED);

	memcpy(Data, &SGP_Data.VertSpeed, sizeof(float));
	Send_CAN_Message(Data, CAN_SGP_VERTSPEED);

	memcpy(Data, &SGP_Data.PosTTracking, sizeof(float));
	Send_CAN_Message(Data, CAN_SGP_POSTTRACKING);

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

	//SD_Save_Data(Save_String);
	//CANBUS_LaunchDataRead();
}

//Calculate bezier curve coefficient
void Calculate_BiezerCoef(Bezier_curve_t *_BiezerData)
{
	_BiezerData->c.X = 3 * (_BiezerData->cp1.X - _BiezerData->tip.X);
	_BiezerData->b.X = 3 * (_BiezerData->cp2.X - _BiezerData->cp1.X) - _BiezerData->c.X;
	_BiezerData->a.X = _BiezerData->fap.X - _BiezerData->tip.X - _BiezerData->c.X - _BiezerData->b.X;

	_BiezerData->c.Y = 3 * (_BiezerData->cp1.Y - _BiezerData->tip.Y);
	_BiezerData->b.Y = 3 * (_BiezerData->cp2.Y - _BiezerData->cp1.Y) - _BiezerData->c.Y;
	_BiezerData->a.Y = _BiezerData->fap.Y - _BiezerData->tip.Y - _BiezerData->c.Y - _BiezerData->b.Y;
}

float Evaluate_VertSpeed(float _diff_altitude, float _diff_time)
{
	static uint8_t WriteIndex;
	static float VertSpeed[5];

	int i = 0;
	float SpeedSum = 0.0;

	VertSpeed[WriteIndex] = _diff_altitude / _diff_time;

	SpeedSum = 0;
	for (i = 0; i < 5; i++)
	{
		SpeedSum = VertSpeed[i];
	}

	WriteIndex++;

	if (WriteIndex >= 5)
		WriteIndex = 0;

	return SpeedSum / 5.0;
}

float Evaluate_HorzSpeed(float _dist_traveled, float _diff_time)
{
	static uint8_t WriteIndex;
	static float HorzSpeed[5];

	int i = 0;
	float SpeedSum = 0.0;

	HorzSpeed[WriteIndex] = _dist_traveled / _diff_time;

	SpeedSum = 0;
	for (i = 0; i < 5; i++)
	{
		SpeedSum = HorzSpeed[i];
	}

	WriteIndex++;

	if (WriteIndex >= 5)
		WriteIndex = 0;

	return SpeedSum / 5.0;
}

void Save_BezierData(Bezier_curve_t *_BezierData)
{

	//CAN message
	uint8_t Data[8];

	//Bezier curve parameter
	//TIP
	memcpy(Data, &_BezierData->tip.X, sizeof(float));
	Send_CAN_Message(Data, CAN_BEZIER_TIP_X);
	memcpy(Data, &_BezierData->tip.Y, sizeof(float));
	Send_CAN_Message(Data, CAN_BEZIER_TIP_Y);

	//FAP
	memcpy(Data, &_BezierData->fap.X, sizeof(float));
	Send_CAN_Message(Data, CAN_BEZIER_FAP_X);
	memcpy(Data, &_BezierData->fap.Y, sizeof(float));
	Send_CAN_Message(Data, CAN_BEZIER_FAP_Y);

	//CP1
	memcpy(Data, &_BezierData->cp1.X, sizeof(float));
	Send_CAN_Message(Data, CAN_BEZIER_CP1_X);
	memcpy(Data, &_BezierData->cp1.Y, sizeof(float));
	Send_CAN_Message(Data, CAN_BEZIER_CP1_Y);

	//CP2
	memcpy(Data, &_BezierData->fap.X, sizeof(float));
	Send_CAN_Message(Data, CAN_BEZIER_CP2_X);
	memcpy(Data, &_BezierData->fap.Y, sizeof(float));
	Send_CAN_Message(Data, CAN_BEZIER_CP2_Y);
}

uint8_t Launch_MotorTest() {
	if ((Rocket_State == INITIALISATION) || (Rocket_State == STANDBY_ON_PAD)) {

		motor_testCounter = 0;
		SGP_Data.SGP_State = MOTOR_TEST;
		return 0;

	} else {

		return 1;

	}
}

void Set_uWindVector(vector_2D_t _uWind)
{
	SGP_Data.uWind = _uWind;
}

void Set_RocketState(Rocket_State_t _Rocket_State) {
	Rocket_State = _Rocket_State;
}

//Set altitude in meters
void Set_Altitude(float _altitude) {

	SGP_Data.oldAltitude = SGP_Data.Altitude;
	SGP_Data.Altitude = _altitude;

}

Rocket_State_t Get_RocketState(void) {
	return Rocket_State;
}

