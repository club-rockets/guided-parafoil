#include "GPS.h"

typedef struct UBX_Parser_s {
	uint8_t GPS_frame[GPS_FRAME_LENGTH]; //USART2 Buffer
	uint8_t GPS_FrameRdy; //Data frame ready to parse flag
	GPS_Data_t GPS_Data;
} UBX_Parser_t;

UBX_Parser_t GPS1_Parser;
UBX_Parser_t GPS2_Parser;

PolarCoordinate_t PolarGPS_Destination;

uint8_t CoordinateUpdated = 0;
uint8_t DestinationSet = 0;


uint8_t parse_UBX_char(UBX_Parser_t *_UBX_Parser);

void GPS_Init() {
	GPS1_Parser.GPS_Data.GPS_Number = 1;
	GPS1_Parser.GPS_FrameRdy = 0;

	GPS2_Parser.GPS_Data.GPS_Number = 2;
	GPS2_Parser.GPS_FrameRdy = 0;

	PolarGPS_Destination.latitude = 0.0;
	PolarGPS_Destination.longitude = 0.0;

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);

	HAL_GPIO_WritePin(GPS_D_SEL_GPIO_Port, GPS_D_SEL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_SET);
}

void GPS_Read_Data() {

	/* SD card  */
	uint8_t Save_String[512];                  //SD card write buffer

	//GPS1 Frame parse launcher
	if (GPS1_Parser.GPS_FrameRdy != 0) {

		parse_UBX_char(&GPS1_Parser);

	}

	//GPS1 Frame parse launcher
	if (GPS2_Parser.GPS_FrameRdy != 0) {

		parse_UBX_char(&GPS2_Parser);

	}


	/***************************************************
	 * SD save in buffer
	 ***************************************************/

	/*
	 * https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf
	 * p 225
	 GPSfix Type, range 0..5
	 0x00 = No Fix
	 0x01 = Dead Reckoning only
	 0x02 = 2D-Fix
	 0x03 = 3D-Fix
	 0x04 = GPS + dead reckoning combined
	 0x05 = Time only fix
	 0x06..0xff: reserved
	 */
	sprintf((char*) (Save_String), "%s,%i,%f,%f,%f", "GPS_Data",
			GPS2_Parser.GPS_Data.fix_type, GPS2_Parser.GPS_Data.altitude,
			GPS2_Parser.GPS_Data.PolarCoordinate.latitude, GPS2_Parser.GPS_Data.PolarCoordinate.longitude);
	SD_Save_Data(Save_String);

}

uint8_t parse_UBX_char(UBX_Parser_t *_UBX_Parser) {

	if ((_UBX_Parser->GPS_frame[0] == UBX_SYNC1) && (_UBX_Parser->GPS_frame[1] == UBX_SYNC2))
	{
		int i = 0;
		uint8_t CK_A = 0, CK_B = 0;

		for (i = 2 ; i < 98 ; i++)
		{
			CK_A = CK_A + _UBX_Parser->GPS_frame[i];
			CK_B = CK_B + CK_A;
		}

		if ((CK_A == _UBX_Parser->GPS_frame[98]) && (CK_B == _UBX_Parser->GPS_frame[99]))
		{
			HAL_GPIO_TogglePin(GPIOD, LED1_Pin);

			_UBX_Parser->GPS_Data.fix_type = _UBX_Parser->GPS_frame[26];
			_UBX_Parser->GPS_Data.N_satellites = _UBX_Parser->GPS_frame[29];

			_UBX_Parser->GPS_Data.rawlatitude = _UBX_Parser->GPS_frame[34] | (_UBX_Parser->GPS_frame[35] << 8) | (_UBX_Parser->GPS_frame[36] << 16) | (_UBX_Parser->GPS_frame[37] << 24);
			_UBX_Parser->GPS_Data.PolarCoordinate.latitude = _UBX_Parser->GPS_Data.rawlatitude / 10000000.0;

			_UBX_Parser->GPS_Data.rawlongitude = _UBX_Parser->GPS_frame[30] | (_UBX_Parser->GPS_frame[31] << 8) | (_UBX_Parser->GPS_frame[32] << 16) | (_UBX_Parser->GPS_frame[33] << 24);
			_UBX_Parser->GPS_Data.PolarCoordinate.longitude = _UBX_Parser->GPS_Data.rawlongitude / 10000000.0;

			_UBX_Parser->GPS_Data.ground_speed = _UBX_Parser->GPS_frame[66] | (_UBX_Parser->GPS_frame[67] << 8) | (_UBX_Parser->GPS_frame[68] << 16) | (_UBX_Parser->GPS_frame[69] << 24);
			_UBX_Parser->GPS_Data.heading_motion = _UBX_Parser->GPS_frame[70] | (_UBX_Parser->GPS_frame[71] << 8) | (_UBX_Parser->GPS_frame[72] << 16) | (_UBX_Parser->GPS_frame[73] << 24);

			_UBX_Parser->GPS_Data.CartesianCoordinate.X = (_UBX_Parser->GPS_Data.PolarCoordinate.longitude - PolarGPS_Destination.longitude) * LON_DEGREEVALUE;
			_UBX_Parser->GPS_Data.CartesianCoordinate.Y = (_UBX_Parser->GPS_Data.PolarCoordinate.latitude - PolarGPS_Destination.latitude) * LAT_DEGREEVALUE;
		}
	}

	_UBX_Parser->GPS_FrameRdy = 0;

	return 1;
}

GPS_Data_t* GPS_GetData() {
	if ((GPS2_Parser.GPS_Data.N_satellites >= GPS1_Parser.GPS_Data.N_satellites) && (GPS2_Parser.GPS_Data.fix_type == 3))
		return &GPS2_Parser.GPS_Data;
	else if (GPS1_Parser.GPS_Data.fix_type == 3)
	{
		return &GPS1_Parser.GPS_Data;
	}
	return NULL;
}

GPS_Data_t* GPS_GetSpecificData(uint8_t _GPS_Number) {
	if (GPS1_Parser.GPS_Data.GPS_Number == _GPS_Number){
		return &GPS1_Parser.GPS_Data;
	} else if (GPS2_Parser.GPS_Data.GPS_Number == _GPS_Number){
		return &GPS2_Parser.GPS_Data;
	}

	return NULL;
}

uint8_t IsCoordinateUpdated() {
	return CoordinateUpdated;
}

void Set_GPSDestination(PolarCoordinate_t _PolarDest_Coordinate) {

	PolarGPS_Destination = _PolarDest_Coordinate;
	DestinationSet = 1;
}

//UsART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	//GPS1 UBX frame caption
	if (huart->Instance == USART2) {
		//HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
		memcpy(GPS1_Parser.GPS_frame, usart2_rx, sizeof(usart2_rx));
		GPS1_Parser.GPS_FrameRdy = 1;

		//HAL_UART_Receive_IT(&huart2, usart2_rx, 100);
	}

	//GPS1 UBX frame caption
	if (huart->Instance == USART6) {
		//HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
		memcpy(GPS2_Parser.GPS_frame, usart6_rx, sizeof(usart6_rx));
		GPS2_Parser.GPS_FrameRdy = 1;

		//HAL_UART_Receive_IT(&huart6, usart6_rx, 100);
	}
}

