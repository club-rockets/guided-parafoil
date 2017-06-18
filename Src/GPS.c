#include "GPS.h"

GNSS_HandleTypeDef hgps;

/* Decoder state */
//typedef enum {
//	UBX_DECODE_SYNC1 = 0,
//	UBX_DECODE_SYNC2,
//	UBX_DECODE_CLASS,
//	UBX_DECODE_ID,
//	UBX_DECODE_LENGTH1,
//	UBX_DECODE_LENGTH2,
//	UBX_DECODE_PAYLOAD,
//	UBX_DECODE_CHKSUM1,
//	UBX_DECODE_CHKSUM2
//} ubx_decode_state_t;

typedef struct UBX_Parser_s {
	ubx_decode_state_t parser_state;
} UBX_Parser_t;

UBX_Parser_t GPS1_Parser;

PolarGPS_Coordinate_t PolarGPS_Coordinate = { .altitude = 0.0,
                                              .longitude = 0.0,
                                              .latitude = 0.0,
                                              .N_satellites = 0,
                                              .fix_type = 0};
uint8_t CoordinateUpdated = 0;

void GPS_Init() {
	GPS1_Parser.parser_state = UBX_DECODE_SYNC1;
}

void GPS_Read_Data(uint8_t * GPS_Read_Data) {

  int i = 0;
  uint8_t ack[1];

  /* SD card  */
  uint8_t Save_String[512];                  //SD card write buffer

  for (i = 0; i < GPS_FRAME_LENGTH; i++)
	{
	  //GPS_Read_Data[i]
	}

  GNSS_log(&hgps);

  PolarGPS_Coordinate.altitude = hgps.gps_position->alt / 1000.0;
  PolarGPS_Coordinate.latitude = hgps.gps_position->lat / 10000000.0;
  PolarGPS_Coordinate.longitude = hgps.gps_position->lon / 10000000.0;
  PolarGPS_Coordinate.fix_type = hgps.gps_position->fix_type;
  PolarGPS_Coordinate.N_satellites = hgps.satellite_info->count;

  CoordinateUpdated = 1;

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
  sprintf((char*) (Save_String), "%s,%i,%f,%f,%f", "GPS_Data", hgps.gps_position->fix_type, PolarGPS_Coordinate.altitude, PolarGPS_Coordinate.latitude, PolarGPS_Coordinate.longitude);
  SD_Save_Data(Save_String);

}

uint8_t parse_UBX_char(UBX_Parser_t *_UBX_Parser, const char b)
{
//	switch (_UBX_Parser->parser_state)
//	{
//	case :
//	}
}

PolarGPS_Coordinate_t GPS_GetCoordinate()
{
  CoordinateUpdated = 0;
  return PolarGPS_Coordinate;
}


CartesianGPS_Coordinate_t GPS_GetCartesianCoordinate(PolarGPS_Coordinate_t _PolarDest_Coordinate)
{
  CartesianGPS_Coordinate_t CartesianGPS_Coordinate;

  CartesianGPS_Coordinate.altitude = PolarGPS_Coordinate.altitude;
  CartesianGPS_Coordinate.y = (_PolarDest_Coordinate.latitude - PolarGPS_Coordinate.latitude) * LAT_DEGREEVALUE;
  CartesianGPS_Coordinate.x = (_PolarDest_Coordinate.longitude - PolarGPS_Coordinate.longitude) * LON_DEGREEVALUE;

  CoordinateUpdated = 0;

  return CartesianGPS_Coordinate;
}

uint8_t IsCoordinateUpdated()
{
  return CoordinateUpdated;
}


