#include "GPS.h"

GNSS_HandleTypeDef hgps;

PolarGPS_Coordinate_t PolarGPS_Coordinate = { .altitude = 0.0, .longitude = 0.0, .latitude = 0.0 };
uint8_t CoordinateUpdated = 0;

void GPS_Init() {
  hgps.Init.baudrate = huart2.Init.BaudRate;
  hgps.Init.portID = UBLOC_UART1;
  hgps.Init.dynModel = DYNMODEL_AUTOMOTIVE;
  hgps.Init.fixMode = FIXMODE_3D;
  hgps.Init.MeasurementRate = UBX_TX_CFG_RATE_MEASINTERVAL;
  hgps.Init.navigationRate = UBX_TX_CFG_RATE_NAVRATE;
  hgps.Init.timeRef = UBX_REF_TIME_GPS;
  hgps.huart = &huart2;
  hgps.use_sat_info = 0;
  hgps.use_gnss_meas = 0;
  hgps.use_subFrame = 0;
  GNSS_Init(&hgps);

  struct satellite_info_s satv;
  hgps.satellite_info = &satv;

  struct vehicle_gps_position_s posGPS;
  hgps.gps_position = &posGPS;

//  struct gnss_raw_meas_s rawGnssData;
//  hgps.gnss_meas = &rawGnssData;
//
//  struct raw_subframe_s sframe;
//  hgps.subFrames = &sframe;

  //release GPS reset
  HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_SET);

  UBX_StatusTypeDef response = UBX_CFG_ERROR_BAUD;

  while (response != UBX_OK) {
    HAL_GPIO_WritePin(GPIOD, LED4_Pin, GPIO_PIN_SET);
    response = GNSS_configure();
  }
  HAL_GPIO_WritePin(GPIOD, LED4_Pin, GPIO_PIN_RESET);
}

void GPS_Read_Data(uint8_t * GPS_Read_Data) {

  int i = 0;

  /* SD card  */
  uint8_t Save_String[512];                  //SD card write buffer


  for (i = 0; i < BUFFER_SIZE; i++)
    parse_char(GPS_Read_Data[i]);

  PolarGPS_Coordinate.altitude = hgps.gps_position->alt / 1000.0;
  PolarGPS_Coordinate.latitude = hgps.gps_position->lat / 10000000.0;
  PolarGPS_Coordinate.longitude = hgps.gps_position->lon / 10000000.0;

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

//  if (hgps.got_posllh || hgps.got_sol) {
//
//    hgps.got_posllh = false;
//    hgps.got_sol = false;
////  Set_GPSFix(hgps.gps_position->fix_type);
//
////  Set_GPS_DOP(hgps.gps_position->dop);
////  Set_GPS_DOPH(hgps.gps_position->eph);
////  Set_GPS_DOPV(hgps.gps_position->epv);
////  Set_Longitude((hgps.gps_position->lon) / 10000000.0);
////  Set_Latitude((hgps.gps_position->lat) / 10000000.0);
////  Set_Altitude((hgps.gps_position->alt) / 1000.0);
//  }

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
