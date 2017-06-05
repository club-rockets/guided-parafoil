#include "GPS.h"

GNSS_HandleTypeDef hgps;

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

  /***************************************************
   * SD save in buffer
   ***************************************************/
  sprintf((char*) (Save_String), "%s,%i", "GPS_Data", hgps.gps_position->fix_type);

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
