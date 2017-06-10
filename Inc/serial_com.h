/*
 * serial_com.h
 *
 *  Created on: Jun 9, 2017
 *      Author: Hugo
 */

#ifndef SERIAL_COM_H_
#define SERIAL_COM_H_

#include "string.h"
#include "stdlib.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "SGP_Control.h"
#include "motorcmd.h"
#include "SD_save.h"
#include "GPS.h"

void serial_menu();
void Send_serial_message(char* _message);

//Buffer for virtual com port USB
  uint8_t USB_CDC_RX[64];
  uint8_t USB_CDC_TX[64];

#endif /* SERIAL_COM_H_ */
