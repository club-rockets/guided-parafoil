#include "serial_com.h"

void serial_menu() {

	uint8_t val;

	/***************************************************
	 * USB SERIAL COM PORT
	 ***************************************************/

	//code test pour programmer l'horloge
	//est presentement utilisé avec des script sur teraterm
	//rx is done elsewhere in usb_cdc_if.c
	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {

		if (USB_CDC_RX[0] == 's') {

			HAL_GPIO_TogglePin(GPIOD, LED4_Pin);

			val = Launch_MotorTest();
			//val = atoi(&USB_CDC_RX[4]);

			//itoa(115, USB_CDC_TX, 10);

			if (val == 0)
				strcpy(USB_CDC_TX, "Motor test launched\n\r");
			else
				strcpy(USB_CDC_TX, "Motor test canceled\n\r");

			CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

			USB_CDC_RX[0] = 0;
		}

		/* Left motor Command */
		if (USB_CDC_RX[0] == 'q') {

			config_Motor_Command(100, 0);

			strcpy(USB_CDC_TX, "LeftCMD: +100 rad\n\r");

			CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

			USB_CDC_RX[0] = 0;
		}

		if (USB_CDC_RX[0] == 'a') {

			config_Motor_Command(-100, 0);

			strcpy(USB_CDC_TX, "LeftCMD: -100 rad\n\r");

			CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

			USB_CDC_RX[0] = 0;
		}

		/* Left motor Command */
		if (USB_CDC_RX[0] == 'e') {

			config_Motor_Command(0, 100);

			strcpy(USB_CDC_TX, "RightCMD: +100 rad\n\r");

			CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

			USB_CDC_RX[0] = 0;
		}

		if (USB_CDC_RX[0] == 'd') {

			config_Motor_Command(0, -100);

			strcpy(USB_CDC_TX, "RightCMD: -100 rad\n\r");

			CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

			USB_CDC_RX[0] = 0;
		}

		if (USB_CDC_RX[0] == 'r') {

			MotorPos_Reset();

			strcpy(USB_CDC_TX, "Motor position RESET\n\r");

			CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

			USB_CDC_RX[0] = 0;
		}

		if (USB_CDC_RX[0] == 'z') {

			int motorleft = 0, motorright = 0;
			float motorleft_pos = 0.0, motorright_pos = 0.0;

			motorleft = Get_LeftMotor_command();
			motorright = Get_RightMotor_command();

			motorleft_pos = Get_LeftMotor_position();
			motorright_pos = Get_RightMotor_position();

			sprintf(USB_CDC_TX,
					"\n\rMotorLeft CMD:%i\n\rMotorRight CMD:%i\n\rMotorLeft Position:%f\n\rMotorRight Position:%f\n\r",
					motorleft, motorright, motorleft_pos, motorright_pos);

			CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

			USB_CDC_RX[0] = 0;
		}

		if (USB_CDC_RX[0] == 'g') {

			GPS_Data_t *GPSData;

			GPSData = GPS_GetData();

			if (GPSData != NULL) {
				sprintf(USB_CDC_TX,
						"\n\rGPS DATA:\n\rGPS Number: %i\n\rFix type: %i\n\rLatitude: %f\n\rLongitude: %f\n\rX: %f\n\rY: %f\n\rNb Satellites: %lu\n\r",
						GPSData->GPS_Number, GPSData->fix_type,
						GPSData->PolarCoordinate.latitude,
						GPSData->PolarCoordinate.longitude,
						GPSData->CartesianCoordinate.X,
						GPSData->CartesianCoordinate.Y, GPSData->N_satellites);
			}
			else
			{
				sprintf(USB_CDC_TX,
						"\n\rGPS DATA: no fix\n\r");
			}

			CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

			USB_CDC_RX[0] = 0;
		}

	if (USB_CDC_RX[0] == '1') {

			GPS_Data_t *GPSData;

			GPSData = GPS_GetSpecificData(1);

			if (GPSData != NULL) {
				sprintf(USB_CDC_TX,
						"\n\rGPS DATA:\n\rGPS Number: %i\n\rFix type: %i\n\rLatitude: %f\n\rLongitude: %f\n\rX: %f\n\rY: %f\n\rNb Satellites: %lu\n\r",
						GPSData->GPS_Number, GPSData->fix_type,
						GPSData->PolarCoordinate.latitude,
						GPSData->PolarCoordinate.longitude,
						GPSData->CartesianCoordinate.X,
						GPSData->CartesianCoordinate.Y, GPSData->N_satellites);
			}
			else
			{
				sprintf(USB_CDC_TX,
						"\n\rGPS DATA: no fix\n\r");
			}

			CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

			USB_CDC_RX[0] = 0;
		}

		if (USB_CDC_RX[0] == 'c') {

			uint8_t res = 0;
			hcan2.pTxMsg->Data[0] = 'a';
			hcan2.pTxMsg->Data[1] = 'b';
			hcan2.pTxMsg->Data[2] = 'c';
			hcan2.pTxMsg->Data[3] = 'd';
			hcan2.pTxMsg->Data[4] = 'e';
			hcan2.pTxMsg->StdId = 0x001;
			hcan2.pTxMsg->DLC = 2;

			res = HAL_CAN_Transmit(&hcan2, 5);

			sprintf(USB_CDC_TX, "CAN TEST: %i\n\r", Get_RocketState());
			CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

			USB_CDC_RX[0] = 0;
		}
	}
}

void Send_serial_message(char* _message) {
	//memcpy(USB_CDC_TX, _message, strlen(_message));
	CDC_Transmit_FS((uint8_t *) _message, strlen(_message));
}
