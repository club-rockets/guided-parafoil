#include "serial_com.h"


void serial_menu()
{

  uint8_t val;


  /***************************************************
   * USB SERIAL COM PORT
   ***************************************************/

  //code test pour programmer l'horloge
  //est presentement utilis� avec des script sur teraterm
  //rx is done elsewhere in usb_cdc_if.c
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {

    if (USB_CDC_RX[0] == 's') {


      HAL_GPIO_TogglePin(GPIOD, LED4_Pin);

      val = Launch_MotorTest();
      //val = atoi(&USB_CDC_RX[4]);

      //itoa(115, USB_CDC_TX, 10);

      if (val == 0)
        strcpy(USB_CDC_TX,"Motor test launched\n\r");
      else
        strcpy(USB_CDC_TX,"Motor test canceled\n\r");

      CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

      USB_CDC_RX[0] = 0;
    }

    /* Left motor Command */
    if (USB_CDC_RX[0] == 'q') {


      config_Motor_Command(100, 0);

      strcpy(USB_CDC_TX,"LeftCMD: +100 rad\n\r");

      CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

      USB_CDC_RX[0] = 0;
    }

    if (USB_CDC_RX[0] == 'a') {


        config_Motor_Command(-100, 0);

        strcpy(USB_CDC_TX,"LeftCMD: -100 rad\n\r");

        CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

        USB_CDC_RX[0] = 0;
      }

    /* Left motor Command */
    if (USB_CDC_RX[0] == 'e') {


      config_Motor_Command(0, 100);

      strcpy(USB_CDC_TX,"RightCMD: +100 rad\n\r");

      CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

      USB_CDC_RX[0] = 0;
    }

    if (USB_CDC_RX[0] == 'd') {


        config_Motor_Command(0, -100);

        strcpy(USB_CDC_TX,"RightCMD: -100 rad\n\r");

        CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

        USB_CDC_RX[0] = 0;
      }

    if (USB_CDC_RX[0] == 'r') {


      MotorPos_Reset();

      strcpy(USB_CDC_TX,"Motor position RESET\n\r");

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

      sprintf(USB_CDC_TX, "\n\rMotorLeft CMD:%i\n\rMotorRight CMD:%i\n\rMotorLeft Position:%f\n\rMotorRight Position:%f\n\r",motorleft, motorright, motorleft_pos, motorright_pos);

      CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

      USB_CDC_RX[0] = 0;
    }

    if (USB_CDC_RX[0] == 'g') {

      PolarGPS_Coordinate_t polar_pos, polar_dest;
      CartesianGPS_Coordinate_t cart_pos;

      //Hall A ETS
      polar_dest.altitude = 0;
      polar_dest.latitude = 45.495414;
      polar_dest.longitude = -73.563063;

      polar_pos = GPS_GetCoordinate();
      cart_pos = GPS_GetCartesianCoordinate(polar_dest);

      sprintf(USB_CDC_TX, "\n\rGPS DATA:\n\rFix type: %i\n\rLatitude: %f\n\rLongitude: %f\n\rX: %f\n\rY: %f\n\r", polar_pos.fix_type, polar_pos.latitude, polar_pos.longitude, cart_pos.x, cart_pos.y);

      CDC_Transmit_FS(USB_CDC_TX, strlen(USB_CDC_TX));

      USB_CDC_RX[0] = 0;
    }
  }
}

void Send_serial_message(char* _message)
{
  //memcpy(USB_CDC_TX, _message, strlen(_message));
  CDC_Transmit_FS(_message, strlen(_message));
}