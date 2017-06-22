#include "mti.h"

#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_tim.h"

#include "SD_save.h"
#include "main.h"
#include "serial_com.h"
#include "stm32f4xx_it.h"

// External variables
extern SPI_HandleTypeDef hspi1;

/* XBus Messages */
static const uint8_t MSG_WAKEUP_ACK[] = { 0x03, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0xC2 };
static const uint8_t MSG_SET_SYNC_SETTINGS[] = { 0x03, 0xFF, 0xFF, 0xFF, 0x2C, 0x0C, 0x08, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB };
static const uint8_t MSG_GOTO_MEASUREMENT[] = { 0x03, 0xFF, 0xFF, 0xFF, 0x10, 0x00, 0xF1 };

static const uint8_t MSG_SET_OUTPUT_CONFIGURATION[] = {
	0x03, // Control pipe
	0xFF, 0xFF, 0xFF, // Padding bytes
	0xC0, // Message ID
	0x28, // Len
	0x10, 0x20, 0x00, 0x32, //  1. Packet Counter
	0x10, 0x60, 0x00, 0x32, //  2. Sample time fine
	0x20, 0x30, 0x00, 0x32, //  3. Euler angles
	0x40, 0x10, 0x00, 0x32, //  4. Delta V
	0x40, 0x20, 0x00, 0x32, //  5. Acceleration
	0x40, 0x30, 0x00, 0x32, //  6. Free acceleration
	0x80, 0x20, 0x00, 0x32, //  8. Rate of turn
	0x80, 0x30, 0x00, 0x32, //  7. Delta Q
	0xC0, 0x20, 0x00, 0x32, //  9. Magnetic field
	0xE0, 0x10, 0x00, 0x32, // 10. Status byte
	0xF5 // Checksum
};

// Static variables
static uint8_t rxmsg[256] = { 0 };
static uint8_t txmsg[256] = { 0 };

// Utility functions

/* Get current microsecond */
static uint32_t get_us(void)
{
	uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
	register uint32_t ms, cycle_cnt;
	do {
		ms = HAL_GetTick();
		cycle_cnt = SysTick->VAL;
	} while (ms != HAL_GetTick());
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

/* Delay for microseconds */
static void delay_us(uint16_t micros)
{
	uint32_t start = get_us();
	while (get_us() - start < (uint32_t)micros) {
		asm("nop");
	}
}

/* Calculate checksum for a given mti mesage */
uint8_t mti_checksum(MTiMsg* msg)
{
	uint8_t sum = 0xFF + msg->mid + msg->len;

	for (uint8_t i = 0; i < msg->len; i++)
		sum += msg->data[i];

	return ((uint8_t)~sum) + 1;
}

/* Send a message to the MTi over SPI */
void mti_send_message(const uint8_t* msg)
{
	HAL_GPIO_WritePin(MTi_CS_GPIO_Port, MTi_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, msg, 7 + msg[5], 100);
	HAL_GPIO_WritePin(MTi_CS_GPIO_Port, MTi_CS_Pin, GPIO_PIN_SET);
}

/* Receive a message from the MTi, over SPI */
void mti_receive_message()
{
	uint8_t rxPipeBuf[8] = { 0 };
	uint16_t notification_size = 0, measurement_size = 0;
	MTiMsg msg;

	// Set the pipe
	txmsg[0] = 0x04;

	// Read message size
	HAL_GPIO_WritePin(MTi_CS_GPIO_Port, MTi_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, txmsg, rxPipeBuf, 8, 100);
	HAL_GPIO_WritePin(MTi_CS_GPIO_Port, MTi_CS_Pin, GPIO_PIN_SET);

	// Figure out whether message is notification or measurement based
	// on size information.
	notification_size = (rxPipeBuf[4] | rxPipeBuf[5] << 8);
	measurement_size = (rxPipeBuf[6] | rxPipeBuf[7] << 8);
	txmsg[0] = (notification_size != 0 ? 0x05 : 0x06);

	// Read message
	HAL_GPIO_WritePin(MTi_CS_GPIO_Port, MTi_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, txmsg, rxmsg, (notification_size != 0 ? notification_size : measurement_size) + 3, 100);
	HAL_GPIO_WritePin(MTi_CS_GPIO_Port, MTi_CS_Pin, GPIO_PIN_SET);

	// Call function to handle message
	msg.mid = rxmsg[3];
	msg.len = rxmsg[4];

	if (msg.len != 0)
		msg.data = &rxmsg[5];
	else
		msg.data = NULL;

	mti_handle_message(&msg);
}

/* Handle a received message */
void mti_handle_message(MTiMsg* msg)
{
	switch (msg->mid) {
	case MID_WAKEUP:
		mti_send_message(MSG_WAKEUP_ACK);
		delay_us(10);
		mti_send_message(MSG_SET_OUTPUT_CONFIGURATION);
		//Send_serial_message("WAKEUP ACK, SET_OUTPUT\n\r");
		break;

	case MID_SET_OUTPUT_CONFIGURATION_ACK:
		SD_Save_Data("MTi_DATA,PacketCounter,SampleTime,EulerX,EulerY,EulerZ,DeltaVX,DeltaVY,DeltaVZ,AccX,AccY,AccZ,FreeAccX,FreeAccY,FreeAccZ,RoTX,RoTY,RoTZ,DeltaQ1,DeltaQ2,DeltaQ3,DeltaQ4,MagFieldX,MagFieldY,MagFieldZ,Status");
		delay_us(10);
		mti_send_message(MSG_SET_SYNC_SETTINGS);
		//Send_serial_message("SET_OUTPUT_ACK, SET_SYNC\n\r");
		break;

	case MID_MTDATA2:
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		mti_handle_mtdata2(msg);
		break;

	case MID_SET_SYNC_SETTINGS_ACK:
		mti_send_message(MSG_GOTO_MEASUREMENT);
		//Send_serial_message("SET_SYNC_ACK, GOTO_MEASUREMENT\n\r");
		break;

	case MID_GOTO_MEASUREMENT_ACK:
		//Send_serial_message("GOTO_MEASUREMENT_ACK\n\r");
		break;

	default:
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		char errcode[32] = { 0 };
		sprintf(errcode, "MTi is Kaput : %02x", msg->mid);
		break;
	}
}

/* Handle a MTData 2 message */
void mti_handle_mtdata2(MTiMsg* msg)
{
	uint8_t* data = msg->data;

	char str[512] = { 0 };

	uint8_t data_count = 0;
	uint16_t dataid = 0;
	uint32_t compound = 0;

	if (msg->data == NULL)
		return; // ABANDON THREAD!

	// Convert to CSV
	strcat(str, "MTi_DATA,");
	for (uint8_t i = 0; i < msg->len; i++) {
		dataid = (data[i] << 8) | data[i + 1];

		// Packet counter
		if (dataid == 0x1020) {
			data_count++;
			compound = ((data[i + 3] << 8) | (data[i + 3 + 1] << 0));
			sprintf(&str[strlen(str)], "%05lu,", compound);
		}
		// Sample time fine
		else if (dataid == 0x1060) {
			data_count++;
			compound = (data[i + 3 + 0] << 24) | (data[i + 3 + 1] << 16) | (data[i + 3 + 2] << 8) | (data[i + 3 + 3] << 0);
			sprintf(&str[strlen(str)], "%010lu,", compound);
		}
		// Status byte
		else if (dataid == 0xe010) {
			data_count++;
			sprintf(&str[strlen(str)], "0x%02x,", data[i + 3]);
		}
		// Anything else is a series of floats
		else {
			data_count++;
			for (uint8_t j = 0; j < data[i + 2]; j += 4) {
				compound = (data[i + 3 + j + 0] << 24) | (data[i + 3 + j + 1] << 16) | (data[i + 3 + j + 2] << 8) | (data[i + 3 + j + 3] << 0);
				sprintf(&str[strlen(str)], "%+012.8f,", ((fbit)compound).val);
			}
		}
		i += 2 + data[i + 2];
	}
	str[strlen(str) - 1] = '\n';

	// Erroneous message : the number of data packets sent did not match the number expected.
	// The packet is flushed.
	if (data_count != 10)
		return;

	//write to sd card
	SD_Save_Data(str);
}
