#ifndef __MTI_H_
#define __MTI_H_

/* MTi Message IDs */
#define MID_WAKEUP 0x3E
#define MID_SET_OUTPUT_CONFIGURATION_ACK 0xC1
#define MID_SET_SYNC_SETTINGS_ACK 0x2D
#define MID_GOTO_MEASUREMENT_ACK 0x11
#define MID_REQ_DATA_ACK 0x35
#define MID_MTDATA2 0x36

#include <stdint.h>

/* Data structures */
typedef struct {
	uint8_t mid;
	uint8_t len;
	uint8_t* data;
} MTiMsg;

/* Bit reinterpret unions */
typedef union {
	uint32_t bit;
	float val;
} fbit;

static const uint8_t MSG_REQ_DATA[] = { 0x03, 0xFF, 0xFF, 0xFF, 0x34, 0x00, 0xCD };

/* Prototypes */
uint8_t mti_checksum(uint8_t* message, uint8_t len);
void mti_send_message(const uint8_t* msg);
void mti_handle_message(MTiMsg* msg);
void mti_handle_mtdata2(MTiMsg* msg);

#endif
