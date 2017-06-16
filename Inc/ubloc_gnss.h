/****************************************************************************
 *
 *   Copyright (c) 2012, 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 
/*******************************************************************************
  * File Name          : ubloc_gnss.h
  * Date               : 26/06/2015
  * Description        : the u-bloc gnss library
  ******************************************************************************
*/


#ifndef UBX_H_
#define UBX_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <time.h>
#include <assert.h>
#include <string.h>
#include "SD_save.h"
//#include "serial_com.h"

#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
#define SWAP16(X)	((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))
#define COUNT_ACK 5
#define COUNT_TRY 5
#define FNV1_32_INIT	((uint32_t)0x811c9dc5)	// init value for FNV1 hash algorithm
#define FNV1_32_PRIME	((uint32_t)0x01000193)	// magic prime for FNV1 hash algorithm

#define M_DEG_TO_RAD_F 	0.01745329251994f
#define M_RAD_TO_DEG_F 	57.2957795130823f

#define GPS_EPOCH_SECS 1234567890ULL

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_SYNC ((UBX_SYNC1) | UBX_SYNC2 << 8)

/**< NAV-SVINFO Len : 8+12*SAT_INFO_MAX_SATELLITES = 152*/
/**< RXM-RAWX		Len	:	16+32*RAW_MAX_MEASURE 			 = 208*/
/**< RXM-SFRBX	Len	:	8+4*SUBFRAME_MAX_WORD        = 48*/

#define MAX_FRAME_LEN 208//164 //The maximum payload size for firmware 2.01 is 164 bytes
#define ACK_FRAME_LEN 10
#define NAV_SOL_FRAME_LEN 	 60
#define NAL_POSLLH_FRAME_LEN 36
#define NAV_VALNED_FRAME_LEN 44
#define NAV_TIMEUTC_FRAME_LEN 28
#define NAV_SVINFO_FRAME_LEN 	166//8+12*nbsat =248+ 8
#define RXM_RAWX_FRAME_LEN		208+8 // 8 is the header frame
#define RXM_SFRBX_FRAME_LEN		(48+8)*9
#define NAV_PVT_FRAME_LEN			100

//#define RECEIVE_COUNT (ACK_FRAME_LEN + NAV_SOL_FRAME_LEN + NAL_POSLLH_FRAME_LEN + NAV_VALNED_FRAME_LEN + NAV_TIMEUTC_FRAME_LEN + NAV_SVINFO_FRAME_LEN + RXM_RAWX_FRAME_LEN + RXM_SFRBX_FRAME_LEN) *2

//#define RECEIVE_COUNT	(ACK_FRAME_LEN + NAV_PVT_FRAME_LEN+NAV_SVINFO_FRAME_LEN+ RXM_RAWX_FRAME_LEN+ RXM_SFRBX_FRAME_LEN)*2
#define RECEIVE_COUNT (NAV_PVT_FRAME_LEN + NAV_TIMEUTC_FRAME_LEN)*2
#define TX_CFG_MSG_LEN 3

#define SAT_INFO_MAX_SATELLITES  12
#define RAW_MAX_MEASURE 6
#define SUBFRAME_MAX_WORD	10

#define UBX_HEADER_SIZE 6

#define WAIT_ACK_TIME 1000

/* Message Classes */
#define UBX_CLASS_NAV		0x01
#define UBX_CLASS_RXM		0x02
#define UBX_CLASS_ACK		0x05
#define UBX_CLASS_CFG		0x06
#define UBX_CLASS_MON		0x0A

/* Message IDs */
#define UBX_ID_NAV_POSLLH	0x02
#define UBX_ID_NAV_SOL		0x06
#define UBX_ID_NAV_PVT		0x07
#define UBX_ID_NAV_VELNED	0x12
#define UBX_ID_NAV_TIMEUTC 0x21
#define UBX_ID_NAV_SVINFO	0x30
#define UBX_ID_ACK_NAK		0x00
#define UBX_ID_ACK_ACK		0x01
#define UBX_ID_CFG_PRT		0x00
#define UBX_ID_CFG_MSG		0x01
#define UBX_ID_CFG_RATE		0x08
#define UBX_ID_CFG_NAV5		0x24
#define UBX_ID_CFG_SBAS		0x16
#define UBX_ID_MON_VER		0x04
#define UBX_ID_MON_HW			0x09

#define UBX_ID_RXM_RAWX		0x15
#define UBX_ID_RXM_SFRBX	0x13

/* Message Classes & IDs */
#define UBX_MSG_NAV_POSLLH	((UBX_CLASS_NAV) | UBX_ID_NAV_POSLLH << 8)
#define UBX_MSG_NAV_SOL		((UBX_CLASS_NAV) | UBX_ID_NAV_SOL << 8)
#define UBX_MSG_NAV_PVT		((UBX_CLASS_NAV) | UBX_ID_NAV_PVT << 8)
#define UBX_MSG_NAV_VELNED	((UBX_CLASS_NAV) | UBX_ID_NAV_VELNED << 8)
#define UBX_MSG_NAV_TIMEUTC	((UBX_CLASS_NAV) | UBX_ID_NAV_TIMEUTC << 8)
#define UBX_MSG_NAV_SVINFO	((UBX_CLASS_NAV) | UBX_ID_NAV_SVINFO << 8)
#define UBX_MSG_ACK_NAK		((UBX_CLASS_ACK) | UBX_ID_ACK_NAK << 8)
#define UBX_MSG_ACK_ACK		((UBX_CLASS_ACK) | UBX_ID_ACK_ACK << 8)
#define UBX_MSG_CFG_PRT		((UBX_CLASS_CFG) | UBX_ID_CFG_PRT << 8)
#define UBX_MSG_CFG_MSG		((UBX_CLASS_CFG) | UBX_ID_CFG_MSG << 8)
#define UBX_MSG_CFG_RATE	((UBX_CLASS_CFG) | UBX_ID_CFG_RATE << 8)
#define UBX_MSG_CFG_NAV5	((UBX_CLASS_CFG) | UBX_ID_CFG_NAV5 << 8)
#define UBX_MSG_CFG_SBAS	((UBX_CLASS_CFG) | UBX_ID_CFG_SBAS << 8)
#define UBX_MSG_MON_HW		((UBX_CLASS_MON) | UBX_ID_MON_HW << 8)
#define UBX_MSG_MON_VER		((UBX_CLASS_MON) | UBX_ID_MON_VER << 8)

#define UBX_MSG_RXM_RAWX	 ((UBX_CLASS_RXM) | UBX_ID_RXM_RAWX << 8)
#define UBX_MSG_RXM_SFRBX	 ((UBX_CLASS_RXM) | UBX_ID_RXM_SFRBX <<8)


/* RX NAV-PVT message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_PVT_VALID_VALIDDATE		0x01	/**< validDate (Valid UTC Date) */
#define UBX_RX_NAV_PVT_VALID_VALIDTIME		0x02	/**< validTime (Valid UTC Time) */
#define UBX_RX_NAV_PVT_VALID_FULLYRESOLVED	0x04	/**< fullyResolved (1 = UTC Time of Day has been fully resolved (no seconds uncertainty)) */

/*   Bitfield "flags" masks */
#define UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK		0x01	/**< gnssFixOK (A valid fix (i.e within DOP & accuracy masks)) */
#define UBX_RX_NAV_PVT_FLAGS_DIFFSOLN		0x02	/**< diffSoln (1 if differential corrections were applied) */
#define UBX_RX_NAV_PVT_FLAGS_PSMSTATE		0x1C	/**< psmState (Power Save Mode state (see Power Management)) */
#define UBX_RX_NAV_PVT_FLAGS_HEADVEHVALID	0x20	/**< headVehValid (Heading of vehicle is valid) */

/* RX NAV-TIMEUTC message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDTOW	0x01	/**< validTOW (1 = Valid Time of Week) */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDKWN	0x02	/**< validWKN (1 = Valid Week Number) */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC	0x04	/**< validUTC (1 = Valid UTC Time) */
#define UBX_RX_NAV_TIMEUTC_VALID_UTCSTANDARD	0xF0	/**< utcStandard (0..15 = UTC standard identifier) */

/* TX CFG-PRT message contents */
#define UBLOC_UART1 0x01
#define UBLOC_I2C		0x00
#define UBLOC_UART2	0x02
#define UBLOC_USB		0x03
#define UBLOC_SPI		0x04
#define UBX_TX_CFG_PRT_PORTID		0x01		/**< UART1 */
#define UBX_TX_CFG_PRT_MODE		0x000008D0	/**< 0b0000100011010000: 8N1 */// 
#define UBX_TX_CFG_PRT_BAUDRATE		115200		/**< choose 115200 as GPS baudrate */
#define UBX_TX_CFG_PRT_INPROTOMASK	0x01		/**< UBX in */
#define UBX_TX_CFG_PRT_OUTPROTOMASK	0x01		/**< UBX out */

/* TX CFG-RATE message contents */
#define UBX_REF_TIME_LOCAL 2
#define UBX_REF_TIME_GPS 	 1
#define UBX_REF_TIME_UTC 	 0

#define UBX_TX_CFG_RATE_MEASINTERVAL	200		/**< 200ms for 5Hz  / or 1000 1Hz*/
#define UBX_TX_CFG_RATE_NAVRATE		1		/**< cannot be changed */
#define UBX_TX_CFG_RATE_TIMEREF		1		/**< 0: UTC, 1: GPS time */

/* TX CFG-NAV5 message contents */
#define DYNMODEL_PORTABLE 0
#define DYNMODEL_STATIONARY 2
#define DYNMODEL_PEDESTRIAN 3
#define DYNMODEL_AUTOMOTIVE 4
#define DYNMODEL_SEA				5
#define DYNMODEL_AIRBORNE_1G	6
#define DYNMODEL_AIRBORNE_2G	7
#define DYNMODEL_AIRBORNE_4G	8


#define FIXMODE_2D 1
#define FIXMODE_3D 2
#define FIXMODE_2D3D 3

#define UBX_TX_CFG_NAV5_MASK		0x0005		/**< Only update dynamic model and fix mode */
#define UBX_TX_CFG_NAV5_DYNMODEL	4		/**< 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne <4g */
#define UBX_TX_CFG_NAV5_FIXMODE		2		/**< 1 2D only, 2 3D only, 3 Auto 2D/3D */



/* TX CFG-SBAS message contents */
#define UBX_TX_CFG_SBAS_MODE_ENABLED	1				/**< SBAS enabled */
#define UBX_TX_CFG_SBAS_MODE_DISABLED	0				/**< SBAS disabled */
#define UBX_TX_CFG_SBAS_MODE		UBX_TX_CFG_SBAS_MODE_ENABLED	/**< SBAS enabled or disabled */



/* TX CFG-MSG message contents */
#define UBX_TX_CFG_MSG_RATE1_5HZ	0x01 		/**< {0x00, 0x01, 0x00, 0x00, 0x00, 0x00} the second entry is for UART1 */
#define UBX_TX_CFG_MSG_RATE1_1HZ	0x05		/**< {0x00, 0x05, 0x00, 0x00, 0x00, 0x00} the second entry is for UART1 */
#define UBX_TX_CFG_MSG_RATE1_05HZ	10


/* General: Header */

typedef	union {
	uint8_t headerBytes[UBX_HEADER_SIZE];
	struct {
	uint16_t	sync;
	uint16_t	msg;
	uint16_t	length;
	}headerField;
} ubx_header_t;

typedef struct {
	uint8_t		ck_a;
	uint8_t		ck_b;
} ubx_checksum_t ;



/* Rx NAV-POSLLH */
typedef struct {
	uint32_t	iTOW;		/**< GPS Time of Week [ms] */
	int32_t		lon;		/**< Longitude [1e-7 deg] */
	int32_t		lat;		/**< Latitude [1e-7 deg] */
	int32_t		height;		/**< Height above ellipsoid [mm] */
	int32_t		hMSL;		/**< Height above mean sea level [mm] */
	uint32_t	hAcc;  		/**< Horizontal accuracy estimate [mm] */
	uint32_t	vAcc;  		/**< Vertical accuracy estimate [mm] */
} ubx_payload_rx_nav_posllh_t;

/* Rx NAV-SOL */
typedef struct {
	uint32_t	iTOW;		/**< GPS Time of Week [ms] */
	int32_t		fTOW;		/**< Fractional part of iTOW (range: +/-500000) [ns] */
	int16_t		week;		/**< GPS week */
	uint8_t		gpsFix;		/**< GPSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GPS + dead reckoning, 5 = time only fix */
	uint8_t		flags;
	int32_t		ecefX;
	int32_t		ecefY;
	int32_t		ecefZ;
	uint32_t	pAcc;
	int32_t		ecefVX;
	int32_t		ecefVY;
	int32_t		ecefVZ;
	uint32_t	sAcc;
	uint16_t	pDOP;
	uint8_t		reserved1;
	uint8_t		numSV;		/**< Number of SVs used in Nav Solution */
	uint32_t	reserved2;
} ubx_payload_rx_nav_sol_t;

/* Rx NAV-PVT (ubx8) */
typedef struct {
	uint32_t	iTOW;		/**< GPS Time of Week [ms] */
	uint16_t	year; 		/**< Year (UTC)*/
	uint8_t		month; 		/**< Month, range 1..12 (UTC) */
	uint8_t		day; 		/**< Day of month, range 1..31 (UTC) */
	uint8_t		hour; 		/**< Hour of day, range 0..23 (UTC) */
	uint8_t		min; 		/**< Minute of hour, range 0..59 (UTC) */
	uint8_t		sec;		/**< Seconds of minute, range 0..60 (UTC) */
	uint8_t		valid; 		/**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
	uint32_t	tAcc; 		/**< Time accuracy estimate (UTC) [ns] */
	int32_t		nano;		/**< Fraction of second (UTC) [-1e9...1e9 ns] */
	uint8_t		fixType;	/**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	uint8_t		flags;		/**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
	uint8_t		reserved1;
	uint8_t		numSV;		/**< Number of SVs used in Nav Solution */
	int32_t		lon;		/**< Longitude [1e-7 deg] */
	int32_t		lat;		/**< Latitude [1e-7 deg] */
	int32_t		height;		/**< Height above ellipsoid [mm] */
	int32_t		hMSL;		/**< Height above mean sea level [mm] */
	uint32_t	hAcc;  		/**< Horizontal accuracy estimate [mm] */
	uint32_t	vAcc;  		/**< Vertical accuracy estimate [mm] */
	int32_t		velN;		/**< NED north velocity [mm/s]*/
	int32_t		velE;		/**< NED east velocity [mm/s]*/
	int32_t		velD;		/**< NED down velocity [mm/s]*/
	int32_t		gSpeed;		/**< Ground Speed (2-D) [mm/s] */
	int32_t		headMot;	/**< Heading of motion (2-D) [1e-5 deg] */
	uint32_t	sAcc;		/**< Speed accuracy estimate [mm/s] */
	uint32_t	headAcc;	/**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
	uint16_t	pDOP;		/**< Position DOP [0.01] */
	uint16_t	reserved2;
	uint32_t	reserved3;
	int32_t		headVeh;	/**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
	uint32_t	reserved4;	/**< (ubx8+ only) */
} ubx_payload_rx_nav_pvt_t;


#define UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX7	(sizeof(ubx_payload_rx_nav_pvt_t) - 8)
#define UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8	(sizeof(ubx_payload_rx_nav_pvt_t))

/* Rx NAV-TIMEUTC */
typedef struct {
	uint32_t	iTOW;		/**< GPS Time of Week [ms] */
	uint32_t	tAcc; 		/**< Time accuracy estimate (UTC) [ns] */
	int32_t		nano;		/**< Fraction of second, range -1e9 .. 1e9 (UTC) [ns] */
	uint16_t	year; 		/**< Year, range 1999..2099 (UTC) */
	uint8_t		month; 		/**< Month, range 1..12 (UTC) */
	uint8_t		day; 		/**< Day of month, range 1..31 (UTC) */
	uint8_t		hour; 		/**< Hour of day, range 0..23 (UTC) */
	uint8_t		min; 		/**< Minute of hour, range 0..59 (UTC) */
	uint8_t		sec;		/**< Seconds of minute, range 0..60 (UTC) */
	uint8_t		valid; 		/**< Validity Flags (see UBX_RX_NAV_TIMEUTC_VALID_...) */
} ubx_payload_rx_nav_timeutc_t;

/* Rx NAV-SVINFO Part 1 */
typedef struct {
	uint32_t	iTOW;		/**< GPS Time of Week [ms] */
	uint8_t		numCh; 		/**< Number of channels */
	uint8_t		globalFlags;
	uint16_t	reserved2;
} ubx_payload_rx_nav_svinfo_part1_t;

/* Rx NAV-SVINFO Part 2 (repeated) */
typedef struct {
	uint8_t		chn; 		/**< Channel number, 255 for SVs not assigned to a channel */
	uint8_t		svid; 		/**< Satellite ID */
	uint8_t		flags;
	uint8_t		quality;
	uint8_t		cno;		/**< Carrier to Noise Ratio (Signal Strength) [dbHz] */
	int8_t		elev; 		/**< Elevation [deg] */
	int16_t		azim; 		/**< Azimuth [deg] */
	int32_t		prRes; 		/**< Pseudo range residual [cm] */
} ubx_payload_rx_nav_svinfo_part2_t;


/* Rx RXM-RAWX Part 1 */
typedef struct {
	double		rcvTow;		/**< Measurement Time of Week [s] */
	uint16_t	week; 		/**< GPS week number */
	int8_t 		leapS;
	uint8_t		numMeas;
	uint8_t		recStat;
	uint8_t 	reserved[3];
} ubx_payload_rx_rxm_rawx_part1_t;

/* Rx RXM-RAWX Part 2 (repeated) */
typedef struct {
	double		prMes; 		/**< Pseudorange measurement[m]. GLONASS inter frequency channel. */
	double		cpMes; 		/**< Carrier phase measurement [cycle] */
	float			doMes;		/**< Doppler measurement [Hz]**/
	uint8_t		gnssId;
	uint8_t		svId;			/**< Satellite identifier */
	int8_t		freqId; 	/**< frequency slot  */
	int16_t		locktime; /**< Carrier phase locktime [ms]*/
	uint8_t		cno; 			/**< carrier-to-noise density ratio [dB-Hz] */
	int8_t		prStdev;		/**< Estimated pseudorange measurement standard deviation [m] (scaling 0.01*2^n)*/
	int8_t		cpStdev;		/**< Estimated carrier phase measurement standard deviation [cycle] (scaling 0.002*2^n)*/
	int8_t		trkStat;		/**< Tracking status bitfield>*/
	uint8_t		reserved;
} ubx_payload_rx_rxm_rawx_part2_t;


/* Rx RXM-SFRBX Part 1 */
typedef struct {
	uint8_t		gnssId;		/**< Measurement Time of Week [s] */
	uint8_t		svId; 		/**< GPS week number */
	uint8_t 	reserved1;
	uint8_t		freqId;
	uint8_t		numWords;
	uint8_t 	reserved2;
	uint8_t		version;
	uint8_t		reserved3;
} ubx_payload_rx_rxm_sfrbx_part1_t;

/* Rx RXM-SFRBX Part 2 (repeated) */
typedef struct {
	uint32_t	dwrd[SUBFRAME_MAX_WORD];
} ubx_payload_rx_rxm_sfrbx_part2_t;

/* Rx NAV-VELNED */
typedef struct {
	uint32_t	iTOW;		/**< GPS Time of Week [ms] */
	int32_t		velN;		/**< North velocity component [cm/s]*/
	int32_t		velE;		/**< East velocity component [cm/s]*/
	int32_t		velD;		/**< Down velocity component [cm/s]*/
	uint32_t	speed;		/**< Speed (3-D) [cm/s] */
	uint32_t	gSpeed;		/**< Ground speed (2-D) [cm/s] */
	int32_t		heading;	/**< Heading of motion 2-D [1e-5 deg] */
	uint32_t	sAcc;		/**< Speed accuracy estimate [cm/s] */
	uint32_t	cAcc;		/**< Course / Heading accuracy estimate [1e-5 deg] */
} ubx_payload_rx_nav_velned_t;

/* Rx MON-HW (ubx6) */
typedef struct {
	uint32_t	pinSel;
	uint32_t	pinBank;
	uint32_t	pinDir;
	uint32_t	pinVal;
	uint16_t	noisePerMS;
	uint16_t	agcCnt;
	uint8_t		aStatus;
	uint8_t		aPower;
	uint8_t		flags;
	uint8_t		reserved1;
	uint32_t	usedMask;
	uint8_t		VP[25];
	uint8_t		jamInd;
	uint16_t	reserved3;
	uint32_t	pinIrq;
	uint32_t	pullH;
	uint32_t	pullL;
} ubx_payload_rx_mon_hw_ubx6_t;

/* Rx MON-HW (ubx7+) */
typedef struct {
	uint32_t	pinSel;
	uint32_t	pinBank;
	uint32_t	pinDir;
	uint32_t	pinVal;
	uint16_t	noisePerMS;
	uint16_t	agcCnt;
	uint8_t		aStatus;
	uint8_t		aPower;
	uint8_t		flags;
	uint8_t		reserved1;
	uint32_t	usedMask;
	uint8_t		VP[17];
	uint8_t		jamInd;
	uint16_t	reserved3;
	uint32_t	pinIrq;
	uint32_t	pullH;
	uint32_t	pullL;
} ubx_payload_rx_mon_hw_ubx7_t;

/* Rx MON-VER Part 1 */
typedef struct {
	uint8_t		swVersion[30];
	uint8_t		hwVersion[10];
} ubx_payload_rx_mon_ver_part1_t;

/* Rx MON-VER Part 2 (repeated) */
typedef struct {
	uint8_t		extension[30];
} ubx_payload_rx_mon_ver_part2_t;

/* Rx ACK-ACK */
typedef	union {
	uint16_t	msgByte;
	struct {
		uint8_t	clsID;
		uint8_t	msgID;
	}msgField;
} ubx_payload_rx_ack_ack_t;

/* Rx ACK-NAK */
typedef	union {
	uint16_t	msgByte;
	struct {
		uint8_t	clsID;
		uint8_t	msgID;
	}msgField;
} ubx_payload_rx_ack_nak_t;

/* Tx CFG-PRT */
typedef struct {
	uint8_t		portID;
	uint8_t		reserved0;
	uint16_t	txReady;
	uint32_t	mode;
	uint32_t	baudRate;
	uint16_t	inProtoMask;
	uint16_t	outProtoMask;
	uint16_t	flags;
	uint16_t	reserved5;
} ubx_payload_tx_cfg_prt_t;

/* Tx CFG-RATE */
typedef struct {
	uint16_t	measRate;	/**< Measurement Rate, GPS measurements are taken every measRate milliseconds */
	uint16_t	navRate;	/**< Navigation Rate, in number of measurement cycles. This parameter cannot be changed, and must be set to 1 */
	uint16_t	timeRef;	/**< Alignment to reference time: 0 = UTC time, 1 = GPS time */
} ubx_payload_tx_cfg_rate_t;

/* Tx CFG-NAV5 */
typedef struct {
	uint16_t	mask;
	uint8_t		dynModel;	/**< Dynamic Platform model: 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne <4g */
	uint8_t		fixMode;	/**< Position Fixing Mode: 1 2D only, 2 3D only, 3 Auto 2D/3D */
	int32_t		fixedAlt;
	uint32_t	fixedAltVar;
	int8_t		minElev;
	uint8_t		drLimit;
	uint16_t	pDop;
	uint16_t	tDop;
	uint16_t	pAcc;
	uint16_t	tAcc;
	uint8_t		staticHoldThresh;
	uint8_t		dgpsTimeOut;
	uint8_t		cnoThreshNumSVs;	/**< (ubx7+ only, else 0) */
	uint8_t		cnoThresh;		/**< (ubx7+ only, else 0) */
	uint16_t	reserved;
	uint16_t	staticHoldMaxDist;	/**< (ubx8+ only, else 0) */
	uint8_t		utcStandard;		/**< (ubx8+ only, else 0) */
	uint8_t		reserved3;
	uint32_t	reserved4;
} ubx_payload_tx_cfg_nav5_t;

/* tx cfg-sbas */
typedef struct {
	uint8_t		mode;
	uint8_t		usage;
	uint8_t		maxSBAS;
	uint8_t		scanmode2;
	uint32_t	scanmode1;
} ubx_payload_tx_cfg_sbas_t;

/* Tx CFG-MSG */
typedef struct {
	union {
		uint16_t	msgByte;
		struct {
			uint8_t	msgClass;
			uint8_t	msgID;
		}msgField;
	}msg;
	uint8_t rate;
} ubx_payload_tx_cfg_msg_t;

#define NO_FIX 1// greater than 1 then there is a FIX
// the struct that defines all the needed data to receive form GPS
struct vehicle_gps_position_s {
	uint32_t timestamp_position;			/**< Timestamp for position information */
	int32_t lat;					/**< Latitude in 1E-7 degrees */
	int32_t lon;					/**< Longitude in 1E-7 degrees */
	int32_t alt;					/**< Altitude in 1E-3 meters (millimeters) above MSL  */

	uint64_t timestamp_variance;
	float s_variance_m_s;				/**< speed accuracy estimate m/s */
	float c_variance_rad;				/**< course accuracy estimate rad */
	uint8_t fix_type; 				/**< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.   */

	float eph;					/**< GPS HDOP horizontal dilution of position in m */
	float epv;					/**< GPS VDOP horizontal dilution of position in m */
	float dop;                /**< GPS DOP position dilution of position in m */

	unsigned noise_per_ms;				/**< */
	unsigned jamming_indicator;			/**< */

	uint64_t timestamp_velocity;			/**< Timestamp for velocity informations */
	float vel_m_s;					/**< GPS ground speed (m/s) */
	float vel_n_m_s;				/**< GPS ground speed in m/s */
	float vel_e_m_s;				/**< GPS ground speed in m/s */
	float vel_d_m_s;				/**< GPS ground speed in m/s */
	float cog_rad;					/**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
	bool vel_ned_valid;				/**< Flag to indicate if NED speed is valid */

	uint32_t timestamp_time;			/**< Timestamp for time information */
	uint32_t time_utc_usec;				/**< Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0 */

	uint8_t satellites_used;			/**< Number of satellites used */
};

//struct tha t defines the satellite info when enable
struct satellite_info_s {
	uint64_t timestamp;				/**< Timestamp of satellite info */
	uint8_t count;					/**< Number of satellites in satellite info */
	uint8_t svid[SAT_INFO_MAX_SATELLITES]; 		/**< Space vehicle ID [1..255], see scheme below  */
	uint8_t used[SAT_INFO_MAX_SATELLITES];		/**< 0: Satellite not used, 1: used for navigation */
	uint8_t elevation[SAT_INFO_MAX_SATELLITES];	/**< Elevation (0: right on top of receiver, 90: on the horizon) of satellite */
	uint8_t azimuth[SAT_INFO_MAX_SATELLITES];	/**< Direction of satellite, 0: 0 deg, 255: 360 deg. */
	uint8_t snr[SAT_INFO_MAX_SATELLITES];		/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite. */
};

struct gnss_raw_meas_s{
	uint8_t numMeas;
	double 	 prMes[RAW_MAX_MEASURE];//pseudo range
	double 	 cpMes[RAW_MAX_MEASURE];//carrier
	float		 doMes[RAW_MAX_MEASURE];//doppler
	uint8_t	 gnssId[RAW_MAX_MEASURE];
	uint8_t	 svId[RAW_MAX_MEASURE];
	uint16_t locktime[RAW_MAX_MEASURE];
	uint8_t	 cno[RAW_MAX_MEASURE];
};

struct raw_subframe_s{
	uint8_t	 numWord;
	uint8_t	 gnssId;
	uint8_t	 svId;
	uint8_t	 freqId;
	uint32_t dwrd[SUBFRAME_MAX_WORD];
};

/* General message and payload buffer union */
typedef union {
	ubx_payload_rx_nav_pvt_t					payload_rx_nav_pvt;
	ubx_payload_rx_nav_posllh_t				payload_rx_nav_posllh;
	ubx_payload_rx_nav_sol_t					payload_rx_nav_sol;
	ubx_payload_rx_nav_timeutc_t			payload_rx_nav_timeutc;
	ubx_payload_rx_nav_svinfo_part1_t	payload_rx_nav_svinfo_part1;
	ubx_payload_rx_nav_svinfo_part2_t	payload_rx_nav_svinfo_part2;
	ubx_payload_rx_rxm_rawx_part1_t	  payload_rx_rxm_rawx_part1;
	ubx_payload_rx_rxm_rawx_part2_t	  payload_rx_rxm_rawx_part2;
	ubx_payload_rx_rxm_sfrbx_part1_t	payload_rx_rxm_sfrbx_part1;
	ubx_payload_rx_rxm_sfrbx_part2_t	payload_rx_rxm_sfrbx_part2;
	ubx_payload_rx_nav_velned_t				payload_rx_nav_velned;
	ubx_payload_rx_mon_hw_ubx6_t			payload_rx_mon_hw_ubx6;
	ubx_payload_rx_mon_hw_ubx7_t			payload_rx_mon_hw_ubx7;
	ubx_payload_rx_mon_ver_part1_t		payload_rx_mon_ver_part1;
	ubx_payload_rx_mon_ver_part2_t		payload_rx_mon_ver_part2;
	ubx_payload_rx_ack_ack_t					payload_rx_ack_ack;
	ubx_payload_rx_ack_nak_t					payload_rx_ack_nak;
	ubx_payload_tx_cfg_prt_t					payload_tx_cfg_prt;
	ubx_payload_tx_cfg_rate_t					payload_tx_cfg_rate;
	ubx_payload_tx_cfg_nav5_t					payload_tx_cfg_nav5;
	ubx_payload_tx_cfg_sbas_t					payload_tx_cfg_sbas;
	ubx_payload_tx_cfg_msg_t					payload_tx_cfg_msg;
	uint8_t														raw[MAX_FRAME_LEN];
} ubx_buf_t;

//#pragma pack(pop)
/*** END OF u-blox protocol binary message and payload definitions ***/

/* Decoder state */
typedef enum {
	UBX_DECODE_SYNC1 = 0,
	UBX_DECODE_SYNC2,
	UBX_DECODE_CLASS,
	UBX_DECODE_ID,
	UBX_DECODE_LENGTH1,
	UBX_DECODE_LENGTH2,
	UBX_DECODE_PAYLOAD,
	UBX_DECODE_CHKSUM1,
	UBX_DECODE_CHKSUM2
} ubx_decode_state_t;

/* Rx message state */
typedef enum {
	UBX_RXMSG_IGNORE = 0,
	UBX_RXMSG_HANDLE,
	UBX_RXMSG_DISABLE,
	UBX_RXMSG_ERROR_LENGTH
} ubx_rxmsg_state_t;

/* ACK state */
typedef enum {
	UBX_ACK_IDLE = 0,
	UBX_ACK_WAITING,
	UBX_ACK_GOT_ACK,
	UBX_ACK_GOT_NAK
} ubx_ack_state_t;


typedef enum{
	UBX_OK,
	UBX_CFG_ERROR_BAUD,
	UBX_CFG_FAIL_DETECT_BAUD,
	UBX_CFG_NO_RES_ACK, // waiting for ack no response
	UBX_ACK_SEM_ERROR,
	UBX_NO_RES,
	UBX_UART_ERR,
	UBX_ACK_OK
}UBX_StatusTypeDef;

typedef enum{
	NAV_PVT,
	NAV_POSLLH,
	NAV_SOL,
	NAV_TIMEUTC,
	NAV_SVINFO,
	NAV_VELNED,
	NAV_NO_FRAME
}UBX_FrameTypeDef;


typedef struct{
	//cfg-prt
	unsigned int baudrate;
	uint8_t 		 portID;
	//cfg-rate
	uint8_t			 MeasurementRate;
	uint16_t		 navigationRate;
	uint16_t 		 timeRef;
	//cfg-nav5
	uint8_t dynModel;
	uint8_t fixMode;
	/*...*/
}GNSS_InitTypeDef;



typedef struct{
	UART_HandleTypeDef*    				 	huart;
//	RTC_HandleTypeDef*							hrtc;
	GNSS_InitTypeDef								Init;
	UBX_StatusTypeDef								errorStatus;
	struct vehicle_gps_position_s* 	gps_position;
	struct satellite_info_s*       	satellite_info;
	struct gnss_raw_meas_s*					gnss_meas;
	struct raw_subframe_s*					subFrames;
	bool													 	enable_sat_info;
	bool			 											got_posllh;
	bool														got_velned;
	bool 														got_sol;
	bool														got_svinfo;
	bool														got_gnssMeas;
	bool														got_subFrame;
	UBX_FrameTypeDef								rec_frame;
	uint32_t 												time;
	uint32_t												ubx_version;
	bool														use_nav_pvt;
	bool 														use_sat_info;
	bool														use_subFrame;
	bool														use_gnss_meas;
}GNSS_HandleTypeDef;

//the big boss
typedef struct{
	GNSS_HandleTypeDef*							hgps;
	ubx_decode_state_t							decode_state;
	uint16_t												rx_msg;
	ubx_rxmsg_state_t								rx_state;
	uint16_t												rx_payload_length;
	uint16_t												rx_payload_index;
	uint8_t													rx_ck_a;
	uint8_t													rx_ck_b;
	uint16_t												ack_waiting_msg;
	bool													 	configured;
	bool														rtc_configured;
	ubx_ack_state_t								 	ack_state;
	ubx_buf_t												buf_packet;
	uint8_t 												buf_reception[RECEIVE_COUNT];
}UBX_ParserHandler;


//initialise the GPS handler
void GNSS_Init(UBX_ParserHandler *_Parser, UART_HandleTypeDef *_huart);
UBX_StatusTypeDef GNSS_Start(void);
UBX_StatusTypeDef GNSS_configure(UBX_ParserHandler* _Parser);//configure the GPS
void 			  GNSS_log(GNSS_HandleTypeDef *hgps);
int  parse_char(UBX_ParserHandler* _Parser, const uint8_t b);


#endif //UBX_H_
