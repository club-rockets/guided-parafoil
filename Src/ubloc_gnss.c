/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
/*************** some function and defines were taken from PX4 **************/


/*******************************************************************************
 * File Name          : ubloc_gnss.c
 * Date               : 26/06/2015
 * Description        : the u-bloc gnss library
 ******************************************************************************
 */

#include "ubloc_gnss.h"

static UBX_ParserHandler Parser;
uint8_t semGpsGate =0;
//extern DMA_HandleTypeDef hdma_usart1_rx;
//extern UART_HandleTypeDef huart1;
/** @addtogroup GNSS_Private_Functions   GNSS Private Functions
 * @{
 */
static void UBX_resetParser(void);
static void UBX_Parser_Init(void);
//static int  parse_char(const uint8_t b);
static void add_byte_to_checksum(const uint8_t b);
static int  payload_rx_init(void);
static int  payload_rx_add_nav_svinfo( const uint8_t b);

static int  payload_rx_add_rxm_rawx (const uint8_t b);
static int  payload_rx_add_rxm_sfrbx(const uint8_t b);

static int  payload_rx_add(const uint8_t b);
static int  payload_rx_add_mon_ver( const uint8_t b);
static uint32_t fnv1_32_str(uint8_t *str, uint32_t hval);
static int payload_rx_done(void);
static void configure_message_rate(const uint16_t msg, const uint8_t rate);
static void calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum);
static void GNSS_send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length);
static UBX_StatusTypeDef wait_for_ack(const uint16_t msg);
static UBX_StatusTypeDef wait_for_ack2(const uint16_t msg);
//UBX_StatusTypeDef GNSS_configure(void);//configure the GPS
/**
 * @}
 */


/**  
===============================================================================
            #####         Initialisation function          #####
 ===============================================================================  
    The u-bloc GNSS module is configure with UBX communication using a UART.
		The u-bloc GNSS module can be configure with the fallowing parameters
		- baudrate 			 :  the UART bauderate
		- portID   			 :  the Port Identifier Number
		- fixMode	 			 :  Position Fixing Mode:
												1: 2D only
												2: 3D only
												3: auto 2D/3D
		- MeasurementRate:  Measurement Rate, GPS measurements are
												taken every measRate milliseconds
		- navigationRate :	Navigation Rate, in number of measurement
												cycles. This parameter cannot be changed, and
												must be set to 1.
		- timeRef  			 :  Alignment to reference time
												0: UTC time
												1: GPS time
	The gnss struture also need the reference to a uart and a reference to the rtc.
 **/


/**
 * @brief  Initialise the parser
 * @param  hgps: pointer to a GNSS_HandleTypeDef structure that contains
 *                the configuration information for the specified GNSS module.
 * @retval None
 */


//void MX_GPS_Init(void)
//{
//	hgps.Init.baudrate  			= huart2.Init.BaudRate;
//	hgps.Init.portID 					= UBLOC_UART1;
//	hgps.Init.dynModel 				= DYNMODEL_AUTOMOTIVE;
//	hgps.Init.fixMode  				= FIXMODE_3D;
//	hgps.Init.MeasurementRate = UBX_TX_CFG_RATE_MEASINTERVAL;
//	hgps.Init.navigationRate  = UBX_TX_CFG_RATE_NAVRATE;
//	hgps.Init.timeRef					= UBX_REF_TIME_GPS;
//	hgps.huart                = &huart2;
//	hgps.hrtc									= &hrtc;
//	hgps.use_sat_info					= false;
//	hgps.use_gnss_meas				= true;
//	hgps.use_subFrame					= true;
//	GNSS_Init(&hgps);
//	if(GNSS_Start() != UBX_OK)
//		Error_Handler();
//}


void GNSS_Init(GNSS_HandleTypeDef *hgps)
{
	hgps->got_posllh= false;
	hgps->got_velned = false;
	hgps->got_svinfo = false;
	hgps->got_gnssMeas = false;
	hgps->got_subFrame	= false;
	hgps->ubx_version =0;
	UBX_Parser_Init();
	//	hgps->use_nav_pvt =false;
	//	hgps->use_sat_info=false;
	hgps->satellite_info = NULL;
	hgps->gnss_meas			 = NULL;
	hgps->subFrames			 = NULL;
	Parser.hgps = hgps;
}


/**
 * @brief  Start the dma reception packet
 *					from the GNSS module. Once this function is call the DMA start to copy the
 *					the frame from the gnss to the parser buffer. A half and complet recetion
 *					callback function is call to parse the data.
 * @retval GNSS status
 */
UBX_StatusTypeDef GNSS_Start(void)
{
	//start the DMA transmission ( continous reception with half and complete transmission callback)

	//	HAL_Delay(200);
	//	HAL_UART_DeInit(&huart1);
	//	__DMA2_CLK_DISABLE();
	//	HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
	//
	//
	//  __DMA2_CLK_ENABLE();
	//  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 2, 0);
	//  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	//	HAL_UART_Init(&huart1);

	if(HAL_UART_Receive_DMA(Parser.hgps->huart, Parser.buf_reception, RECEIVE_COUNT) != HAL_OK)
	{
		return UBX_UART_ERR;
	}
	// receive some junk
	int tickstart =HAL_GetTick();
	semGpsGate =0;
	while((semGpsGate < 1) && ((HAL_GetTick() - tickstart) < 1000))
	{
	}
	if(((HAL_GetTick() - tickstart) >= 1000) && (semGpsGate < 1))
	{
		return UBX_NO_RES;
	}
	return UBX_OK;
}


/**
 * @brief  configure the GNSS mode according to the specified parameters in
 *         the GNSS_InitTypeDef. Find the correct baudrate by itself. For now
 *					only the NAV-SOL, NAV-POSLLH, NAV-VELNED, NAV_PVT and NAV-TIMEUTC are configured.
 *					The communication is with GPS satellites.
 * @retval GNSS status
 */
UBX_StatusTypeDef GNSS_configure(void)
{

	UART_HandleTypeDef *huart = Parser.hgps->huart;
	GNSS_HandleTypeDef *hgps  = Parser.hgps;

	//build the payload
	ubx_payload_tx_cfg_prt_t *ptrPyld = &(Parser.buf_packet.payload_tx_cfg_prt);

	memset(ptrPyld, 0, sizeof(Parser.buf_packet.payload_tx_cfg_prt));
	ptrPyld->portID		    = hgps->Init.portID;
	ptrPyld->mode		      = UBX_TX_CFG_PRT_MODE;
	ptrPyld->baudRate	    = huart->Init.BaudRate;
	ptrPyld->inProtoMask	= UBX_TX_CFG_PRT_INPROTOMASK;
	ptrPyld->outProtoMask	= UBX_TX_CFG_PRT_OUTPROTOMASK;

	GNSS_send_message(UBX_MSG_CFG_PRT, Parser.buf_packet.raw, sizeof(Parser.buf_packet.payload_tx_cfg_prt));
	if (wait_for_ack(UBX_MSG_CFG_PRT) != UBX_ACK_OK)
	{
		/* try next baudrate */
		log_message("not good baud try next\r\n");
		return UBX_CFG_ERROR_BAUD;
	}


	/* Send a CFG-RATE message to define update rate */
	memset(&(Parser.buf_packet.payload_tx_cfg_rate), 0, sizeof(Parser.buf_packet.payload_tx_cfg_rate));
	Parser.buf_packet.payload_tx_cfg_rate.measRate	= hgps->Init.MeasurementRate;
	Parser.buf_packet.payload_tx_cfg_rate.navRate 	= hgps->Init.navigationRate;
	Parser.buf_packet.payload_tx_cfg_rate.timeRef	  = hgps->Init.timeRef;

	GNSS_send_message(UBX_MSG_CFG_RATE, Parser.buf_packet.raw, sizeof(Parser.buf_packet.payload_tx_cfg_rate));

	if (wait_for_ack(UBX_MSG_CFG_RATE) != UBX_ACK_OK) {
		hgps->errorStatus = UBX_CFG_NO_RES_ACK;
		return UBX_CFG_NO_RES_ACK;
	}


	/* send a NAV5 message to set the options for the internal filter */
	memset(&(Parser.buf_packet.payload_tx_cfg_nav5), 0, sizeof(Parser.buf_packet.payload_tx_cfg_nav5));
	Parser.buf_packet.payload_tx_cfg_nav5.mask		= UBX_TX_CFG_NAV5_MASK;
	Parser.buf_packet.payload_tx_cfg_nav5.dynModel= hgps->Init.dynModel;
	Parser.buf_packet.payload_tx_cfg_nav5.fixMode	= hgps->Init.fixMode;

	GNSS_send_message(UBX_MSG_CFG_NAV5, Parser.buf_packet.raw, sizeof(Parser.buf_packet.payload_tx_cfg_nav5));

	if (wait_for_ack(UBX_MSG_CFG_NAV5) != UBX_ACK_OK) {
		hgps->errorStatus = UBX_CFG_NO_RES_ACK;
		return UBX_CFG_NO_RES_ACK;
	}

#ifdef UBX_CONFIGURE_SBAS
	/* send a SBAS message to set the SBAS options */
	memset(&(Parser.buf_packet.payload_tx_cfg_sbas, 0, sizeof(Parser.buf_packet.payload_tx_cfg_sbas));
	_buf.payload_tx_cfg_sbas.mode		= UBX_TX_CFG_SBAS_MODE;

	GNSS_send_message(UBX_MSG_CFG_SBAS, Parser.buf_packet.raw, sizeof(Parser.buf_packet.payload_tx_cfg_sbas)))



	if (wait_for_ack(UBX_MSG_CFG_SBAS) != UBX_ACK_OK) {
		hgps->errorStatus = UBX_CFG_NO_RES_ACK;
		return UBX_CFG_NO_RES_ACK;
	}

#endif



	/* configure message rates */
	/* the last argument is divisor for measurement rate (set by CFG RATE), i.e. 1 means 5Hz */

	/* try to set rate for NAV-PVT */
	/* (implemented for ubx7+ modules only, use NAV-SOL, NAV-POSLLH, NAV-VELNED and NAV-TIMEUTC for ubx6) */
	configure_message_rate(UBX_MSG_NAV_PVT, 1);
	if (wait_for_ack(UBX_MSG_CFG_MSG) != UBX_ACK_OK) {
		log_message("NAV-PVT not handled");
		hgps->use_nav_pvt = false;
	} else {
		hgps->use_nav_pvt = true;
	}

	if (!(hgps->use_nav_pvt)) {

		configure_message_rate(UBX_MSG_NAV_TIMEUTC, 5);// the bigger the rate the smaller the frequency ( 1hz here) for 5hz GPS
		if(wait_for_ack(UBX_MSG_CFG_MSG) != UBX_ACK_OK)
		{
			hgps->errorStatus = UBX_CFG_NO_RES_ACK;
			return UBX_CFG_NO_RES_ACK;
		}

		configure_message_rate(UBX_MSG_NAV_POSLLH, 1);
		if (wait_for_ack(UBX_MSG_CFG_MSG) != UBX_ACK_OK) {
			hgps->errorStatus = UBX_CFG_NO_RES_ACK;
			return UBX_CFG_NO_RES_ACK;
		}

		configure_message_rate(UBX_MSG_NAV_SOL, 1);
		if (wait_for_ack(UBX_MSG_CFG_MSG) != UBX_ACK_OK) {
			hgps->errorStatus = UBX_CFG_NO_RES_ACK;
			return UBX_CFG_NO_RES_ACK;
		}


		configure_message_rate(UBX_MSG_NAV_VELNED, 1);
		if (wait_for_ack(UBX_MSG_CFG_MSG) != UBX_ACK_OK) {
			hgps->errorStatus = UBX_CFG_NO_RES_ACK;
			return UBX_CFG_NO_RES_ACK;
		}
	}

	configure_message_rate(UBX_MSG_NAV_SVINFO, (hgps->satellite_info != NULL) ? 5 : 0);
	if (wait_for_ack(UBX_MSG_CFG_MSG) != UBX_ACK_OK ) {
		log_message("not handling NAV-SVINFO");
		hgps->errorStatus = UBX_CFG_NO_RES_ACK;
		return UBX_CFG_NO_RES_ACK;
	}

//	configure_message_rate(UBX_MSG_RXM_RAWX, (hgps->gnss_meas != NULL) ? 5: 0);
//	if(wait_for_ack(UBX_MSG_CFG_MSG) != UBX_ACK_OK){
//		log_message("not handling the RXM-RAWX");
//		hgps->use_gnss_meas = false;
//	}
//
//	configure_message_rate(UBX_MSG_RXM_SFRBX, (hgps->subFrames != NULL) ? 5:0);
//	if(wait_for_ack(UBX_MSG_CFG_MSG) != UBX_ACK_OK){
//		log_message("not handling the RXM-SFRBX");
//		hgps->use_subFrame = false;
//	}

//	if(HAL_UART_DMAResume(huart) != HAL_OK)
//	{
//		log_message("cant resume");
//	}

	Parser.configured= true;
	hgps->errorStatus = UBX_OK;
	return UBX_OK;
}//end fonction configure


/**
 * @brief  send a message to the ubloc module. The DMA reception is paused
 *					during the sending
 * @param  msg    : the message to send ( see defines )
 * @param  payload: the payload that comes with the messages
 * @param  length : the size of the payload
 * @retval None
 */
static void GNSS_send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
	UART_HandleTypeDef *huart = Parser.hgps->huart;
	ubx_header_t   header;// = {UBX_SYNC1, UBX_SYNC2};
	ubx_checksum_t checksum = {0, 0};

	// Populate header
	header.headerField.sync 	= UBX_SYNC;
	header.headerField.msg		= msg;
	header.headerField.length	= length;

	// Calculate checksum
	calc_checksum(((uint8_t*)&header) + 2, sizeof(header) - 2, &checksum);  // skip 2 sync bytes
	if (payload != NULL)
		calc_checksum(payload, length, &checksum);
	// Pause the DMA reception 
	//	if(HAL_UART_DMAPause(huart) != HAL_OK)
	//	{
	//		log_message("cant pause");
	//	}
	//send the message

	HAL_StatusTypeDef response = HAL_BUSY;
	int test = 0;
	if(response != HAL_OK)
	{
		response = HAL_UART_Transmit(huart, (uint8_t *)header.headerBytes, UBX_HEADER_SIZE, 1000);
		//log_message(response);
		log_message("transmit fail 1");
	}

	if (payload != NULL)
	{
		if(HAL_UART_Transmit(huart, (uint8_t *)payload, length,1000) != HAL_OK)
		{
			log_message("tranmit fail 2");
		}
	}
	//send the checksum
	uint8_t cs_buf[2] = {checksum.ck_a, checksum.ck_b};

	if(HAL_UART_Transmit(huart, (uint8_t *)cs_buf, sizeof(checksum),1000) != HAL_OK)
	{
		log_message("tansmit fail 3");
	}

	//	if(HAL_UART_DMAResume(huart) != HAL_OK)
	//	{
	//		log_message("cant resume");
	//	}


}


/**
 * @brief  log the data received by the parser in a uSD card.
 * 				the data will be convert into a string a write in a text file
 * @param  hgps: pointer to a GNSS_HandleTypeDef structure that contains
 *                the configuration information for the specified GNSS module.
 */
void GNSS_log(GNSS_HandleTypeDef *hgps)
{
	if(hgps->gps_position->fix_type > 0)
	{
		log_message("has FIX!");
	}

//	log_nav("test ","test");
	if(hgps->got_posllh)
	{
		char bufPOS[50];
		sprintf(bufPOS, "%d, %d, %d", hgps->gps_position->lat,
				hgps->gps_position->lon, hgps->gps_position->alt);
		log_nav(bufPOS, "NAV-POSLLH:");
		hgps->got_posllh = false;
		//Turn_LED_On(3);
	}
	if(hgps->got_sol)
	{
		char bufSOL[50];
		sprintf(bufSOL, "%d, %d", hgps->gps_position->fix_type,
				 hgps->gps_position->satellites_used);
		log_nav(bufSOL, "NAV-SOL:");
		hgps->got_sol = false;
		//Turn_LED_On(3);
	}

//	if(hgps->got_velned)
//	{
//		char bufSOL[50];
//		sprintf(bufSOL, "%f, %f, %f, %f, %f, %f",
//				hgps->gps_position->vel_m_s,   hgps->gps_position->vel_n_m_s, hgps->gps_position->vel_e_m_s,
//				hgps->gps_position->vel_d_m_s, hgps->gps_position->cog_rad,   hgps->gps_position->c_variance_rad);
//		log_nav(bufSOL, "NAV-VELNED:");
//		hgps->got_velned	= false;
//		Turn_LED_On(3);
//	}


}



/**  
===============================================================================
            #####         private functions          #####
 ===============================================================================  
 **/
/**
 * @brief  reset the parser when a complete frame is received or when a
 *					an error in the packet ( like in the checksum ) is detected
 *					in the UBX parser state machine
 * @retval None
 */
static void UBX_resetParser(void)
{
	Parser.decode_state = UBX_DECODE_SYNC1;
	Parser.rx_ck_a = 0;
	Parser.rx_ck_b = 0;
	Parser.rx_payload_length = 0;
	Parser.rx_payload_index = 0;
}

/**
 * @brief  Initialise the parser for the UBX state machine
 * @retval None
 */
static void UBX_Parser_Init(void)
{
	Parser.configured     = false;
	Parser.rtc_configured = false;
	Parser.ack_state  = UBX_ACK_IDLE;
	Parser.ack_waiting_msg =0;
	UBX_resetParser();
}

//TODO: change the return val for UBX_StatusTypeDef
/**
 * @brief  UBX state machine. the function decode each byte received from the
 *  				the GNSS. INTERRUPTION CONTEXT
 * @param  b: the byte received from the GNSS
 * @retval  0 = decoding, 1 = message handled, 2 = sat info message handled
 */
int  parse_char(const uint8_t b)
{
	int ret = 0;
	//ubx_decode_state_t decode_state = hgps->_decode_state;
	switch (Parser.decode_state) 
	{

	/* Expecting Sync1 */
	case UBX_DECODE_SYNC1:
		if (b == UBX_SYNC1) {	// Sync1 found --> expecting Sync2
			//printf("A\r\n");
			Parser.decode_state = UBX_DECODE_SYNC2;
		}
		break;

		/* Expecting Sync2 */
	case UBX_DECODE_SYNC2:
		if (b == UBX_SYNC2) {	// Sync2 found --> expecting Class
			//printf("B\r\n");
			Parser.decode_state = UBX_DECODE_CLASS;

		} else {		// Sync1 not followed by Sync2: reset parser
			UBX_resetParser();
		}
		break;

		/* Expecting Class */
	case UBX_DECODE_CLASS:
		//printf("C: %d\r\n", b);
		add_byte_to_checksum(b);   // checksum is calculated for everything except Sync and Checksum bytes
		Parser.rx_msg = b;
		Parser.decode_state = UBX_DECODE_ID;
		break;

		/* Expecting ID */
	case UBX_DECODE_ID:
		//printf("ID: %d\r\n ", b);
		add_byte_to_checksum(b); 
		Parser.rx_msg |= b << 8;
		Parser.decode_state = UBX_DECODE_LENGTH1;
		break;

		/* Expecting first length byte */
	case UBX_DECODE_LENGTH1:
		//printf("L:");
		add_byte_to_checksum(b); 
		Parser.rx_payload_length = b;
		Parser.decode_state = UBX_DECODE_LENGTH2;
		break;

		/* Expecting second length byte */
	case UBX_DECODE_LENGTH2:

		add_byte_to_checksum(b); 
		Parser.rx_payload_length |= b << 8;	// calculate payload size
		//printf("L:%d\r\n\n", hgps->_rx_payload_length);
		if (payload_rx_init() != 0) {	// start payload reception change to GNSS_status eventually!!
			// payload will not be handled, discard message
			UBX_resetParser();
		} else {
			Parser.decode_state = (Parser.rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
		}
		break;

		/* Expecting payload */
	case UBX_DECODE_PAYLOAD:
		//printf("(%d, %#04x)", ++countG, b);
		add_byte_to_checksum(b); 
		//printf("D:%d, cs:%d ", b, hgps->_rx_ck_a);
		switch (Parser.rx_msg) {
		case UBX_MSG_NAV_SVINFO:
			ret = payload_rx_add_nav_svinfo(b);	// add a NAV-SVINFO payload byte
			break;
		case UBX_MSG_MON_VER:
			ret = payload_rx_add_mon_ver(b);	// add a MON-VER payload byte
			break;
		case UBX_MSG_RXM_RAWX:
			ret = payload_rx_add_rxm_rawx(b);
			break;
		case UBX_MSG_RXM_SFRBX:
			//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
			ret = payload_rx_add_rxm_sfrbx(b);
			break;
		default:
			ret = payload_rx_add(b);		// add a payload byte
			break;
		}
		if (ret < 0) {
			// payload not handled, discard message
			UBX_resetParser();
		} else if (ret > 0) {
			// payload complete, expecting checksum
			Parser.decode_state = UBX_DECODE_CHKSUM1;
		} else {
			// expecting more payload, stay in state UBX_DECODE_PAYLOAD
		}
		ret = 0;
		break;

		/* Expecting first checksum byte */
		case UBX_DECODE_CHKSUM1:
			//		countG =0;
			if (Parser.rx_ck_a != b) {
				HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_9);
				//log_message("ubx checksum1 err ");
				UBX_resetParser();
			} else {
				Parser.decode_state = UBX_DECODE_CHKSUM2;
			}
			break;

			/* Expecting second checksum byte */
		case UBX_DECODE_CHKSUM2:
			if (Parser.rx_ck_b != b) {
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			} else {
				ret = payload_rx_done();	// finish payload processing
			}
			//printf("got 1 frame\r\n");
			UBX_resetParser();
			break;

		default:
			break;
	}
	//printf("parse_ret: %d\r\n", ret);
	return ret;

}


//TODO: change the return val for UBX_StatusTypeDef
/**
 * @brief  once the ubx message andpayload lenght is receive we check if no errors occurs
 *					and if we do handle the message type. INTERRUPTION CONTEXT
 * @retval -1 = abort, 0 = continue
 */
static int  payload_rx_init(void)
{
	//change to GNSS_StatusTypeDef eventually
	int ret = 0;
	GNSS_HandleTypeDef *hgps = Parser.hgps;
	Parser.rx_state = UBX_RXMSG_HANDLE;	// handle by default

	switch (Parser.rx_msg) {
	case UBX_MSG_NAV_PVT:
		if (   (Parser.rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX7)		/* u-blox 7 msg format */
				&& (Parser.rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8))	/* u-blox 8+ msg format */
			Parser.rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!(Parser.configured))
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (!(hgps->use_nav_pvt))
			Parser.rx_state = UBX_RXMSG_DISABLE;	// disable if not using NAV-PVT
		break;

	case UBX_MSG_NAV_POSLLH:
		if (Parser.rx_payload_length != sizeof(ubx_payload_rx_nav_posllh_t))
			Parser.rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!(Parser.configured))
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (hgps->use_nav_pvt)
			Parser.rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_NAV_SOL:
		if (Parser.rx_payload_length != sizeof(ubx_payload_rx_nav_sol_t))
			Parser.rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!(Parser.configured))
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (hgps->use_nav_pvt)
			Parser.rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_NAV_TIMEUTC:
		if (Parser.rx_payload_length != sizeof(ubx_payload_rx_nav_timeutc_t))
			Parser.rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!(Parser.configured))
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (hgps->use_nav_pvt)
			Parser.rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_NAV_SVINFO:
		//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
		if (hgps->use_sat_info == false || hgps->satellite_info == NULL)
			Parser.rx_state = UBX_RXMSG_DISABLE;	// disable if sat info not requested or not initialise
		else if (!(Parser.configured))
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if not configured
		else
			memset(hgps->satellite_info, 0, sizeof(*(hgps->satellite_info)));	// initialize sat info
		break;

	case UBX_MSG_RXM_RAWX:
		if (hgps->use_gnss_meas == false || hgps->gnss_meas == NULL)
			Parser.rx_state = UBX_RXMSG_DISABLE;	// disable if rxm not requested or not initialise
		else if (!(Parser.configured))
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if not configured
		else
			memset(hgps->gnss_meas, 0, sizeof(*(hgps->gnss_meas)));	// initialize gnss measurements
		break;

	case UBX_MSG_RXM_SFRBX:

		if (hgps->use_subFrame == false || hgps->subFrames == NULL)
			Parser.rx_state = UBX_RXMSG_DISABLE;	// disable if rxm not requested or not initialise
		else if (!(Parser.configured))
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if not configured
		else
			memset(hgps->subFrames, 0, sizeof(*(hgps->subFrames)));	// initialize gnss measurements
		break;


	case UBX_MSG_NAV_VELNED:
		if (Parser.rx_payload_length != sizeof(ubx_payload_rx_nav_velned_t))
			Parser.rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!(Parser.configured))
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (hgps->use_nav_pvt)
			Parser.rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_MON_VER:
		break;		// unconditionally handle this message

	case UBX_MSG_MON_HW:
		if (   (Parser.rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx6_t))	/* u-blox 6 msg format */
				&& (Parser.rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx7_t)))	/* u-blox 7+ msg format */
			Parser.rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!(Parser.configured))
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		break;

	case UBX_MSG_ACK_ACK:
		if (Parser.rx_payload_length != sizeof(ubx_payload_rx_ack_ack_t))
			Parser.rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (Parser.configured)
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if _configured
		break;

	case UBX_MSG_ACK_NAK:
		if (Parser.rx_payload_length != sizeof(ubx_payload_rx_ack_nak_t))
			Parser.rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (Parser.configured)
			Parser.rx_state = UBX_RXMSG_IGNORE;	// ignore if _configured
		break;

	default:
		Parser.rx_state = UBX_RXMSG_DISABLE;	// disable all other messages
		break;
	}

	switch (Parser.rx_state) {
	case UBX_RXMSG_HANDLE:	// handle message
	case UBX_RXMSG_IGNORE:	// ignore message but don't report error
		ret = 0;
		break;

	case UBX_RXMSG_DISABLE:	// disable unexpected messages
	ret = -1;	// return error, abort handling this message
	break;

	case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
		ret = -1;	// return error, abort handling this message
		break;

	default:	// invalid message state
		ret = -1;	// return error, abort handling this message
		break;
	}

	return ret;
}

//TODO: change the return val for UBX_StatusTypeDef
/**
 * @brief  building a satellites payload message. INTERRUPTION CONTEXT
 * @param  b: the byte received from the GNSS and added to the payload
 * @retval  -1 = error, 0 = ok, 1 = payload completed
 */
static int  payload_rx_add_nav_svinfo(const uint8_t b)
{

	int ret = 0;
	GNSS_HandleTypeDef *hgps = Parser.hgps;


	if (Parser.rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
		// Fill Part 1 buffer
		Parser.buf_packet.raw[Parser.rx_payload_index] = b;
	}
	else 
	{
		if (Parser.rx_payload_index == sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
			// Part 1 complete: decode Part 1 buffer
			hgps->satellite_info->count = MIN(Parser.buf_packet.payload_rx_nav_svinfo_part1.numCh, SAT_INFO_MAX_SATELLITES);
			//UBX_TRACE_SVINFO("SVINFO len %u  numCh %u\n", (unsigned)_rx_payload_length, (unsigned)_buf.payload_rx_nav_svinfo_part1.numCh);
		}
		if (Parser.rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t) + hgps->satellite_info->count * sizeof(ubx_payload_rx_nav_svinfo_part2_t)) {
			// Still room in satellite_info: fill Part 2 buffer
			unsigned buf_index = (Parser.rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) % sizeof(ubx_payload_rx_nav_svinfo_part2_t);
			Parser.buf_packet.raw[buf_index] = b;
			if (buf_index == sizeof(ubx_payload_rx_nav_svinfo_part2_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned sat_index 													= (Parser.rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) / sizeof(ubx_payload_rx_nav_svinfo_part2_t);
				hgps->satellite_info->used[sat_index]				= (uint8_t)(Parser.buf_packet.payload_rx_nav_svinfo_part2.flags & 0x01);
				hgps->satellite_info->snr[sat_index]				= (uint8_t)(Parser.buf_packet.payload_rx_nav_svinfo_part2.cno);
				hgps->satellite_info->elevation[sat_index]	= (uint8_t)(Parser.buf_packet.payload_rx_nav_svinfo_part2.elev);
				hgps->satellite_info->azimuth[sat_index]		= (uint8_t)((float)(Parser.buf_packet.payload_rx_nav_svinfo_part2.azim) * 255.0f / 360.0f);
				hgps->satellite_info->svid[sat_index]				= (uint8_t)(Parser.buf_packet.payload_rx_nav_svinfo_part2.svid);

			}
		}
	}

	if (++(Parser.rx_payload_index) >= Parser.rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}



static int  payload_rx_add_rxm_rawx(const uint8_t b)
{
	int ret = 0;
	GNSS_HandleTypeDef *hgps = Parser.hgps;


	if (Parser.rx_payload_index < sizeof(ubx_payload_rx_rxm_rawx_part1_t)) {
		// Fill Part 1 buffer
		Parser.buf_packet.raw[Parser.rx_payload_index] = b;
	}
	else 
	{
		if (Parser.rx_payload_index == sizeof(ubx_payload_rx_rxm_rawx_part1_t)) {
			// Part 1 complete: decode Part 1 buffer
			hgps->gnss_meas->numMeas = MIN(Parser.buf_packet.payload_rx_rxm_rawx_part1.numMeas, RAW_MAX_MEASURE);
		}
		if (Parser.rx_payload_index < sizeof(ubx_payload_rx_rxm_rawx_part1_t) + hgps->gnss_meas->numMeas * sizeof(ubx_payload_rx_rxm_rawx_part2_t)) {
			// Still room in satellite_info: fill Part 2 buffer
			unsigned buf_index = (Parser.rx_payload_index - sizeof(ubx_payload_rx_rxm_rawx_part1_t)) % sizeof(ubx_payload_rx_rxm_rawx_part2_t);
			Parser.buf_packet.raw[buf_index] = b;
			if (buf_index == sizeof(ubx_payload_rx_rxm_rawx_part2_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned meas_index 									= (Parser.rx_payload_index - sizeof(ubx_payload_rx_rxm_rawx_part1_t)) / sizeof(ubx_payload_rx_rxm_rawx_part2_t);
				hgps->gnss_meas->prMes[meas_index]		= (double)(Parser.buf_packet.payload_rx_rxm_rawx_part2.prMes);
				hgps->gnss_meas->cpMes[meas_index]		= (double)(Parser.buf_packet.payload_rx_rxm_rawx_part2.cpMes);
				hgps->gnss_meas->doMes[meas_index]		= (float)(Parser.buf_packet.payload_rx_rxm_rawx_part2.doMes);
				hgps->gnss_meas->gnssId[meas_index]		= (uint8_t)(Parser.buf_packet.payload_rx_rxm_rawx_part2.gnssId);
				hgps->gnss_meas->svId[meas_index]		  = (uint8_t)(Parser.buf_packet.payload_rx_rxm_rawx_part2.svId);
				hgps->gnss_meas->locktime[meas_index] = (uint16_t)(Parser.buf_packet.payload_rx_rxm_rawx_part2.locktime);
				hgps->gnss_meas->cno[meas_index]			=	(uint8_t)(Parser.buf_packet.payload_rx_rxm_rawx_part2.cno);

			}
		}
	}

	if (++(Parser.rx_payload_index) >= Parser.rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

static int  payload_rx_add_rxm_sfrbx(const uint8_t b)
{

	int ret = 0;
	GNSS_HandleTypeDef *hgps = Parser.hgps;

	if (Parser.rx_payload_index < sizeof(ubx_payload_rx_rxm_sfrbx_part1_t)) {
		// Fill Part 1 buffer
		Parser.buf_packet.raw[Parser.rx_payload_index] = b;
	}
	else 
	{
		if (Parser.rx_payload_index == sizeof(ubx_payload_rx_rxm_sfrbx_part1_t)) {
			// Part 1 complete: decode Part 1 buffer
			hgps->subFrames->numWord= MIN(Parser.buf_packet.payload_rx_rxm_sfrbx_part1.numWords, SUBFRAME_MAX_WORD);
			hgps->subFrames->freqId = Parser.buf_packet.payload_rx_rxm_sfrbx_part1.freqId;
			hgps->subFrames->gnssId = Parser.buf_packet.payload_rx_rxm_sfrbx_part1.gnssId;
			hgps->subFrames->svId   = Parser.buf_packet.payload_rx_rxm_sfrbx_part1.svId;
		}
		if (Parser.rx_payload_index < sizeof(ubx_payload_rx_rxm_sfrbx_part1_t) + hgps->subFrames->numWord * sizeof(ubx_payload_rx_rxm_sfrbx_part2_t)) {
			// Still room in satellite_info: fill Part 2 buffer
			unsigned buf_index = (Parser.rx_payload_index - sizeof(ubx_payload_rx_rxm_sfrbx_part1_t)) % sizeof(ubx_payload_rx_rxm_sfrbx_part2_t);
			Parser.buf_packet.raw[buf_index] = b;
			if (buf_index == sizeof(ubx_payload_rx_rxm_sfrbx_part2_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned meas_index 									= (Parser.rx_payload_index - sizeof(ubx_payload_rx_rxm_sfrbx_part1_t)) / sizeof(ubx_payload_rx_rxm_sfrbx_part2_t);
				hgps->subFrames->dwrd[meas_index] = (uint32_t)(Parser.buf_packet.payload_rx_rxm_sfrbx_part2.dwrd);
			}
		}
	}

	if (++(Parser.rx_payload_index) >= Parser.rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

//TODO: change the return val for UBX_StatusTypeDef
/**
 * @brief  building a payload message. INTERRUPTION CONTEXT
 * @param  b: the byte received from the GNSS and added to the payload
 * @retval  -1 = error, 0 = ok, 1 = payload completed
 */
static int  payload_rx_add(const uint8_t b)
{
	int ret = 0;

	Parser.buf_packet.raw[Parser.rx_payload_index] = b;
	Parser.rx_payload_index++;
	//printf("%d >= %d\r\n", hgps->_rx_payload_index,  hgps->_rx_payload_length);
	if ((Parser.rx_payload_index) >= Parser.rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

//TODO: change the return val for UBX_StatusTypeDef
//INTERRUPTION CONTEXT
static int payload_rx_add_mon_ver(const uint8_t b)
{

	int ret = 0;
	GNSS_HandleTypeDef *hgps = Parser.hgps;


	if (Parser.rx_payload_index < sizeof(ubx_payload_rx_mon_ver_part1_t)) {
		// Fill Part 1 buffer
		Parser.buf_packet.raw[Parser.rx_payload_index] = b;
	} else {
		if (Parser.rx_payload_index == sizeof(ubx_payload_rx_mon_ver_part1_t)) {
			// Part 1 complete: decode Part 1 buffer and calculate hash for SW&HW version strings
			hgps->ubx_version = fnv1_32_str(Parser.buf_packet.payload_rx_mon_ver_part1.swVersion, FNV1_32_INIT);
			hgps->ubx_version = fnv1_32_str(Parser.buf_packet.payload_rx_mon_ver_part1.hwVersion, hgps->ubx_version);

		}
		// fill Part 2 buffer
		unsigned buf_index = (Parser.rx_payload_index - sizeof(ubx_payload_rx_mon_ver_part1_t)) % sizeof(ubx_payload_rx_mon_ver_part2_t);
		Parser.buf_packet.raw[buf_index] = b;
		if (buf_index == sizeof(ubx_payload_rx_mon_ver_part2_t) - 1) {
			// Part 2 complete: decode Part 2 buffer
			//			UBX_WARN("VER ext \" %30s\"", Parser.buf_packet.payload_rx_mon_ver_part2.extension);
		}
	}

	if (++(Parser.rx_payload_index) >= Parser.rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

//TODO: change the return val for UBX_StatusTypeDef
/**
 * @brief  once a packet payload is received, we initialise the data structure
 * 				with the payload information. the RTC is configure only one here if
 *					GPS time is received. INTERRUPTION CONTEXT
 * @retval   0 = no message handled, 1 = message handled, 2 = sat info message handled or raw meas
 */
static int payload_rx_done(void)
{
	int ret = 0;
	GNSS_HandleTypeDef *hgps = Parser.hgps;

	// return if no message handled
	if (Parser.rx_state != UBX_RXMSG_HANDLE) {
		return ret;
	}

	// handle message
	switch (Parser.rx_msg) {

	case UBX_MSG_NAV_PVT:
		//printf("Rx NAV-PVT\n");

		hgps->gps_position->fix_type		= Parser.buf_packet.payload_rx_nav_pvt.fixType;
		hgps->gps_position->satellites_used	= Parser.buf_packet.payload_rx_nav_pvt.numSV;

		hgps->gps_position->lat		= Parser.buf_packet.payload_rx_nav_pvt.lat;
		hgps->gps_position->lon		= Parser.buf_packet.payload_rx_nav_pvt.lon;
		hgps->gps_position->alt		= Parser.buf_packet.payload_rx_nav_pvt.hMSL;

		hgps->gps_position->eph		= (float)Parser.buf_packet.payload_rx_nav_pvt.hAcc * 1e-3f;
		hgps->gps_position->epv		= (float)Parser.buf_packet.payload_rx_nav_pvt.vAcc * 1e-3f;
		hgps->gps_position->dop     = (float)Parser.buf_packet.payload_rx_nav_pvt.pDOP *1e-3f;
		hgps->gps_position->s_variance_m_s	= (float)Parser.buf_packet.payload_rx_nav_pvt.sAcc * 1e-3f;

		hgps->gps_position->vel_m_s		= (float)Parser.buf_packet.payload_rx_nav_pvt.gSpeed * 1e-3f;

		hgps->gps_position->vel_n_m_s	= (float)Parser.buf_packet.payload_rx_nav_pvt.velN * 1e-3f;
		hgps->gps_position->vel_e_m_s	= (float)Parser.buf_packet.payload_rx_nav_pvt.velE * 1e-3f;
		hgps->gps_position->vel_d_m_s	= (float)Parser.buf_packet.payload_rx_nav_pvt.velD * 1e-3f;
		hgps->gps_position->vel_ned_valid	= true;

		hgps->gps_position->cog_rad		= (float)Parser.buf_packet.payload_rx_nav_pvt.headMot * M_DEG_TO_RAD_F * 1e-5f;
		hgps->gps_position->c_variance_rad	= (float)Parser.buf_packet.payload_rx_nav_pvt.headAcc * M_DEG_TO_RAD_F * 1e-5f;

		{
			//			if(Parser.rtc_configured ==false && Parser.hgps->gps_position->fix_type > NO_FIX)
			//			{
			//				RTC_TimeTypeDef sTime;
			//				RTC_DateTypeDef sDate;
			//				sTime.Hours = Parser.buf_packet.payload_rx_nav_pvt.hour;
			//				sTime.Minutes = Parser.buf_packet.payload_rx_nav_pvt.min;
			//				sTime.Seconds = Parser.buf_packet.payload_rx_nav_pvt.sec;
			//				sTime.SubSeconds = 0;
			//				sTime.TimeFormat = RTC_HOURFORMAT12_AM;
			//				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			//				sTime.StoreOperation = RTC_STOREOPERATION_RESET;
			//				HAL_RTC_SetTime(Parser.hgps->hrtc, &sTime, RTC_FORMAT_BCD);
			//
			////				sDate.WeekDay = RTC_WEEKDAY_MONDAY;
			//				sDate.Month = Parser.buf_packet.payload_rx_nav_pvt.month ;
			//				sDate.Date = Parser.buf_packet.payload_rx_nav_pvt.day;
			//				sDate.Year = Parser.buf_packet.payload_rx_nav_pvt.year;
			//				HAL_RTC_SetDate(Parser.hgps->hrtc, &sDate, RTC_FORMAT_BCD);
			//
			//				HAL_RTCEx_BKUPWrite(Parser.hgps->hrtc,RTC_BKP_DR0,0x32F2);
			//				Parser.rtc_configured =true;
			//			}

		}
		// NO! to change!!!!
		hgps->gps_position->timestamp_time		= HAL_GetTick();//hrt_absolute_time();
		hgps->gps_position->timestamp_velocity 	= HAL_GetTick();//hrt_absolute_time();
		hgps->gps_position->timestamp_variance 	= HAL_GetTick();//hrt_absolute_time();
		hgps->gps_position->timestamp_position	= HAL_GetTick();//hrt_absolute_time();

		//		hgps->_rate_count_vel++;
		//		hgps->_rate_count_lat_lon++;
		hgps->got_posllh = true;
		hgps->got_velned = true;
		hgps->got_sol	 = true;

		ret = 1;
		break;

	case UBX_MSG_NAV_POSLLH:
		//printf("Rx NAV-POSLLH\n");

		hgps->gps_position->lat	= Parser.buf_packet.payload_rx_nav_posllh.lat;
		hgps->gps_position->lon	= Parser.buf_packet.payload_rx_nav_posllh.lon;
		hgps->gps_position->alt	= Parser.buf_packet.payload_rx_nav_posllh.hMSL;
		hgps->gps_position->eph	= (float)(Parser.buf_packet.payload_rx_nav_posllh.hAcc * 1e-3f); // from mm to m
		hgps->gps_position->epv	= (float)(Parser.buf_packet.payload_rx_nav_posllh.vAcc * 1e-3f); // from mm to m

		hgps->gps_position->timestamp_position = HAL_GetTick();//hrt_absolute_time();
		//		hgps->rec_frame = NAV_POSLLH;
		hgps->got_posllh = true;
		//		_rate_count_lat_lon++;
		//		char bufPOS[50];
		//		sprintf(bufPOS, "%d, %d, %d, %f, %f", hgps->gps_position->lat,
		//					hgps->gps_position->lon, hgps->gps_position->alt, hgps->gps_position->eph, hgps->gps_position->epv);

		//		log_nav(bufPOS, "NAV-POSLLH:");

		ret = 1;
		break;

	case UBX_MSG_NAV_SOL:
		//printf("Rx NAV-SOL\r\n");

		hgps->gps_position->fix_type		= Parser.buf_packet.payload_rx_nav_sol.gpsFix;
		hgps->gps_position->s_variance_m_s	= (float)(Parser.buf_packet.payload_rx_nav_sol.sAcc * 1e-2f);	// from cm to m
		hgps->gps_position->satellites_used	= Parser.buf_packet.payload_rx_nav_sol.numSV;

		hgps->gps_position->timestamp_variance = HAL_GetTick();//hrt_absolute_time();
		//		hgps->rec_frame = NAV_SOL;
		hgps->got_sol			= true;
		//		char bufSOL[50];
		//		sprintf(bufSOL, "%d, %f, %d", hgps->gps_position->fix_type,
		//						hgps->gps_position->s_variance_m_s, hgps->gps_position->satellites_used);
		//		log_nav(bufSOL, "NAV-SOL:");
		ret = 1;
		break;

	case UBX_MSG_NAV_TIMEUTC:
		//printf("Rx NAV-TIMEUTC\n");

	{
		//			if(Parser.rtc_configured ==false && Parser.hgps->gps_position->fix_type > NO_FIX)
		//			{
		//				RTC_TimeTypeDef sTime;
		//				RTC_DateTypeDef sDate;
		//				sTime.Hours = Parser.buf_packet.payload_rx_nav_pvt.hour;
		//				sTime.Minutes = Parser.buf_packet.payload_rx_nav_pvt.min;
		//				sTime.Seconds = Parser.buf_packet.payload_rx_nav_pvt.sec;
		//				sTime.SubSeconds = 0;
		//				sTime.TimeFormat = RTC_HOURFORMAT12_AM;
		//				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		//				sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		//				HAL_RTC_SetTime(Parser.hgps->hrtc, &sTime, RTC_FORMAT_BIN);
		//
		////				sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		//				sDate.Month = Parser.buf_packet.payload_rx_nav_pvt.month ;
		//				sDate.Date = Parser.buf_packet.payload_rx_nav_pvt.day;
		//				sDate.Year = Parser.buf_packet.payload_rx_nav_pvt.year;
		//				HAL_RTC_SetDate(Parser.hgps->hrtc, &sDate, RTC_FORMAT_BIN);
		//
		//				HAL_RTCEx_BKUPWrite(Parser.hgps->hrtc,RTC_BKP_DR0,0x32F2);
		//				Parser.rtc_configured =true;
		////				log_message("utc time configured, has fix");
		//			}
		//			log_time();

	}
	//		hgps->rec_frame = NAV_TIMEUTC;
	hgps->gps_position->timestamp_time = HAL_GetTick();//hrt_absolute_time();

	ret = 1;
	break;

	case UBX_MSG_NAV_SVINFO:
		//printf("Rx NAV-SVINFO\n");

		// _satellite_info already populated by payload_rx_add_svinfo(), just add a timestamp
		hgps->satellite_info->timestamp = HAL_GetTick();//hrt_absolute_time();
		hgps->got_svinfo = true;
		//		hgps->rec_frame = NAV_SVINFO;
		ret = 2;
		break;


	case UBX_MSG_RXM_RAWX:
		hgps->got_gnssMeas = true;
		ret = 2;
		break;

	case UBX_MSG_RXM_SFRBX:
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
		hgps->got_subFrame = true;
		ret = 2;
		break;

	case UBX_MSG_NAV_VELNED:
		//printf("Rx NAV-VELNED\n");

		hgps->gps_position->vel_m_s		= (float)(Parser.buf_packet.payload_rx_nav_velned.speed * 1e-2f);
		hgps->gps_position->vel_n_m_s	= (float)(Parser.buf_packet.payload_rx_nav_velned.velN * 1e-2f); /* NED NORTH velocity */
		hgps->gps_position->vel_e_m_s	= (float)(Parser.buf_packet.payload_rx_nav_velned.velE * 1e-2f); /* NED EAST velocity */
		hgps->gps_position->vel_d_m_s	= (float)(Parser.buf_packet.payload_rx_nav_velned.velD * 1e-2f); /* NED DOWN velocity */
		hgps->gps_position->cog_rad		= (float)(Parser.buf_packet.payload_rx_nav_velned.heading * M_DEG_TO_RAD_F * 1e-5f);
		hgps->gps_position->c_variance_rad	= (float)(Parser.buf_packet.payload_rx_nav_velned.cAcc * M_DEG_TO_RAD_F * 1e-5f);
		hgps->gps_position->vel_ned_valid	= true;

		//		hgps->gps_position->timestamp_velocity = HAL_GetTick();//hrt_absolute_time();

		//		hgps->_rate_count_vel++;
		//				printf("NAL_VELNED : %f, %f, %f\r\n", hgps->_gps_position->vel_m_s,
		//						hgps->_gps_position->vel_n_m_s, hgps->_gps_position->vel_e_m_s);
		//printf("V\r\n");
		hgps->got_velned = true;
		//		hgps->rec_frame = NAV_VELNED;
		//		char bufVEL[50];
		//		sprintf(bufVEL, "%f, %f, %f, %f, %f, %f",
		//						hgps->gps_position->vel_m_s,   hgps->gps_position->vel_n_m_s, hgps->gps_position->vel_e_m_s,
		//						hgps->gps_position->vel_d_m_s, hgps->gps_position->cog_rad,   hgps->gps_position->c_variance_rad);
		//		log_nav(bufVEL, "NAV-VELNED:");

		ret = 1;
		break;

	case UBX_MSG_MON_VER:
		//printf("Rx MON-VER\n");

		ret = 1;
		break;

	case UBX_MSG_MON_HW:
		//printf("Rx MON-HW\n");

		switch (Parser.rx_payload_length) {

		case sizeof(ubx_payload_rx_mon_hw_ubx6_t):	/* u-blox 6 msg format */
							hgps->gps_position->noise_per_ms		= Parser.buf_packet.payload_rx_mon_hw_ubx6.noisePerMS;
		hgps->gps_position->jamming_indicator	= Parser.buf_packet.payload_rx_mon_hw_ubx6.jamInd;

		ret = 1;
		break;

		case sizeof(ubx_payload_rx_mon_hw_ubx7_t):	/* u-blox 7+ msg format */
							hgps->gps_position->noise_per_ms		= Parser.buf_packet.payload_rx_mon_hw_ubx7.noisePerMS;
		hgps->gps_position->jamming_indicator	= Parser.buf_packet.payload_rx_mon_hw_ubx7.jamInd;

		ret = 1;
		break;

		default:		// unexpected payload size:
			ret = 0;	// don't handle message
			break;
		}
		break;

		case UBX_MSG_ACK_ACK:
			//printf("Rx ACK-ACK\n");

			if ((Parser.ack_state == UBX_ACK_WAITING) && (Parser.buf_packet.payload_rx_ack_ack.msgByte == Parser.ack_waiting_msg)) {
				Parser.ack_state = UBX_ACK_GOT_ACK;
			}

			ret = 1;
			break;

		case UBX_MSG_ACK_NAK:
			//printf("Rx ACK-NAK\n");

			if ((Parser.ack_state == UBX_ACK_WAITING) && (Parser.buf_packet.payload_rx_ack_ack.msgByte == Parser.ack_waiting_msg)) {
				Parser.ack_state = UBX_ACK_GOT_NAK;
			}
			ret = 1;
			break;

		default:
			//printf("no msg to parse rx_done");
			break;
	}
	//printf("done_ret: %d\r\n", ret);
	return ret;
}

/**
 * @brief  start or stop streaming a message from the ubloc module.
 * @param  msg    : the message to start or stop stream
 * @param  rate   : message frequency rate. is divisor for measurement
 *                  rate (set by CFG RATE), i.e. 1 means 5Hz ( 0 will stop the streaming)
 * @retval None
 */
static void configure_message_rate( const uint16_t msg, const uint8_t rate)
{
	ubx_payload_tx_cfg_msg_t cfg_msg;	// don't use _buf (allow interleaved operation)

	cfg_msg.msg.msgByte	= msg;
	cfg_msg.rate	= rate;

	//	send_message(hgps, UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
	GNSS_send_message(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, TX_CFG_MSG_LEN);
}


/**
 * @brief  calculate the checksum.
 * @param  buffer    : payload buffer
 * @param  length    : the payload size
 * @param  checksum : struct that will containe the checksum result
 * @retval None
 */
static void calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum)
{
	uint16_t i;
	for (i = 0; i < length; i++) {

		checksum->ck_a = checksum->ck_a + buffer[i];
		checksum->ck_b = checksum->ck_b + checksum->ck_a;
	}
}
/**
 * @brief  use to verify the payload received. we calculate the checksum
 *					for each payload byte and compare withe the received checksum
 * @param  b    : payload byte received
 * @retval None
 */
static void add_byte_to_checksum(const uint8_t b)
{
	Parser.rx_ck_a = Parser.rx_ck_a + b;
	Parser.rx_ck_b = Parser.rx_ck_b + Parser.rx_ck_a;
}


/**
 * @brief  after sending a message, we wait for a response from the ubloc module
 * @param  msg  : message to wait for ( is the same as the message send to configure)
 * @retval None
 */
static UBX_StatusTypeDef wait_for_ack(const uint16_t msg)
{
	int i;
	UART_HandleTypeDef *huart = Parser.hgps->huart;

	Parser.ack_state = UBX_ACK_WAITING;
	Parser.ack_waiting_msg = msg;	// memorize sent msg class&ID for ACK check
	semGpsGate =0;

	uint8_t ack[1];

	int tickstart =HAL_GetTick();
	while((Parser.ack_state == UBX_ACK_WAITING) && ((HAL_GetTick() - tickstart) < WAIT_ACK_TIME))
	{
		HAL_UART_Receive(huart, ack, 1, 200);
		parse_char(ack[0]);

	}
	if(((HAL_GetTick() - tickstart) > WAIT_ACK_TIME) && semGpsGate ==0)
	{
		log_message("error wait ack no gps reception");
	}
	UBX_StatusTypeDef ret =  (Parser.ack_state == UBX_ACK_GOT_ACK)? UBX_ACK_OK : UBX_CFG_NO_RES_ACK;
	Parser.ack_state = UBX_ACK_IDLE;
	return ret;
}

static UBX_StatusTypeDef wait_for_ack2(const uint16_t msg)
{

	Parser.ack_state = UBX_ACK_WAITING;
	Parser.ack_waiting_msg = msg;	// memorize sent msg class&ID for ACK check
	semGpsGate =0;
	int tickstart =HAL_GetTick();
	while((Parser.ack_state == UBX_ACK_WAITING) && ((HAL_GetTick() - tickstart) < WAIT_ACK_TIME))
	{
	}
	if(((HAL_GetTick() - tickstart) > WAIT_ACK_TIME) && semGpsGate ==0)
	{
		log_message("error wait ack no gps reception");
	}
	UBX_StatusTypeDef ret =  (Parser.ack_state == UBX_ACK_GOT_ACK)? UBX_ACK_OK : UBX_CFG_NO_RES_ACK;
	Parser.ack_state = UBX_ACK_IDLE;
	return ret;
}


static uint32_t fnv1_32_str(uint8_t *str, uint32_t hval)
{


	uint8_t *s = str;

	/*
	 * FNV-1 hash each octet in the buffer
	 */
	while (*s) {

		/* multiply by the 32 bit FNV magic prime mod 2^32 */
#if defined(NO_FNV_GCC_OPTIMIZATION)
		hval *= FNV1_32_PRIME;
#else
		hval += (hval<<1) + (hval<<4) + (hval<<7) + (hval<<8) + (hval<<24);
#endif

		/* xor the bottom with the current octet */
		hval ^= (uint32_t)*s++;
	}

	/* return our new hash value */
	return hval;
}

/**
 * @brief  Rx Transfer completed callbacks.
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_8);

	int i;
	for(i =RECEIVE_COUNT/2; i < RECEIVE_COUNT; i++){
		parse_char(Parser.buf_reception[i]);
	}
	semGpsGate++;

}
/**
 * @brief  Tx Half Transfer completed callbacks.
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	//	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_8);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	int i;
	for(i =0; i < RECEIVE_COUNT/2; i++){
		parse_char(Parser.buf_reception[i]);
	}
	semGpsGate++;

}
/* END OF FILE */

