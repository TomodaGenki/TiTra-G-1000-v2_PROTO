/*
 * nuc.c
 *
 *  Created on: 2019/05/10
 *      Author: ayabe
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "nuc.h"
#include "lift.h"
#include "Cntrl_Proc.h"
#include "battery.h"
#include "charge.h"
#include "task_charger_link.h"
#include "lidar.h"
#include "syncturn.h"
#include "sound.h"
#include "turn.h"
#include "stm32f4xx_hal_uart.h"
#include "load.h"
#include "wheel.h"
#include <string.h>

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart3;	// nuc
extern UART_HandleTypeDef huart6;	// lift motor
extern	uint8_t 	ID_buf[];

// ------
extern uint8_t ComBuf[COMSIZE];
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static int16_t RxDataLsp = 0;
static int16_t RxDataRsp = 0;
static int16_t RxDataTsp = 0;
static uint8_t nuc_rx_buf[RxBufSize];
static uint8_t RxDataBuf[RxBufSize];
static uint8_t TxDataBuf[TxBufSize];
static uint8_t RxDataCom = 0;
static uint8_t RxDataLidarArea = 0;
static uint8_t RxDataCtrlOut1 = 0;
static uint8_t RxDataCtrlOut2 = 0;
static uint8_t RxDataCtrlOut3 = 0;
static uint8_t RxDataCtrlOut4 = 0;
static uint8_t com_error_flag = 0;
static uint8_t com_start_flg = 0;
static uint8_t nuc_receive_flg = 0;

/******************************************************************************/
/*           Initiate RX-control buffer	    						      */
/******************************************************************************/
void init_RXBuffer(void){

	RxDataLsp = 0;				// lsp
	RxDataRsp = 0;				// rsp
	RxDataLidarArea = 0;        // LIDAR Area
	RxDataCtrlOut1 = 0;         // control_out1
	RxDataTsp = 0;  			// tsp
	RxDataCtrlOut2 = 0;
	RxDataCtrlOut3 = 0;
	RxDataCtrlOut4 = 0;
}

/******************************************************************************/
/*           get Lidar Area data		    						      	  */
/******************************************************************************/
uint8_t get_Lidar_Area(void){
	return RxDataLidarArea;
}

/******************************************************************************/
/*           get RX control out4 data	    						      	  */
/******************************************************************************/
uint8_t get_RxCntout4(void) {
	return RxDataCtrlOut4;
}

/******************************************************************************/
/*           get RX-command data	    						      	  	  */
/******************************************************************************/
uint8_t get_RxCommand(void){
	return RxDataCom;
}

/******************************************************************************/
/*           get left wheel data	    						      	  	  */
/******************************************************************************/
int16_t get_LeftWheel_Speed(void) {
	return RxDataLsp;
}

/******************************************************************************/
/*           get right wheel data	    						      	  	  */
/******************************************************************************/
int16_t get_RightWheel_Speed(void) {
	return RxDataRsp;
}

/******************************************************************************/
/*           get turn table data	    						      	  	  */
/******************************************************************************/
int16_t get_TurnTable_Speed(void) {
	return RxDataTsp;
}

/******************************************************************************/
/*           get sound control data	    						      	  	  */
/******************************************************************************/
uint8_t get_sound_order(void) {
	return RxDataCtrlOut4 & SOUND_CH_ALL;
}

/******************************************************************************/
/*           check driving speed	    						      	  	  */
/******************************************************************************/
int16_t ck_RxDrvSpeed(void) {
	return (RxDataLsp | RxDataRsp);
}

/******************************************************************************/
/*           request charging	    							      	  	  */
/******************************************************************************/
uint8_t ck_charge_req(void) {
	return (RxDataCtrlOut1 & CHK_CHG_RQST);
}

/******************************************************************************/
/*           request power relay	    						      	  	  */
/******************************************************************************/
uint8_t ck_power_relay_req(void) {
	return (RxDataCtrlOut1 & POWER_RELAY);
}

/******************************************************************************/
/*           request wheel brake	    						      	  	  */
/******************************************************************************/
uint8_t check_wheelbrake(void){
	return RxDataCtrlOut1 & WHEELBRAKE;
}

/******************************************************************************/
/*           request lift up/down	    						      	  	  */
/******************************************************************************/
uint8_t check_lift_order(void) {
	return RxDataCtrlOut1 & LIFT_ALL_FLAG;
}

/******************************************************************************/
/*           request led color and blink pattern				      	  	  */
/******************************************************************************/
uint8_t led_get_indication(void){
	return RxDataCtrlOut3;
}

/******************************************************************************/
/*           communication error flag control					      	  	  */
/******************************************************************************/
void set_com_error_flag(void) {
	com_error_flag |= COM_ERROR_BIT;
}

void reset_com_error_flag(void) {
	com_error_flag &= ~COM_ERROR_BIT;
}

uint8_t get_com_error_flag(void) {
	return com_error_flag;
}

void set_com_start_flag(void) {
	com_start_flg = 1;
}

uint8_t get_com_start_flag(void) {
	return com_start_flg;
}

void ck_com_error(void) {

	if ((get_com_start_flag() != 0) && (check_timer(CNT_GO5) == TIME_UP)) {
		set_com_error_flag();
	}
}

/******************************************************************************/
/*           request power relay	    						      	  	  */
/******************************************************************************/
void uart2_transmitte(char *p) {
	int len = strlen(p);

	for (int i = 0; i < len; i++) {
		while (1) {
			if ((huart3.Instance->SR & 0x80) == 0x80) {
				huart3.Instance->DR = p[i];
				break;
			}
		}
	}
}

/******************************************************************************/
/*           Send Firmware Version					   						  */
/******************************************************************************/
void send_firmware_version(void) {

	TxDataBuf[0] = TX_HEADER;
	TxDataBuf[1] = TxBufSize -3;
	TxDataBuf[2] = AllSesponce;

	TxDataBuf[3] = get_HardwareID_High();
	TxDataBuf[4] = get_HardwareID_Low();
	for (uint8_t i = 0; i < (TxBufSize - 5); i++) {
		TxDataBuf[i+5] = get_fw_version(i);
	}
	HAL_UART_Transmit_DMA(&huart3, TxDataBuf, TxBufSize);
}

/******************************************************************************/
/*           Make Response Data						   						  */
/******************************************************************************/
void HAL_UART_ResponseData(uint8_t rx_command)
{
	// Make Tx Header
	TxDataBuf[0] = TX_HEADER;
	TxDataBuf[1] = TxBufSize -3;
	TxDataBuf[2] = AllSesponce;

	if ((rx_command == NUC_BATT_TEST) || (rx_command == NUC_R_EP) ||
		(rx_command ==NUC_R_BATT_ST) || (rx_command == NUC_R_BATT_ADD))
	{
		for (uint8_t i = 0 ; i < (TxBufSize - 3) ; i++) {
			if (i < COMSIZE) {
				TxDataBuf[i+3] = ComBuf[i];
			} else {
				TxDataBuf[i+3] = 0;
			}
		}
		TxDataBuf[TxBufSize-1] = TX_FOOTER;
		HAL_UART_Transmit_DMA(&huart3,TxDataBuf,TxBufSize);  		// send to NUC

	} else if (RxDataCom == LOOPBACK) {								// Loop back
			for (uint8_t i = 0 ; i < (RxBufSize) ; i++) {
				TxDataBuf[i+3] = RxDataBuf[i];
			}
		TxDataBuf[TxBufSize-1] = TX_FOOTER;
		HAL_UART_Transmit_DMA(&huart3,TxDataBuf,TxBufSize);  		// send to NUC
	} else if (rx_command == NUC_R_ID){
		TxDataBuf[3] = ID_buf[1];
		TxDataBuf[4] = ID_buf[0];
		HAL_UART_Transmit_DMA(&huart3,TxDataBuf,TxBufSize);
	}

}

/******************************************************************************/
/*           Make TxData						   						      */
/******************************************************************************/
void HAL_UART_MakeTxData(void)
{
	static uint32_t r_wheel_enc_old = 0;
	static uint32_t l_wheel_enc_old = 0;
	static uint16_t turn_enc_old = 0;
	uint32_t r_wheel_enc = get_r_wheel_encoder();
	uint32_t l_wheel_enc = get_l_wheel_encoder();
	int16_t  r_wheel_enc_diff = 0;
	int16_t  l_wheel_enc_diff = 0;
	uint16_t turn_enc = get_turn_encoder();
	uint16_t turn_enc_diff;
	uint8_t chg_state = 0;
	uint8_t com_err_bit = get_com_error_flag();

	// Make Tx Header
	TxDataBuf[0] = TX_HEADER;               // STX
	TxDataBuf[1] = TxBufSize - 3;           // LEN
	TxDataBuf[2] = AllSesponce;             // COM

	TxDataBuf[3] = ID_buf[1];				// Hardware ID upper
	TxDataBuf[4] = ID_buf[0];				// Hardware ID lower

	l_wheel_enc_diff = (double)((int32_t)(l_wheel_enc - l_wheel_enc_old))/ 2.0;
	l_wheel_enc_old = l_wheel_enc;
	TxDataBuf[5] = (uint8_t)(l_wheel_enc_diff >> 8);
	TxDataBuf[6] = (uint8_t)l_wheel_enc_diff;

	r_wheel_enc_diff = -(double)((int32_t)(r_wheel_enc - r_wheel_enc_old))/ 2.0;
	r_wheel_enc_old = r_wheel_enc;
	TxDataBuf[7] = (uint8_t)(r_wheel_enc_diff >> 8);
	TxDataBuf[8] = (uint8_t)r_wheel_enc_diff;

	TxDataBuf[9] = get_mcu_state();
	TxDataBuf[9] |= com_err_bit;

	TxDataBuf[10] = get_sensor1_state();
	TxDataBuf[11] = get_sensor2_state();				// TurnTable Angle & Lift State
	TxDataBuf[12] = get_sensor3_state();

	TxDataBuf[13] = (uint8_t)get_battery1_capcity();	// battery1 remained capacity
	TxDataBuf[14] = (uint8_t)get_battery2_capcity();	// battery2 remained capacity

	chg_state = ck_charge_state();
	TxDataBuf[15] = send_mt_alarm();					// set the status of each motor alarm
	TxDataBuf[15] |= chg_state;

	TxDataBuf[16] = 0;									// reserved

	turn_enc_diff = turn_enc - turn_enc_old;				// Difference between the current pulse and the previous pulse
	TxDataBuf[17] = (uint8_t)(turn_enc_diff >> 8);
	TxDataBuf[18] = (uint8_t)turn_enc_diff;
	turn_enc_old = turn_enc;

	TxDataBuf[19] = get_syncturn_status();		// sync-turn status
	if (TxDataBuf[19] >= SYNC_TURN_COMP){
		set_sync_flg_send_comp();
	}
	TxDataBuf[20] = get_turn_status();	// turn table status
	if(TxDataBuf[20] >= ST_TURN_ANG_COMP){
		set_turn_flg_send_comp();
	}

	TxDataBuf[21] = 0;							// reserved
	TxDataBuf[22] = 0;							// reserved

	TxDataBuf[TxBufSize-1] = TX_FOOTER;     //! ETX

	HAL_UART_Transmit_DMA(&huart3, TxDataBuf,TxBufSize);	// send to NUC
}

/******************************************************************************/
/*           NUC UART communication procedure    						      */
/******************************************************************************/
void set_nuc_receive_flag(void) {
	nuc_receive_flg = 1;
}

void reset_nuc_receive_flag(void) {
	nuc_receive_flg = 0;
}

uint8_t get_nuc_receive_flag(void) {
	return nuc_receive_flg;
}

void nuc_init(void) {

	for (int i = 0 ; i < RxBufSize ; i++) {
		nuc_rx_buf[i] = 0;
		RxDataBuf[i] = 0;
	}
	HAL_UART_Receive_DMA(&huart3, nuc_rx_buf, RxBufSize);
}

// DMAエラーを監視し、エラー発生したらリセットする関数
void monitor_uart_dma_error(void) {

	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) ||
		__HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE) ||
		__HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE) ||
		__HAL_UART_GET_FLAG(&huart3, UART_FLAG_PE)) {
			HAL_UART_Abort(&huart3);
			HAL_UART_Receive_DMA(&huart3, nuc_rx_buf, RxBufSize);
			reset_nuc_receive_flag();
	}
}

void check_nuc_receive_data(void) {

	if (get_nuc_receive_flag() != 0) {
		for (int i = 0 ; i < RxBufSize ; i++) {
			RxDataBuf[i] = nuc_rx_buf[i];
		}
		reset_nuc_receive_flag();

		if ((RxDataBuf[0] == RX_HEADER) && (RxDataBuf[1] == RX_LENGTH) && (RxDataBuf[13] == RX_FOOTER)) {
			RxDataCom = RxDataBuf[2]; 				// COMM
			if ((RxDataCom == NUC_CMND) || (RxDataCom == REQ_SYNC_TURN) || (RxDataCom == TURN_ANG_MODE)) {
				// 正規データ受信
				RxDataLsp = ((RxDataBuf[3] << 8) | RxDataBuf[4]);	// lsp
				RxDataRsp = ((RxDataBuf[5] << 8) | RxDataBuf[6]);	// rsp
				RxDataLidarArea = RxDataBuf[7];                   	// LIDAR Area
				RxDataCtrlOut1 = RxDataBuf[8];                      // control_out1
				RxDataTsp = ((RxDataBuf[9] << 8) | RxDataBuf[10]);  // tsp
				RxDataCtrlOut3 = RxDataBuf[11];
				RxDataCtrlOut4 = RxDataBuf[12];

				if (RxDataCom == REQ_SYNC_TURN) {						//0x11でシンクロターン要求
					uint8_t syncspd = (RxDataBuf[3] & 0b00001100) >> 2; //2-3bitはターン速度　01:低速, 10:標準, 11:高速
					uint8_t syncdir = RxDataBuf[3] & 0b00000011; 		//回転方向は0-1bit　01:右90度, 10:左90度, 11:近いほうに180度
					uint16_t pfangle = 0;
					pfangle = RxDataBuf[5];
					pfangle = pfangle * 256;
					pfangle += RxDataBuf[6];

					set_sync_nuc_command(REQ_SYNC_TURN, syncspd, syncdir, pfangle);
				} else {
					set_sync_nuc_command(NO_REQ_SYNC_TURN, LOW, R90T, 0);
				}
				// Send resp data to NUC
				HAL_UART_MakeTxData();
				set_utimer(CNT_GO5, CNT_PERIOD5);
				set_com_start_flag();

				// NUCからの電文受信完了したらcanの送信を開始する
				set_can_trans_start();
			} else {
				// デバック用データ受信
				for (int i = 0 ; i < RxBufSize ; i++){
					set_command_data(RxDataBuf[i], i);
				}
				set_cmd_process_flag();
			}
		} else {
			// 通信が失敗した場合、リングバッファの内容を削除する
			HAL_UART_Abort(&huart3);
			HAL_UART_Receive_DMA(&huart3, nuc_rx_buf, RxBufSize);
		}
	}
}

// 受信完了時に割り込む関数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == huart3.Instance) {
		set_nuc_receive_flag();
	}
}

void chk_nuc_comm(void) {

	if ((ck_emerg_stop() != NOT_EMERGENCY) && (ck_emerg_stop() < EMERGENCY_CT3)) {
		init_RXBuffer();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart6.Instance) {
		lift_uart_reveive();   // 送信完了後、BusラインをReceiveに切り替え
	}
}
