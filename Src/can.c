/*
 * can.c
 *
 *  Created on: 2021/11/10
 *      Author: Katte
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "common_func.h"
#include "wheel.h"

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
/* Private define ------------------------------------------------------------*/
#define RING_LENGTH	100
#define BUF_LENGTH	10
#define DATA_LENGTH	8

/* Private variables ---------------------------------------------------------*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
/*
 * CANメッセージ受信時にコールバックされる関数
 * 受信データをIDごとに振り分けてストアする
 */
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t	Rxdata_tmp[8] = {0,0,0,0,0,0,0,0};
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, Rxdata_tmp) == HAL_OK){
		switch(RxHeader.StdId){
//		バッテリーの処理　共存させるために頑張らないと
//		case 0x100:
//			memcpy(RxData_id100, Rxdata_tmp, sizeof(Rxdata_tmp));
//			break;
//		case 0x101:
//			memcpy(RxData_id101, Rxdata_tmp, sizeof(Rxdata_tmp));
//			break;

		case (SDO_RX_ID + L_WHEEL_ID):
			// 左走行モータードライバからの返答
			receive_wheel_motor_data(Rxdata_tmp, L_WHEEL_ID);
			break;

		case (SDO_RX_ID + R_WHEEL_ID):
			// 右走行モータードライバからの返答
			receive_wheel_motor_data(Rxdata_tmp, R_WHEEL_ID);
			break;

		case (EMCY_ID + L_WHEEL_ID):
			// 左走行モータードライバからのエマージェンシーメッセージ
			receive_wheel_motor_error_data(Rxdata_tmp, L_WHEEL_ID);
			break;

		case (EMCY_ID + R_WHEEL_ID):
			// 右走行モータードライバからのエマージェンシーメッセージ
			receive_wheel_motor_error_data(Rxdata_tmp, R_WHEEL_ID);
			break;

		case 0x701:
			// モータードライバからのハートビート
			break;
		}
	}
}

// CAN受信フィルタの設定
void can_filter_setting(void) {

	// 全IDを受信する
	CAN_FilterTypeDef filter;
	filter.FilterIdHigh         = 0;                        // フィルターID(上位16ビット)
	filter.FilterIdLow          = 0;                        // フィルターID(下位16ビット)
	filter.FilterMaskIdHigh     = 0;                        // フィルターマスク(上位16ビット)
	filter.FilterMaskIdLow      = 0;                        // フィルターマスク(下位16ビット)
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;    // フィルタースケール
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // フィルターに割り当てるFIFO
	filter.FilterBank           = 0;                        // フィルターバンクNo
	filter.FilterMode           = CAN_FILTERMODE_IDMASK;    // フィルターモード
	filter.SlaveStartFilterBank = 14;                       // スレーブCANの開始フィルターバンクNo
	filter.FilterActivation     = ENABLE;                   // フィルター無効／有効
	HAL_CAN_ConfigFilter(&hcan1, &filter);
}

// CAN送信用の関数
HAL_StatusTypeDef can1_transmit(uint16_t id, uint8_t *pdata) {

	static CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint32_t remain_box = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	HAL_StatusTypeDef result = HAL_OK;

	if (remain_box > 0) {
		// CANの送信BOXに空きがある場合のみ送信を行う
		// CANの設定
		TxHeader.StdId = id;			// CAN ID
		TxHeader.RTR = CAN_RTR_DATA;	// データフレームを指定
		TxHeader.IDE = CAN_ID_STD;		// 11 bit ID (標準ID)
		TxHeader.DLC = 8;				// Data Length 8Byte
		TxHeader.TransmitGlobalTime = DISABLE;

		result = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, pdata, &TxMailbox);
	} else {
		result = HAL_ERROR;
	}
	return result;
}
