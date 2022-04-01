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
static uint8_t can1_first = 0;	// リングバッファの取り出し位置
static uint8_t can1_last = 0;	// リングバッファの格納位置
static uint8_t can1_ring_buf[RING_LENGTH][BUF_LENGTH];
// リングバッファのデータ構造
// 1次元目；リングバッファのindex
// 2次元目：[id_low][id_high][d0][d1][d2][d3][d4][d5][d6][d7]

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
HAL_StatusTypeDef can1_transmit(void) {

	static CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint32_t remain_box = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	uint16_t id;
	uint8_t data[8];
	HAL_StatusTypeDef result = HAL_OK;

	// 送信するデータが無くなるかメールボックスの空きが無くなるまで送信を行う
//	while (remain_box > 0) {
	if (remain_box > 0) {
		// CANの送信BOXに空きがある場合のみ送信を行う
		if (can1_deque(&id, data)) {
			// CANの設定
			TxHeader.StdId = id;			// CAN ID
			TxHeader.RTR = CAN_RTR_DATA;	// データフレームを指定
			TxHeader.IDE = CAN_ID_STD;		// 11 bit ID (標準ID)
			TxHeader.DLC = 8;				// Data Length 8Byte
			TxHeader.TransmitGlobalTime = DISABLE;

			result = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);

			// 再度メールボックスの空きを調べる
			remain_box = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
		} else {
			// 送信するデータが無ければループを抜ける
//			break;
		}
	}
	return result;
}

uint8_t can1_enque(uint16_t id, uint8_t *pdata) {
	// リングバッファに値を格納するための関数
	union LongByte id_cnv;
	uint8_t ret = 0;

	// リングバッファに余裕があるかどうかを確認
	// 格納位置の次が先頭位置であれば失敗
	if ((can1_last + 1) != can1_first) {
		id_cnv.l_val = 0;	//共用体の初期化
		id_cnv.w_val[0] = id;

		can1_ring_buf[can1_last][0] = id_cnv.b_val[0];
		can1_ring_buf[can1_last][1] = id_cnv.b_val[1];

		for (int i = 0; i < DATA_LENGTH; i++) {
			can1_ring_buf[can1_last][i+2] = pdata[i];
		}

		// リングバッファの格納位置を更新
		can1_last++;
		if (can1_last >= RING_LENGTH) {
			can1_last = 0;
		}
		ret = 1;
	}

	return ret;
}

uint8_t can1_deque(uint16_t *p_id, uint8_t *pdata) {
	// キューからデータを取り出すための関数
	union LongByte id_cnv;
	uint8_t ret = 0;

	// 送信すべきデータがあるかどうかを確認
	// 先頭位置と格納位置が同一であれば、送信するデータ無しと判断
	if (can1_first != can1_last) {
		id_cnv.l_val = 0;	//共用体の初期化
		id_cnv.b_val[0] = can1_ring_buf[can1_first][0];
		id_cnv.b_val[1] = can1_ring_buf[can1_first][1];
		*p_id = id_cnv.w_val[0];

		for (int i = 0; i < DATA_LENGTH; i++) {
			pdata[i] = can1_ring_buf[can1_first][i+2];
		}
		// リングバッファの先頭位置を更新
		can1_first++;
		if (can1_first >= RING_LENGTH) {
			can1_first = 0;
		}
		ret = 1;
	}

	return ret;
}
