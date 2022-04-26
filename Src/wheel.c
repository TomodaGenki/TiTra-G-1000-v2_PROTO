/*
 * wheel.c
 *
 *  Created on: 2021/03/28
 *      Author: Takumi
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "wheel.h"
#include "nuc.h"
#include "Cntrl_Proc.h"
#include "common_func.h"
#include "stdlib.h"
#include "stm32f4xx_hal_can.h"
#include "math.h"
#include "can.h"
#include "conf.h"

/* External variables --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
enum Parameter_Setting {
	Set_Drive_Mode,
	Set_Accel,
	Set_Decel,
	Set_Polarity,
	Set_D_Brk,
	Set_D_Brk_Decel,
	Param_Num
};

enum whl_can_data {
	Response,
	Index_Low,
	Index_High,
	Sub_Index,
	Data_Low,
	Data_Mdl_Low,
	Data_Mdl_High,
	Data_High,
	Whl_Data_Num
};

enum emcy_can_data {
	Code_Low,
	Code_High,
	Resister,
	Field_M0,
	Field_M1,
	Field_M2,
	Field_M3,
	Field_M4,
	Emcy_Data_Num
};

#define MAX_MOTOR_ROT		4500.0	// モーター回転軸の最大回転数4500rpm
#define MAX_VELOCTY_ORDER	26500.0	// 0 - 30000 → 0 - 2.0m/s
#define MAX_VELOCTY			2000.0	// 2.0m/s
#define PULSE_PER_ROUND		16384.0
#define GEAR_RATIO_ECO_M	21.0
#define GEAR_RATIO_PRO_M	22.0
#define GEAR_RATIO			GEAR_RATIO_PRO_M
#define WHEEL_RAD			200.0

#define	DOWN_SLOPE		10
#define	LOW_SPDLIMIT	200
#define RESEND_WAIT		1000
#define THRESH_STOP_VEL	100
#define THRESH_STOP_NUM	5		// 何回連続でエンコーダ値が更新なければ停止したとみなすかのしきい値

#define MOTOR1_ALARM	0x01
#define MOTOR2_ALARM	0x02
#define WHEEL_LOGSIZE	6000
#define WHEEL_LOG_CYCLE 10		// ログ計測周期[msec]

#define ENC_VAL_MAX		2147483647

// CAN関連の定義

// Index情報の定義
#define DRIVE_MODE		0x6060		// 運転モード
#define CNTRL_WORD		0x6040		// コントロールワード
#define STATUS_WORD		0x6041		// ステータスワード
#define TARGET_SPEED	0x60FF		// 目標速度
#define ACCEL_PROFILE	0x6083		// 加速度
#define DECEL_PROFILE	0x6084		// 減速度
#define MAX_ACCEL		0x60C5		// 最大加速度
#define MAX_DECEL		0x60C6		// 最大減速度
#define DEGITAL_IO		0x60FE
#define BRAKE_PARAM		0x3002		// ブレーキのパラメータ設定
#define ENCODER_OBJ		0x6064		// エンコーダ情報
#define POLARITY_OBJ	0x607E		// 極性
#define D_BRAKE_PRAM	0x3007		// ダイナミックブレーキ設定

// Sub-Index情報の定義
#define SUB_INDEX_0		0x00
#define SUB_INDEX_1		0x01
#define SUB_INDEX_2		0x02
#define SUB_INDEX_3		0x03
#define SUB_INDEX_4		0x04
#define SUB_INDEX_5		0x05
#define SUB_INDEX_6		0x06
#define SUB_INDEX_7		0x07

// リクエスト情報の定義
#define WRITE_REQ_4B	0x23
#define WRITE_REQ_2B	0x2B
#define WRITE_REQ_1B	0x2F
#define WRITE_RSP		0x60
#define READ_REQ		0x40

// DSP402に準ずるステータスの定義
#define NOT_READY_TO_SWITCH_ON	0x00
#define SWITCH_ON_DISABLE		0x40
#define READY_TO_SWITCH_ON		0x21
#define SWITCHED_ON				0x23
#define OPERATION_ENABLED		0x27
#define QUICK_STOP_ACTIVE		0x07
#define FAULT_REACTION_ACTIVE	0x0F
#define FAULT					0x08
// Smartris特有のステータス
#define SAFETY					0xFF
// 左右のモーターのステータスが同期しない時
#define SYNC_ERR				0xFE
// この実装独自のステータス
#define SWITCHED_ON_BRK_ON		0x24
#define SWITCHED_ON_BRK_OFF		0x25
// ステータスワードからステータスを取得するためのマスク
#define STATUS_MASK1	0x4F
#define STATUS_MASK2	0x6F
#define SAFETY_MASK		0x4000

// コントロールワードの定義
#define SHUT_DOWN			0x06
#define SWITCH_ON			0x07
#define DISABLE_VOLTAGE		0x00
#define DISABLE_OPERATION	0x07
#define ENABLE_OPERATION	0x0F
#define FAULT_RESET			0x80

// パラメーターの定義
#define SPD_PROFILE_MODE	0x03
#define ACCEL_RATE		((2387.0/60.0)*PULSE_PER_ROUND) // ECO-Mで加速度1.2m/ssを達成するための値(rpm)
#define DECEL_RATE		((2387.0/60.0)*PULSE_PER_ROUND) // ECO-Mで加速度1.2m/ssを達成するための値(rpm)
#define AUTO_BRAKE		0
#define MANUAL_BRAKE	1
#define BRAKE_ON		0x00000000
#define BRAKE_OFF		0x00000001
#define POLARITY_REV	0x40
#define READ_DATA		0
#define D_BRK_ENABLE	1
#define D_BRK_DECEL		45		// ダイナミックブレーキの減速度[rpm*100/s]


/* Private variables ---------------------------------------------------------*/
static WHL_MTR_DATA l_motor_data;
static WHL_MTR_DATA r_motor_data;
static uint8_t wheel_motor_alarm = 0;
static uint8_t wheel_stopped = 0;
static uint8_t wheel_dump_req = 0;
static uint8_t wheel_dump_comp = 0;
static uint8_t flg_can_trans_start = 0;

#if WHEEL_LOG_DUMP
// 開発用ログ変数
static int16_t	ref_left = 0;
static int16_t	ref_right = 0;
static int16_t	log_cnt = 0;
static int16_t	log_idx = 0;
static int32_t	log_time[WHEEL_LOGSIZE]	= {0};
static int16_t	log_ref_vel_r[WHEEL_LOGSIZE]	= {0};
static int16_t	log_ref_vel_l[WHEEL_LOGSIZE] 	= {0};
static int16_t	log_act_vel_r[WHEEL_LOGSIZE]	= {0};
static int16_t	log_act_vel_l[WHEEL_LOGSIZE] 	= {0};
static int32_t	log_act_dist_r[WHEEL_LOGSIZE]	= {0};
static int32_t	log_act_dist_l[WHEEL_LOGSIZE] 	= {0};
#endif
/* Private functions -------------------------------------------------------- */
uint16_t ck_l_wheel_error(void);
uint16_t ck_r_wheel_error(void);

/******************************************************************************/
/*           Wheel motor GPIO control function								  */
/******************************************************************************/
void wheel_relay_off(void) {
	HAL_GPIO_WritePin(O_DriveRelay_GPIO_Port, O_DriveRelay_Pin, GPIO_PIN_RESET);// MOTOR POWER OFF
}

void wheel_relay_on(void) {
	HAL_GPIO_WritePin(O_DriveRelay_GPIO_Port, O_DriveRelay_Pin, GPIO_PIN_SET);// MOTOR POWER ON
}

void add_relay_on(void) {
	HAL_GPIO_WritePin(O_AddRelay_GPIO_Port, O_AddRelay_Pin, GPIO_PIN_SET);
}

void add_relay_off(void) {
	HAL_GPIO_WritePin(O_AddRelay_GPIO_Port, O_AddRelay_Pin, GPIO_PIN_RESET);
}

/******************************************************************************/
/*           Wheel motor alarm control function								  */
/******************************************************************************/
void scan_wheelmotor_alarm(void) {

	// motor1 driver alarm signal
	if(ck_l_wheel_error() != 0) {
		wheel_motor_alarm |= MOTOR1_ALARM;
	} else {
		wheel_motor_alarm &= (~MOTOR1_ALARM);
	}

	// motor2 driver alarm signal
	if(ck_r_wheel_error() != 0){
		wheel_motor_alarm |= MOTOR2_ALARM;
	} else {
		wheel_motor_alarm &= (~MOTOR2_ALARM);
	}
}

uint8_t ck_wheel1_motor_alarm(void) {
	return wheel_motor_alarm & MOTOR1_ALARM;
}

uint8_t ck_wheel2_motor_alarm(void) {
	return wheel_motor_alarm & MOTOR2_ALARM;
}

void reset_alarm(void){
	reset_motor1_alarm();
	reset_motor2_alarm();
}


void reset_motor1_alarm(void){
	HAL_GPIO_WritePin(O_Wheel1Reset_GPIO_Port, O_Wheel1Reset_Pin, GPIO_PIN_RESET);
}


void reset_motor2_alarm(void){
	HAL_GPIO_WritePin(O_Wheel2Reset_GPIO_Port, O_Wheel2Reset_Pin, GPIO_PIN_RESET);
}


void recover_alarm(void){
	recover_motor1_alarm();
	recover_motor2_alarm();
}

void recover_motor1_alarm(void) {
	HAL_GPIO_WritePin(O_Wheel1Reset_GPIO_Port, O_Wheel1Reset_Pin, GPIO_PIN_SET);
}


void recover_motor2_alarm(void) {
	HAL_GPIO_WritePin(O_Wheel2Reset_GPIO_Port, O_Wheel2Reset_Pin, GPIO_PIN_SET);
}

void reset_motor_alarm_auto(void){

}

/******************************************************************************/
/*           Wheel motor CAN control function								  */
/******************************************************************************/
uint8_t transmit_motor_control_data(uint16_t id, uint8_t request, uint16_t index, uint8_t sub_index, int32_t data) {

	union LongByte tx_cnv;
	uint8_t tx_data[8];
	uint8_t ret = 0;

	tx_cnv.l_val = 0;	//共用体の初期化

	// 送信データの設定
	tx_data[0] = request;

	tx_cnv.w_val[0] = index;
	tx_data[1] = tx_cnv.b_val[0];
	tx_data[2] = tx_cnv.b_val[1];

	tx_data[3] = sub_index;

	tx_cnv.l_val = data;
	tx_data[4] = tx_cnv.b_val[0];
	tx_data[5] = tx_cnv.b_val[1];
	tx_data[6] = tx_cnv.b_val[2];
	tx_data[7] = tx_cnv.b_val[3];

	can1_transmit(id, tx_data);
	ret = 1;

	return ret;
}

// モータードライバからの受信データを各モーターの構造体へ格納する
void receive_wheel_motor_data(uint8_t *receive_data, uint8_t l_r) {

	WHL_MTR_DATA *motor_data;
	union LongByte rx_cnv;
	uint16_t index;
	uint8_t sub_index;

	if (l_r == L_WHEEL_ID) {
		motor_data = &l_motor_data;
	} else {
		motor_data = &r_motor_data;
	}

	rx_cnv.l_val = 0;	//共用体の初期化

	rx_cnv.b_val[0] = receive_data[Index_Low];
	rx_cnv.b_val[1] = receive_data[Index_High];
	index = rx_cnv.w_val[0];
	sub_index = receive_data[Sub_Index];

	// 取得したデータを共用体に格納しておく
	for (int i = 0; i < 4; i++) {
		rx_cnv.b_val[i] = receive_data[Data_Low + i];
	}

	// 受信したindexによって構造体に情報を格納する
	switch(index) {
	case STATUS_WORD:
		motor_data->whl_status = rx_cnv.w_val[0];
		break;

	case BRAKE_PARAM:
		if (sub_index == SUB_INDEX_5) {
			motor_data->brake_mode = rx_cnv.b_val[0];
		} else if (sub_index == SUB_INDEX_6) {
			motor_data->brake_status = rx_cnv.b_val[0];
		}
		break;

	case 0x60FE:

		break;

	case ENCODER_OBJ:
		motor_data->whl_encoder = -1 * rx_cnv.l_val;//符合を反転させる
		break;
	}

	// 応答チェック用
	volatile uint8_t dumm = 0;
	if(l_r == L_WHEEL_ID){
		switch(index) {
		case STATUS_WORD:
			dumm++;
			break;
		case DRIVE_MODE:
			dumm++;
			break;
		case ACCEL_PROFILE:
			dumm++;
			break;
		case DECEL_PROFILE:
			dumm++;
			break;
		case POLARITY_OBJ:
			dumm++;
			break;
		case D_BRAKE_PRAM:
			dumm++;
			break;
		case BRAKE_PARAM:
			dumm++;
			break;
		case TARGET_SPEED:
			dumm++;
			break;
		}
	}
}

// モータードライバからのエラー情報を各モーターの構造体へ格納する
void receive_wheel_motor_error_data(uint8_t *receive_data, uint8_t l_r) {

	WHL_MTR_DATA *motor_data;
	union LongByte rx_cnv;
	uint16_t err_code;

	if (l_r == L_WHEEL_ID) {
		motor_data = &l_motor_data;
	} else {
		motor_data = &r_motor_data;
	}

	rx_cnv.l_val = 0;	//共用体の初期化

	rx_cnv.b_val[0] = receive_data[Code_Low];
	rx_cnv.b_val[1] = receive_data[Code_High];
	err_code = rx_cnv.w_val[0];

	motor_data->whl_error = err_code;
}

uint16_t get_l_wheel_status(void){
	return l_motor_data.whl_status;
}

uint16_t get_r_wheel_status(void){
	return r_motor_data.whl_status;
}

uint8_t ck_l_wheel_brake_mode(void) {
	return l_motor_data.brake_mode;
}

uint8_t ck_r_wheel_brake_mode(void) {
	return r_motor_data.brake_mode;
}

uint8_t ck_l_wheel_brake_status(void) {
	return l_motor_data.brake_status;
}

uint8_t ck_r_wheel_brake_status(void) {
	return r_motor_data.brake_status;
}

uint16_t ck_l_wheel_error(void) {
	return l_motor_data.whl_error;
}

uint16_t ck_r_wheel_error(void) {
	return r_motor_data.whl_error;
}

uint32_t get_l_wheel_encoder(void) {
	return l_motor_data.whl_encoder;
}

uint32_t get_r_wheel_encoder(void) {
	return r_motor_data.whl_encoder;
}

void set_can_trans_start(){
	flg_can_trans_start = 1;
}

void reset_can_trans_start(){
	flg_can_trans_start = 0;
}

uint8_t get_can_trans_start(){
	return flg_can_trans_start;
}

void set_STO(){
	HAL_GPIO_WritePin(O_Wheel_STO_GPIO_Port, O_Wheel_STO_Pin, GPIO_PIN_SET);
}


void reset_STO(){
	HAL_GPIO_WritePin(O_Wheel_STO_GPIO_Port, O_Wheel_STO_Pin, GPIO_PIN_RESET);
}


uint8_t set_drive_mode(uint8_t mode) {

	uint8_t l_result = 0;
	uint8_t r_result = 0;
	uint8_t result = 0;

	l_result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_1B, DRIVE_MODE, SUB_INDEX_0, mode);
	r_result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_1B, DRIVE_MODE, SUB_INDEX_0, mode);

	if ((l_result == 1) && (r_result == 1)) {
		result = 1;
	}
	return result;
}

uint8_t set_control_word(uint16_t mode) {

	uint8_t l_result = 0;
	uint8_t r_result = 0;
	uint8_t result = 0;

	l_result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_2B, CNTRL_WORD, SUB_INDEX_0, mode);
	r_result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_2B, CNTRL_WORD, SUB_INDEX_0, mode);

	if ((l_result == 1) && (r_result == 1)) {
		result = 1;
	}
	return result;
}

void read_status_word(void) {
	transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, STATUS_WORD, SUB_INDEX_0, READ_DATA);
	transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, STATUS_WORD, SUB_INDEX_0, READ_DATA);
}

uint8_t change_wheel_brake_mode(uint16_t mode) {
	// ブレーキモードの変更
	uint8_t l_result = 0;
	uint8_t r_result = 0;
	uint8_t result = 0;

	l_result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_2B, BRAKE_PARAM, SUB_INDEX_5, mode);
	r_result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_2B, BRAKE_PARAM, SUB_INDEX_5, mode);

	if ((l_result == 1) && (r_result == 1)) {
		result = 1;
	}
	return result;
}

void read_brake_mode(void) {
	// ブレーキモードの確認
	transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, BRAKE_PARAM, SUB_INDEX_5, READ_DATA);
	transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, BRAKE_PARAM, SUB_INDEX_5, READ_DATA);
}


uint8_t wheel_set_brake(uint32_t brake) {

	uint8_t l_result = 0;
	uint8_t r_result = 0;
	uint8_t result = 0;

	l_result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, DEGITAL_IO, SUB_INDEX_1, brake);
	r_result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, DEGITAL_IO, SUB_INDEX_1, brake);

	if ((l_result == 1) && (r_result == 1)) {
		result = 1;
	}
	return result;
}

void read_brake_status(void) {
	// ブレーキが解除されているか確認
	transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, BRAKE_PARAM, SUB_INDEX_6, READ_DATA);
	transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, BRAKE_PARAM, SUB_INDEX_6, READ_DATA);
}

void reset_brake_status(void) {
	// ブレーキ状態は 0:ロック、1:アンロックのため、明示的にリセットを示すために0xFFを代入
	l_motor_data.brake_status = 0xFF;
	r_motor_data.brake_status = 0xFF;
}

/******************************************************************************/
/*           Wheel motor speed control function								  */
/******************************************************************************/
int32_t calc_wheel_speed(int16_t order) {

	double order_speed = ((double)order / MAX_VELOCTY_ORDER) * MAX_VELOCTY;	// 0 - 30000の指令を0 - 2000mm/sの速度に変換
	double rotation_speed;
	double max_guard = (MAX_MOTOR_ROT/60.0) * PULSE_PER_ROUND;

	rotation_speed = ((order_speed / (WHEEL_RAD * M_PI)) * GEAR_RATIO);		// 速度(mm/s)をモーターの回転数(round/sec)に変換
	rotation_speed *= PULSE_PER_ROUND;	// r/s をモーター指令の単位inc/sに変換

	if (rotation_speed > max_guard) {
		// 4500rpmで上限ガード(モーターの性能上限)
		rotation_speed = max_guard;
	}

	return (int32_t)rotation_speed;
}


void wheel_log_update(){
	// NUCからの速度指示値とエンコーダ情報をログに残す関数
#if WHEEL_LOG_DUMP
	static uint32_t wheel_time = 0;

	// 速度指示値をm/sに直す
	double ref_vel_r = (double)ref_right * (MAX_VELOCTY / 1000.0) / MAX_VELOCTY_ORDER;
	double ref_vel_l = (double)ref_left * (MAX_VELOCTY / 1000.0) / MAX_VELOCTY_ORDER;


	// 移動量の計算
	double resolution = 1.74e-6;	// エンコーダ分解能[m]
	uint32_t wheel_dist_right = get_r_wheel_encoder();// * resolution;;
	uint32_t wheel_dist_left = get_l_wheel_encoder();// * resolution;

	// 速度の計算
	static uint32_t r_wheel_enc_old = 0;
	static uint32_t l_wheel_enc_old = 0;
	int32_t r_wheel_enc = get_r_wheel_encoder();
	int32_t l_wheel_enc = get_l_wheel_encoder();
	int32_t r_wheel_enc_diff = r_wheel_enc - r_wheel_enc_old;
	int32_t l_wheel_enc_diff = l_wheel_enc - l_wheel_enc_old;
	double wheel_vel_right= r_wheel_enc_diff * resolution / 0.01;
	double wheel_vel_left = l_wheel_enc_diff * resolution / 0.01;
	r_wheel_enc_old = r_wheel_enc;
	l_wheel_enc_old = l_wheel_enc;

	// ログ用配列に格納
	if(log_cnt % WHEEL_LOG_CYCLE == 0){
		log_time[log_idx]		= wheel_time;
		log_ref_vel_r[log_idx]	= (int16_t)(ref_vel_r * 1000.0);
		log_ref_vel_l[log_idx]	= (int16_t)(ref_vel_l * 1000.0);
		log_act_vel_r[log_idx]	= (int16_t)(wheel_vel_right * 1000.0);
		log_act_vel_l[log_idx]	= (int16_t)(wheel_vel_left * 1000.0);
		log_act_dist_r[log_idx]	= r_wheel_enc;//(int16_t)(wheel_dist_right * 1000.0);
		log_act_dist_l[log_idx]	= l_wheel_enc;//(int16_t)(wheel_dist_left * 1000.0);
		if (log_idx < (WHEEL_LOGSIZE - 1)){
			log_idx++;
		}
	}
	log_cnt++;
	wheel_time++;
#endif
}


void wheel_log_dump() {
	// ログを出力する関数
#if WHEEL_LOG_DUMP
	extern void uart2_transmitte(char *p);
	char buf[600];

	memset(buf, 0x00, sizeof(buf));
	sprintf(buf, "time,ref_vel_r,ref_vel_l,act_vel_r,act_vel_l,act_dist_r,act_dist_l\r\n");
	uart2_transmitte(buf);
	for(int i = 0; i < WHEEL_LOGSIZE; i++) {
		memset(buf, 0x00, sizeof(buf));
		//             1   2   3   4   5   6   7
		sprintf(buf, "%d, %d, %d, %d, %d, %d, %d\r\n",
				log_time[i],			// 1
				log_ref_vel_r[i],		// 2
				log_ref_vel_l[i],		// 3
				log_act_vel_r[i],		// 4
				log_act_vel_l[i],		// 5
				log_act_dist_r[i],		// 6
				log_act_dist_l[i]		// 7
				);
		uart2_transmitte(buf);
		if (i != 0 && log_time[i+1] == 0){
			break;
		}
	}
#endif
	set_wheel_dump_comp();
}

void set_wheel_dump_req(){
	wheel_dump_req = 1;
}

void reset_wheel_dump_req(){
	wheel_dump_req = 0;
}

uint8_t get_wheel_dump_req(){
	return wheel_dump_req;
}

void set_wheel_dump_comp(){
	wheel_dump_comp = 1;
}

void reset_wheel_dump_comp(){
	wheel_dump_comp = 0;
}

uint8_t get_wheel_dump_comp(){
	return wheel_dump_comp;
}

/******************************************************************************/
/*           Wheel motor control function									  */
/******************************************************************************/
void request_wheel_encoder(void) {
	// エンコーダ情報の要求
	transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, ENCODER_OBJ, SUB_INDEX_0, READ_DATA);
	transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, ENCODER_OBJ, SUB_INDEX_0, READ_DATA);
}

void judge_wheel_stopped(void){
	// タイヤが停止中か判定する
	static uint16_t counter = 0;
	static uint32_t l_wheel_enc_old = 0;
	static uint32_t r_wheel_enc_old = 0;
	uint32_t l_wheel_enc = get_l_wheel_encoder();
	uint32_t r_wheel_enc = get_r_wheel_encoder();
	uint32_t l_enc_diff = abs(l_wheel_enc_old - l_wheel_enc);
	uint32_t r_enc_diff = abs(r_wheel_enc_old - r_wheel_enc);
	wheel_stopped = 0;
	if((l_enc_diff <= THRESH_STOP_VEL) && (r_enc_diff <= THRESH_STOP_VEL)){
		if(counter > THRESH_STOP_NUM){
			wheel_stopped = 1;
		}
		else{
			counter ++;
		}
	}
	else{
		counter = 0;
	}
	l_wheel_enc_old = l_wheel_enc;
	r_wheel_enc_old = r_wheel_enc;
}

uint8_t ck_wheel_stopped(void){
	// 0は回転中, 1は停止中
	return wheel_stopped;
}


void wheel_cntrl(int16_t left, int16_t right){
	// can送信タイミングを制御する関数
	// この関数は1msec周期でmainからコールされる
	static uint8_t trans_phase = 0;
	static uint8_t wheel_status = NOT_READY_TO_SWITCH_ON;
	uint16_t l_wheel_status = 0;
	uint16_t r_wheel_status = 0;

	switch(wheel_status){
	case NOT_READY_TO_SWITCH_ON:
		// ステータスワードのリードリクエスト
		read_status_word();
		// ステータスを確認して状態遷移
		l_wheel_status = get_l_wheel_status();
		r_wheel_status = get_r_wheel_status();
		//if((l_wheel_status & STATUS_MASK1) == SWITCH_ON_DISABLE
		//		&& (r_wheel_status & STATUS_MASK1) == SWITCH_ON_DISABLE){
			wheel_status = SWITCH_ON_DISABLE;
			trans_phase = 0;
		//}
		break;

	case SWITCH_ON_DISABLE:
		if(trans_phase == 0){
			// READY_TO_SWITCH_ONへ遷移要求
			set_control_word(SHUT_DOWN);
			trans_phase++;
		}
		else if(trans_phase < 10){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 10){
			// ステータスワードのリードリクエスト
			read_status_word();
			trans_phase++;
		}
		else if(trans_phase < 20){
			// 待機
			trans_phase++;
		}
		else{
			// ステータスを確認して状態遷移
			l_wheel_status = get_l_wheel_status();
			r_wheel_status = get_r_wheel_status();
			if((l_wheel_status & STATUS_MASK2) == READY_TO_SWITCH_ON
					&& (r_wheel_status & STATUS_MASK2) == READY_TO_SWITCH_ON){
				wheel_status = READY_TO_SWITCH_ON;
				trans_phase = 0;
			}
			else{
				// ステータスが変わってなかったらやり直し
				trans_phase = 0;
			}
		}
		break;

	case READY_TO_SWITCH_ON:
		// 運転モードとパラメーターを設定
		if(trans_phase == 0){
			// 運転モードを速度プロファイルモードに設定
			transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_1B, DRIVE_MODE, SUB_INDEX_0, SPD_PROFILE_MODE);
			transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_1B, DRIVE_MODE, SUB_INDEX_0, SPD_PROFILE_MODE);
			trans_phase++;
		}
		else if(trans_phase < 10){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 10){
			// 加速度を設定
			transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, ACCEL_PROFILE, SUB_INDEX_0, ACCEL_RATE);
			transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, ACCEL_PROFILE, SUB_INDEX_0, ACCEL_RATE);
			trans_phase++;
		}
		else if(trans_phase < 20){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 20){
			// 減速度を設定
			transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, DECEL_PROFILE, SUB_INDEX_0, DECEL_RATE);
			transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, DECEL_PROFILE, SUB_INDEX_0, DECEL_RATE);
			trans_phase++;
		}
		else if(trans_phase < 30){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 30){
			// 左モーターの極性反転
			transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_1B, POLARITY_OBJ, SUB_INDEX_0, POLARITY_REV);
			trans_phase++;
		}
		else if(trans_phase < 40){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 40){
			// ダイナミックブレーキを有効にする
			transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_2B, D_BRAKE_PRAM, SUB_INDEX_1, D_BRK_ENABLE);
			transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_2B, D_BRAKE_PRAM, SUB_INDEX_1, D_BRK_ENABLE);
			trans_phase++;
		}
		else if(trans_phase < 50){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 50){
			// 設定完了したらSWITCHED_ONへ遷移要求
			transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_2B, CNTRL_WORD, SUB_INDEX_0, SWITCH_ON);
			transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_2B, CNTRL_WORD, SUB_INDEX_0, SWITCH_ON);
			trans_phase++;
		}
		else if(trans_phase < 60){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 60){
			// ステータスワードのリードリクエスト
			read_status_word();
			trans_phase++;
		}
		else if(trans_phase < 70){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 70){
			// ステータスを確認して状態遷移
			l_wheel_status = get_l_wheel_status();
			r_wheel_status = get_r_wheel_status();
			if((l_wheel_status & STATUS_MASK2) == SWITCHED_ON
					&& (r_wheel_status & STATUS_MASK2) == SWITCHED_ON){
				if(check_wheel_brake() == 0){
					wheel_status = SWITCHED_ON_BRK_ON;
				}
				else{
					wheel_status = SWITCHED_ON_BRK_OFF;
				}
			}
			trans_phase = 0;
		}
		break;

	case SWITCHED_ON_BRK_ON:
		if(check_wheel_brake() != 0){
			wheel_status = SWITCHED_ON_BRK_OFF;
			trans_phase = 0;
		}
		else if(trans_phase == 0){
			if(ck_emerg_stop() == NOT_EMERGENCY){
				// エラーが無ければOPERATION_ENABLEDへの遷移を開始する
				trans_phase++;
			}
		}
		else if(trans_phase == 1){
			// ブレーキモードを自動に変更
			change_wheel_brake_mode(AUTO_BRAKE);
			trans_phase++;
		}
		else if(trans_phase < 3){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 3){
			//　OPERATION_ENABLEDへ遷移要求
			set_control_word(ENABLE_OPERATION);
			trans_phase++;
		}
		else if(trans_phase < 5){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 5){
			//　ステータスワードのリードリクエスト
			read_status_word();
			trans_phase++;
		}
		else if(trans_phase < 7){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 7){
			l_wheel_status = get_l_wheel_status();
			r_wheel_status = get_r_wheel_status();
			if((l_wheel_status & STATUS_MASK2) == OPERATION_ENABLED
				&& (r_wheel_status & STATUS_MASK2) == OPERATION_ENABLED){
				wheel_status = OPERATION_ENABLED;
			}
			trans_phase = 0;
		}
		break;

	case SWITCHED_ON_BRK_OFF:
		if(trans_phase == 0){
			// ブレーキモードを手動に変更
			change_wheel_brake_mode(MANUAL_BRAKE);
			trans_phase++;
		}
		else if(trans_phase < 3){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 3){
			//　ブレーキ解放
			wheel_set_brake(BRAKE_OFF);
			trans_phase++;
		}
		else if(trans_phase < 5){
			// 待機
			trans_phase++;
		}
		else if(trans_phase == 5){
			if(check_wheel_brake() == 0){
				// ブレーキ解放要求が無ければSWITCHED_ON_BRK_ONへ遷移
				wheel_set_brake(BRAKE_ON);
				wheel_status = SWITCHED_ON_BRK_ON;
			}
			trans_phase = 0;
		}
		break;

	case OPERATION_ENABLED:
		if(ck_emerg_stop() != NOT_EMERGENCY){
			// エラー発生時はモーターのステータスを遷移させて急ブレーキをかける
			set_control_word(DISABLE_OPERATION);
			wheel_status = SWITCHED_ON_BRK_ON;
			trans_phase = 0;
		}
		else if(check_wheel_brake() != 0){
			// ブレーキ解放要求
			set_control_word(DISABLE_OPERATION);
			wheel_status = SWITCHED_ON_BRK_OFF;
			trans_phase = 0;
		}
		else if(get_can_trans_start()){
			// NUCからの電文受信時
			if(trans_phase == 0){
				// 左モーターへ速度指示
				int32_t l_wheel_rot = calc_wheel_speed(left);
				transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, TARGET_SPEED, SUB_INDEX_0, l_wheel_rot);
				trans_phase++;
			}
			else if(trans_phase == 1){
				// 右モーターへ速度指示
				int32_t r_wheel_rot = calc_wheel_speed(right);
				transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, TARGET_SPEED, SUB_INDEX_0, r_wheel_rot);
				trans_phase++;
			}
			else if(trans_phase < 15){
				// 一定時間待機
				trans_phase++;
			}
			else if(trans_phase == 15){
				// 左モーターへエンコーダリクエスト
				transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, ENCODER_OBJ, SUB_INDEX_0, READ_DATA);
				trans_phase++;
			}
			else if(trans_phase == 16){
				// 右モーターへエンコーダリクエスト
				transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, ENCODER_OBJ, SUB_INDEX_0, READ_DATA);
				trans_phase = 0;
				reset_can_trans_start();
			}
		}
		break;
	}
	ref_right = right;
	ref_left = left;
}


void dump_master_status(uint8_t status){
	// デバッグ用に現在のステータスを出力する
	extern void uart2_transmitte(char *p);

	char buf1[60];
	memset(buf1, 0x00, sizeof(buf1));
	if (status == NOT_READY_TO_SWITCH_ON){
		sprintf(buf1, "Wheel master status:  NOT_READY_TO_SWITCH_ON\r\n");
	}
	else if(status == SWITCH_ON_DISABLE){
		sprintf(buf1, "Wheel master status:  SWITCH_ON_DISABLE\r\n");
	}
	else if(status == READY_TO_SWITCH_ON){
		sprintf(buf1, "Wheel master status:  READY_TO_SWITCH_ON\r\n");
	}
	else if(status == SWITCHED_ON){
		sprintf(buf1, "Wheel master status:  SWITCHED_ON\r\n");
	}
	else if(status == OPERATION_ENABLED){
		sprintf(buf1, "Wheel master status:  OPERATION_ENABLED\r\n");
	}
	else if(status == QUICK_STOP_ACTIVE){
		sprintf(buf1, "Wheel master status:  QUICK_STOP_ACTIVE\r\n");
	}
	else if(status == FAULT_REACTION_ACTIVE){
		sprintf(buf1, "Wheel master status:  FAULT_REACTION_ACTIVE\r\n");
	}
	else if(status == FAULT){
		sprintf(buf1, "Wheel master status:  FAULT\r\n");
	}
	else if(status == SAFETY){
		sprintf(buf1, "Wheel master status:  SAFETY\r\n");
	}
	else if(status == SYNC_ERR){
		sprintf(buf1, "Wheel master status:  SYNC_ERR\r\n");
	}
	uart2_transmitte(buf1);
}
