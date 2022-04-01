/*
 * wheel.c
 *
 *  Created on: 2021/03/28
 *      Author: Takumi
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "wheel.h"
#include "nuc.h"
#include "Cntrl_Proc.h"
#include "common_func.h"
#include "stdlib.h"
#include "stm32f4xx_hal_can.h"
#include "math.h"
#include "can.h"

/* External variables --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
enum whl_sequence {
	Whl_PowerOn,
	Whl_Init,
	Whl_ParamSet,
	Whl_Run,
	Drv_Mode_Disable,
	Change_BrkMode_Manu,
	Whl_TorqueOff,
	Drv_Mode_Enable,
	Change_BrkMode_Auto,
	Whl_EmerStop,
};

enum Init_Setting {
	Set_DriveMode,
	Sw_On_Ready,
	Sw_On,
	Sw_On_Check,
	Drive_Enable,
	Drive_Enable_Check,
	Init_Num
};

enum Parameter_Setting {
	Set_Accel,
	Set_Decel,
	Set_Polarity,
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
#define MAX_VELOCTY_ORDER	30000.0	// 0 - 30000 → 0 - 2.0m/s
#define MAX_VELOCTY			2000.0	// 2.0m/s
#define PULSE_PER_ROUND		16384.0
#define GEAR_RATIO_ECO_M	21.0
#define GEAR_RATIO_PRO_M	26.0
#define GEAR_RATIO			GEAR_RATIO_ECO_M
#define WHEEL_RAD			200.0

#define	DOWN_SLOPE		10
#define	LOW_SPDLIMIT	200
#define RESEND_WAIT		1000

#define MOTOR1_ALARM	0x01
#define MOTOR2_ALARM	0x02

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


// パラメーターの定義
#define SPD_PROFILE_MODE	0x03
#define SW_ON_READY			0x06
#define SW_ON				0x07
#define DRV_ENABLE			0x0F
#define DRV_DISABLE			0x07
#define ST_SW_ON			0x0002
#define ST_DRV_ENABLE		0x0004

//#define ACCEL_RATE		((1604.0/60.0)*PULSE_PER_ROUND) // ECO-Mで加速度0.8m/ssを達成するための値(rpm)
//#define DECEL_RATE		((1604.0/60.0)*PULSE_PER_ROUND) // ECO-Mで加速度0.8m/ssを達成するための値(rpm)
#define ACCEL_RATE		((2387.0/60.0)*PULSE_PER_ROUND) // ECO-Mで加速度1.2m/ssを達成するための値(rpm)
#define DECEL_RATE		((2387.0/60.0)*PULSE_PER_ROUND) // ECO-Mで加速度1.2m/ssを達成するための値(rpm)
#define AUTO_BRAKE		0
#define MANUAL_BRAKE	1
#define BRAKE_ON		0x00000000
#define BRAKE_OFF		0x00000001
#define POLARITY_REV	0x40
#define READ_DATA		0

/* Private variables ---------------------------------------------------------*/
static WHL_MTR_DATA l_motor_data;
static WHL_MTR_DATA r_motor_data;
static uint8_t wheel_motor_alarm = 0;
static uint8_t wheel_seq = Whl_PowerOn;

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

	ret = can1_enque(id, tx_data);

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
		motor_data->whl_encoder = rx_cnv.l_val;
		break;
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

uint16_t ck_l_wheel_switch_on(void) {
	return (l_motor_data.whl_status & ST_SW_ON);
}

uint16_t ck_r_wheel_switch_on(void) {
	return (r_motor_data.whl_status & ST_SW_ON);
}

uint16_t ck_l_wheel_drv_enable(void) {
	return (l_motor_data.whl_status & ST_DRV_ENABLE);
}

uint16_t ck_r_wheel_drv_enable(void) {
	return (r_motor_data.whl_status & ST_DRV_ENABLE);
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

void reset_brake_mode(void) {
	// ブレーキモードは 0:自動、1:手動のため、明示的にリセットを示すために0xFFを代入
	l_motor_data.brake_mode = 0xFF;
	r_motor_data.brake_mode = 0xFF;
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

void wheel_set_speed(int16_t left, int16_t right) {

	static int16_t left_old = 0xFFFF;
	static int16_t right_old = 0xFFFF;
	int32_t l_wheel_rot = calc_wheel_speed(left);
	int32_t r_wheel_rot = calc_wheel_speed(right);
	uint8_t l_result = 0;
	uint8_t r_result = 0;

	// 同じ内容の通信を何度も行わないためのガード
	if (left != left_old) {
		l_result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, TARGET_SPEED, SUB_INDEX_0, l_wheel_rot);
		if (l_result == 1) {
			left_old = left;
		}
	}

	if (right != right_old) {
		r_result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, TARGET_SPEED, SUB_INDEX_0, r_wheel_rot);
		if (r_result == 1) {
			right_old = right;
		}
	}
}

/******************************************************************************/
/*           Wheel motor control function									  */
/******************************************************************************/
void monitor_wheel_encoder(void) {

	if (wheel_seq >= Whl_Run) {
		// 起動が完了していないタイミングでは、エンコーダー情報を取得しにいかない
		transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, ENCODER_OBJ, SUB_INDEX_0, READ_DATA);
		transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, ENCODER_OBJ, SUB_INDEX_0, READ_DATA);
	}
}

uint8_t wheel_initialize(void) {
	// DSP402の状態機械に従い、走行モーターを起動する

	static uint8_t init_set = Set_DriveMode;
	static uint8_t retry_cnt = 0;
	uint8_t result = 0;
	uint8_t ret = 0;

	if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
		set_utimer(CNT_WHEEL_WAIT, WHEEL_CAN_WAIT_TIME);

		if (init_set == Set_DriveMode) {
			// 運転モードを速度プロファイルモードに設定
			result = set_drive_mode(SPD_PROFILE_MODE);
			if (result == 1) {
				init_set++;
			}
		} else if (init_set == Sw_On_Ready) {
			// スイッチON準備完了の設定
			result = set_control_word(SW_ON_READY);
			if (result == 1) {
				init_set++;
			}
		} else if (init_set == Sw_On) {
			// スイッチONの設定
			result = set_control_word(SW_ON);
			if (result == 1) {
				init_set++;
			}
		} else if (init_set == Sw_On_Check) {
			//スイッチON状態か確認するためにRead要求を送信
			read_status_word();
			if ((ck_l_wheel_switch_on()) && (ck_r_wheel_switch_on())) {
				init_set++;
				retry_cnt = 0;
			} else {
				retry_cnt++;
				if (retry_cnt > 5) {
					// 一定回数以上読み込んでもスイッチON状態とならない場合、やり直し
					init_set--;
					retry_cnt = 0;
				}
			}
		} else if (init_set == Drive_Enable) {
			// 運転有効の設定
			result = set_control_word(DRV_ENABLE);
			if (result == 1) {
				init_set++;
			}
		} else if (init_set == Drive_Enable_Check) {
			// 運転有効状態か確認するためにRead要求を送信
			read_status_word();
			if ((ck_l_wheel_drv_enable()) && (ck_r_wheel_drv_enable())) {
				init_set++;
				retry_cnt = 0;
			} else {
				retry_cnt++;
				if (retry_cnt > 5) {
					// 一定回数以上読み込んでもスイッチON状態とならない場合、やり直し
					init_set--;
					retry_cnt = 0;
				}
			}
		} else if (init_set == Init_Num) {
			// 設定完了
			ret = 1;
			init_set = Set_DriveMode;
		}
	}
	return ret;
}

// 走行モーターのパラメーターを設定する
uint8_t wheel_parameter(void) {

	static uint8_t parameter_set = Set_Accel;
	uint8_t result = 0;
	uint8_t ret = 0;

	if (parameter_set == Set_Accel) {
		// 加速度を設定
		transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, ACCEL_PROFILE, SUB_INDEX_0, ACCEL_RATE);
		transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, ACCEL_PROFILE, SUB_INDEX_0, ACCEL_RATE);
		parameter_set++;
	} else if (parameter_set == Set_Decel) {
		// 減速度を設定
		transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, DECEL_PROFILE, SUB_INDEX_0, DECEL_RATE);
		transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, DECEL_PROFILE, SUB_INDEX_0, DECEL_RATE);
		parameter_set++;
	} else if (parameter_set == Set_Polarity) {
		// 右モーターの極性を反転させる
		result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_1B, POLARITY_OBJ, SUB_INDEX_0, POLARITY_REV);
		if (result == 1) {
			parameter_set++;
		}
	} else if (parameter_set == Param_Num) {
		// 設定完了
		ret = 1;
		parameter_set = Set_Accel;
	}
	return ret;
}

void wheel_init(void) {

}

void wheel_cntrl(int16_t left, int16_t right) {

	uint8_t result = 0;

	switch(wheel_seq) {
	case Whl_PowerOn:
		// モーターの起動待ち
		if (ck_motor_active() != 0) {
			if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
				wheel_seq = Whl_Init;
				set_utimer(CNT_WHEEL_WAIT, WHEEL_CAN_WAIT_TIME);
			}
		} else {
			// モーター電源投入後、CAN送信を行うまで待ち時間を入れる
			set_utimer(CNT_WHEEL_WAIT, WHEEL_MOTOR_WAKE_UP);
		}
		break;

	case Whl_Init:
		// モーターの状態遷移に従い起動を行う
		if (wheel_initialize() != 0) {
			wheel_seq = Whl_ParamSet;
		}
		break;

	case Whl_ParamSet:
		// パラメーターの設定
		if (wheel_parameter() != 0) {
			wheel_seq = Whl_Run;
		}
		break;

	case Whl_Run:
		// 定常状態　速度指示により動く
		if (ck_emerg_stop() == NOT_EMERGENCY) {
			if ((left == 0) && (right == 0)) {
				if (check_wheel_brake() != 0) {
					// ブレーキ解除スイッチが押された場合、ブレーキモードを手動に変更しブレーキを解除する
					// 運転モードを無効に設定
					result = set_control_word(DRV_DISABLE);
					if (result == 1) {
						set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
						wheel_seq = Drv_Mode_Disable;
					}
				}
			}
			wheel_set_speed(left, right);
		} else {
			wheel_set_speed(0, 0);
			wheel_seq = Whl_EmerStop;
		}
		break;

	case Drv_Mode_Disable:
		// 運転モードが無効となっていることを確認する
		if ((ck_l_wheel_drv_enable() == 0) && (ck_r_wheel_drv_enable() == 0)) {
			reset_brake_mode();
			// ブレーキモードを手動に設定
			result = change_wheel_brake_mode(MANUAL_BRAKE);
			if (result == 1) {
				set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				wheel_seq = Change_BrkMode_Manu;
			}
		} else {
			if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
				// 一定時間経過しても運転モードが書き変わらない場合、運転モード無効指令を再送する
				set_control_word(DRV_DISABLE);
				set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
			} else {
				read_status_word();
			}
		}
		break;

	case Change_BrkMode_Manu:
		// ブレーキモードが書き変わっていることを確認
		if ((ck_l_wheel_brake_mode() == MANUAL_BRAKE) && (ck_r_wheel_brake_mode() == MANUAL_BRAKE)) {
			// ブレーキモードが手動に設定されたのを確認後、ブレーキを解除する
			reset_brake_status();
			result = wheel_set_brake(BRAKE_OFF);
			if (result == 1) {
				set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				wheel_seq = Whl_TorqueOff;
			}
		} else {
			if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
				// 一定時間経過してもブレーキモードが書き変わらない場合、ブレーキモード切り替え指令を再送する
				result = change_wheel_brake_mode(MANUAL_BRAKE);
				if (result == 1) {
					set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				}
			} else {
				read_brake_mode();
			}
		}
		break;

	case Whl_TorqueOff:
		if (check_wheel_brake() == 0) {
			// ブレーキON指令が来た場合、ブレーキモードを自動に設定する
			reset_brake_mode();
			result = change_wheel_brake_mode(AUTO_BRAKE);
			if (result == 1) {
				set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				wheel_seq = Change_BrkMode_Auto;
			}
		} else if ((ck_l_wheel_brake_status() == BRAKE_OFF) && (ck_r_wheel_brake_status() == BRAKE_OFF)) {
			// ブレーキの解除が完了した場合
			// 特に何もしない
		} else {
			if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
				// 一定時間経過してもブレーキが解除されない場合、ブレーキ解除指令を再送する
				result = wheel_set_brake(BRAKE_OFF);
				if (result == 1) {
					set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				}
			} else {
				read_brake_status();
			}
		}
		break;

	case Change_BrkMode_Auto:
		// ブレーキモードが書き変わっていることを確認
		if ((ck_l_wheel_brake_mode() == AUTO_BRAKE) && (ck_r_wheel_brake_mode() == AUTO_BRAKE)) {
			// 運転有効の設定
			set_utimer(CNT_WHEEL_WAIT, WHEEL_CAN_WAIT_TIME);
			wheel_seq = Whl_Init;
		} else {
			if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
				// 一定時間経過してもブレーキモードが書き変わらない場合、ブレーキモード切り替え指令を再送する
				result = change_wheel_brake_mode(AUTO_BRAKE);
				if (result == 1) {
					set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				}
			} else {
				read_brake_mode();
			}
		}
		break;

	case Whl_EmerStop:
		if (ck_emerg_stop() == NOT_EMERGENCY) {
			wheel_seq = Whl_Run;
		} else if (check_wheel_brake() != 0) {
			result = set_control_word(DRV_DISABLE);
			if (result == 1) {
				set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				wheel_seq = Drv_Mode_Disable;
			}
		}
		break;
	}
}
