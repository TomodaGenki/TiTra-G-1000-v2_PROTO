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

#define MAX_MOTOR_ROT		4500.0	// ���[�^�[��]���̍ő��]��4500rpm
#define MAX_VELOCTY_ORDER	26500.0	// 0 - 30000 �� 0 - 2.0m/s
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
#define THRESH_STOP_NUM	5		// ����A���ŃG���R�[�_�l���X�V�Ȃ���Β�~�����Ƃ݂Ȃ����̂������l

#define MOTOR1_ALARM	0x01
#define MOTOR2_ALARM	0x02

#define ENC_VAL_MAX		2147483647

// CAN�֘A�̒�`

// Index���̒�`
#define DRIVE_MODE		0x6060		// �^�]���[�h
#define CNTRL_WORD		0x6040		// �R���g���[�����[�h
#define STATUS_WORD		0x6041		// �X�e�[�^�X���[�h
#define TARGET_SPEED	0x60FF		// �ڕW���x
#define ACCEL_PROFILE	0x6083		// �����x
#define DECEL_PROFILE	0x6084		// �����x
#define MAX_ACCEL		0x60C5		// �ő�����x
#define MAX_DECEL		0x60C6		// �ő匸���x
#define DEGITAL_IO		0x60FE
#define BRAKE_PARAM		0x3002		// �u���[�L�̃p�����[�^�ݒ�
#define ENCODER_OBJ		0x6064		// �G���R�[�_���
#define POLARITY_OBJ	0x607E		// �ɐ�
#define D_BRAKE_PRAM	0x3007		// �_�C�i�~�b�N�u���[�L�ݒ�

// Sub-Index���̒�`
#define SUB_INDEX_0		0x00
#define SUB_INDEX_1		0x01
#define SUB_INDEX_2		0x02
#define SUB_INDEX_3		0x03
#define SUB_INDEX_4		0x04
#define SUB_INDEX_5		0x05
#define SUB_INDEX_6		0x06
#define SUB_INDEX_7		0x07

// ���N�G�X�g���̒�`
#define WRITE_REQ_4B	0x23
#define WRITE_REQ_2B	0x2B
#define WRITE_REQ_1B	0x2F
#define WRITE_RSP		0x60
#define READ_REQ		0x40

// DSP402�ɏ�����X�e�[�^�X�̒�`
#define NOT_READY_TO_SWITCH_ON	0x00
#define SWITCH_ON_DISABLE		0x40
#define READY_TO_SWITCH_ON		0x21
#define SWITCHED_ON				0x23
#define OPERATION_ENABLED		0x27
#define QUICK_STOP_ACTIVE		0x07
#define FAULT_REACTION_ACTIVE	0x0F
#define FAULT					0x08
// Smartris���L�̃X�e�[�^�X
#define SAFETY					0xFF
// ���E�̃��[�^�[�̃X�e�[�^�X���������Ȃ���
#define SYNC_ERR				0xFE
// �X�e�[�^�X���[�h����X�e�[�^�X���擾���邽�߂̃}�X�N
#define STATUS_MASK1	0x4F
#define STATUS_MASK2	0x6F
#define SAFETY_MASK		0x4000

// �R���g���[�����[�h�̒�`
#define SHUT_DOWN			0x06
#define SWITCH_ON			0x07
#define DISABLE_VOLTAGE		0x00
#define DISABLE_OPERATION	0x07
#define ENABLE_OPERATION	0x0F
#define FAULT_RESET			0x80

// �p�����[�^�[�̒�`
#define SPD_PROFILE_MODE	0x03
#define ACCEL_RATE		((2387.0/60.0)*PULSE_PER_ROUND) // ECO-M�ŉ����x1.2m/ss��B�����邽�߂̒l(rpm)
#define DECEL_RATE		((2387.0/60.0)*PULSE_PER_ROUND) // ECO-M�ŉ����x1.2m/ss��B�����邽�߂̒l(rpm)
#define AUTO_BRAKE		0
#define MANUAL_BRAKE	1
#define BRAKE_ON		0x00000000
#define BRAKE_OFF		0x00000001
#define POLARITY_REV	0x40
#define READ_DATA		0
#define D_BRK_ENABLE	1
#define D_BRK_DECEL		45		// �_�C�i�~�b�N�u���[�L�̌����x[rpm*100/s]


/* Private variables ---------------------------------------------------------*/
static WHL_MTR_DATA l_motor_data;
static WHL_MTR_DATA r_motor_data;
static uint8_t wheel_motor_alarm = 0;
static uint8_t wheel_stopped = 0;
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

	tx_cnv.l_val = 0;	//���p�̂̏�����

	// ���M�f�[�^�̐ݒ�
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

// ���[�^�[�h���C�o����̎�M�f�[�^���e���[�^�[�̍\���̂֊i�[����
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

	rx_cnv.l_val = 0;	//���p�̂̏�����

	rx_cnv.b_val[0] = receive_data[Index_Low];
	rx_cnv.b_val[1] = receive_data[Index_High];
	index = rx_cnv.w_val[0];
	sub_index = receive_data[Sub_Index];

	// �擾�����f�[�^�����p�̂Ɋi�[���Ă���
	for (int i = 0; i < 4; i++) {
		rx_cnv.b_val[i] = receive_data[Data_Low + i];
	}

	// ��M����index�ɂ���č\���̂ɏ����i�[����
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
		motor_data->whl_encoder = -1 * rx_cnv.l_val;//�����𔽓]������
		break;
	}
}

// ���[�^�[�h���C�o����̃G���[�����e���[�^�[�̍\���̂֊i�[����
void receive_wheel_motor_error_data(uint8_t *receive_data, uint8_t l_r) {

	WHL_MTR_DATA *motor_data;
	union LongByte rx_cnv;
	uint16_t err_code;

	if (l_r == L_WHEEL_ID) {
		motor_data = &l_motor_data;
	} else {
		motor_data = &r_motor_data;
	}

	rx_cnv.l_val = 0;	//���p�̂̏�����

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
	// �u���[�L���[�h�̕ύX
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
	// �u���[�L���[�h�̊m�F
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
	// �u���[�L����������Ă��邩�m�F
	transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, BRAKE_PARAM, SUB_INDEX_6, READ_DATA);
	transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, BRAKE_PARAM, SUB_INDEX_6, READ_DATA);
}

void reset_brake_status(void) {
	// �u���[�L��Ԃ� 0:���b�N�A1:�A�����b�N�̂��߁A�����I�Ƀ��Z�b�g���������߂�0xFF����
	l_motor_data.brake_status = 0xFF;
	r_motor_data.brake_status = 0xFF;
}

/******************************************************************************/
/*           Wheel motor speed control function								  */
/******************************************************************************/
int32_t calc_wheel_speed(int16_t order) {

	double order_speed = ((double)order / MAX_VELOCTY_ORDER) * MAX_VELOCTY;	// 0 - 30000�̎w�߂�0 - 2000mm/s�̑��x�ɕϊ�
	double rotation_speed;
	double max_guard = (MAX_MOTOR_ROT/60.0) * PULSE_PER_ROUND;

	rotation_speed = ((order_speed / (WHEEL_RAD * M_PI)) * GEAR_RATIO);		// ���x(mm/s)�����[�^�[�̉�]��(round/sec)�ɕϊ�
	rotation_speed *= PULSE_PER_ROUND;	// r/s �����[�^�[�w�߂̒P��inc/s�ɕϊ�

	if (rotation_speed > max_guard) {
		// 4500rpm�ŏ���K�[�h(���[�^�[�̐��\���)
		rotation_speed = max_guard;
	}

	return (int32_t)rotation_speed;
}

void wheel_set_speed(int16_t left, int16_t right) {

	int32_t l_wheel_rot = calc_wheel_speed(left);
	int32_t r_wheel_rot = calc_wheel_speed(right);

	transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, TARGET_SPEED, SUB_INDEX_0, l_wheel_rot);
	transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, TARGET_SPEED, SUB_INDEX_0, r_wheel_rot);

}

/******************************************************************************/
/*           Wheel motor control function									  */
/******************************************************************************/
void request_wheel_encoder(uint8_t status) {
	// �G���R�[�_���̗v��
	transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, ENCODER_OBJ, SUB_INDEX_0, READ_DATA);
	transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, ENCODER_OBJ, SUB_INDEX_0, READ_DATA);
}

void judge_wheel_stopped(void){
	// �^�C������~�������肷��
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
	// 0�͉�]��, 1�͒�~��
	return wheel_stopped;
}

uint8_t wheel_param_set(void) {

	static uint8_t parameter_set = Set_Drive_Mode;
	uint8_t result = 0;
	uint8_t ret = 0;

	if(parameter_set == Set_Drive_Mode){
		// �^�]���[�h�𑬓x�v���t�@�C�����[�h�ɐݒ�
		result = set_drive_mode(SPD_PROFILE_MODE);
		if(result == 1){
			parameter_set++;
		}
	}
	else if (parameter_set == Set_Accel) {
		// �����x��ݒ�
		transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, ACCEL_PROFILE, SUB_INDEX_0, ACCEL_RATE);
		transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, ACCEL_PROFILE, SUB_INDEX_0, ACCEL_RATE);
		parameter_set++;
	} else if (parameter_set == Set_Decel) {
		// �����x��ݒ�
		transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, DECEL_PROFILE, SUB_INDEX_0, DECEL_RATE);
		transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, DECEL_PROFILE, SUB_INDEX_0, DECEL_RATE);
		parameter_set++;
	} else if (parameter_set == Set_Polarity) {
		// �����[�^�[�̋ɐ��𔽓]������
		result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_1B, POLARITY_OBJ, SUB_INDEX_0, POLARITY_REV);
		if (result == 1) {
			parameter_set++;
		}
	} else if(parameter_set == Set_D_Brk){
		// �_�C�i�~�b�N�u���[�L��L���ɂ���
		transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_2B, D_BRAKE_PRAM, SUB_INDEX_1, D_BRK_ENABLE);
		transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_2B, D_BRAKE_PRAM, SUB_INDEX_1, D_BRK_ENABLE);
		parameter_set++;
	} else if(parameter_set == Set_D_Brk_Decel){
		// �_�C�i�~�b�N�u���[�L�̌����x��ݒ�
		transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_2B, D_BRAKE_PRAM, SUB_INDEX_4, D_BRK_DECEL);
		transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_2B, D_BRAKE_PRAM, SUB_INDEX_4, D_BRK_DECEL);
		parameter_set++;
	} else if (parameter_set == Param_Num) {
		// �ݒ芮��
		ret = 1;
		parameter_set = Set_Accel;
	}
	return ret;
}

void wheel_init(void) {

}


static uint8_t get_master_status(void){
	static uint8_t status = NOT_READY_TO_SWITCH_ON;
	static uint16_t unko_timer = 0;

	// �X�e�[�^�X���[�h�̃��[�h�v��
	read_status_word();
	// ���E���[�^�[�̃X�e�[�^�X�ɉ����ă}�X�^�[�̃X�e�[�^�X������
	uint16_t l_wheel_status = get_l_wheel_status();
	uint16_t r_wheel_status = get_r_wheel_status();

	if(l_wheel_status == r_wheel_status){
		if(l_wheel_status & SAFETY_MASK){
			status = SAFETY;
		}
		else if((l_wheel_status & STATUS_MASK1) == NOT_READY_TO_SWITCH_ON){
			status = NOT_READY_TO_SWITCH_ON;
		}
		else if((l_wheel_status & STATUS_MASK1) == SWITCH_ON_DISABLE){
			status = SWITCH_ON_DISABLE;
		}
		else if((l_wheel_status & STATUS_MASK2) == READY_TO_SWITCH_ON){
			status = READY_TO_SWITCH_ON;
		}
		else if((l_wheel_status & STATUS_MASK2) == SWITCHED_ON){
			status = SWITCHED_ON;
		}
		else if((l_wheel_status & STATUS_MASK2) == OPERATION_ENABLED){
			status = OPERATION_ENABLED;
		}
		else if((l_wheel_status & STATUS_MASK2) == QUICK_STOP_ACTIVE){
			status = QUICK_STOP_ACTIVE;
		}
		else if((l_wheel_status & STATUS_MASK1) == FAULT_REACTION_ACTIVE){
			status = FAULT_REACTION_ACTIVE;
		}
		else if((l_wheel_status & STATUS_MASK1) == FAULT){
			status = FAULT;
		}
		// �^�C�}�[���Z�b�g
		unko_timer = 0;
	}
	else{
		// ���E���[�^�[�̃X�e�[�^�X���قȂ�܂܎��Ԃ��o�߂����瓯���G���[�Ƃ���
		if(unko_timer >= 500){
			status = SYNC_ERR;
		}
		else{
			unko_timer++;
		}
	}

	return status;
}

void dump_master_status(uint8_t status){
	// �f�o�b�O�p�Ɍ��݂̃X�e�[�^�X���o�͂���
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

void wheel_cntrl(int16_t left, int16_t right) {

	static uint8_t master_status = NOT_READY_TO_SWITCH_ON;
	// ���[�^�[�����M�����X�e�[�^�X���[�h�ɏ]����ԑJ�ڂ�����
	master_status = get_master_status();
	request_wheel_encoder(master_status);
	read_brake_mode();

	switch(master_status) {
	case NOT_READY_TO_SWITCH_ON:
		// ���[�^�[���N������̂�҂�
		break;

	case SWITCH_ON_DISABLE:
		if (ck_emerg_stop() != NOT_EMERGENCY) {
			//set_STO();
		}
		else{
			// READY_TO_SWITCH_ON�֑J�ڗv��
			set_control_word(SHUT_DOWN);
		}
		break;

	case READY_TO_SWITCH_ON:
		// �^�]���[�h�ƃp�����[�^�[��ݒ�
		if(ck_emerg_stop() != NOT_EMERGENCY){
			set_control_word(DISABLE_VOLTAGE);
			//set_STO();
		}
		else if (wheel_param_set() != 0 ) {
			// �ݒ芮��������SWITCHED_ON�֑J�ڗv��
			set_control_word(SWITCH_ON);
		}
		break;

	case SWITCHED_ON:
		if(ck_emerg_stop() != NOT_EMERGENCY){
			set_control_word(DISABLE_VOLTAGE);
			//set_STO();
		}
		else if (check_wheel_brake() == 0) {
			set_control_word(ENABLE_OPERATION);
		}
		break;

	case OPERATION_ENABLED:
		if(ck_emerg_stop() != NOT_EMERGENCY){
			set_control_word(DISABLE_VOLTAGE);
			//set_STO();
		}
		else if(check_wheel_brake() != 0){
			// ���J�u���[�L���������邽��SWITCHED_ON�֑J�ڗv��
			set_control_word(DISABLE_OPERATION);
		}
		else if((ck_l_wheel_brake_mode() == AUTO_BRAKE) && (ck_r_wheel_brake_mode() == AUTO_BRAKE)){
			// ���[�^�[��]
			wheel_set_speed(left, right);
		}
		else{
			wheel_set_speed(0, 0);
		}
		break;

	case QUICK_STOP_ACTIVE:
		// ���̃X�e�[�^�X�͒ʂ�Ȃ��͂�
		break;
	case FAULT_REACTION_ACTIVE:
		// ���[�^�[�h���C�o�[�G���[�Ƃ���
		// ���Z�b�gSW�������ꂽ��SWITCH_ON_DISABLE�ɑJ�ڂ�����\��
		break;
	case FAULT:
		// ���[�^�[�h���C�o�[�G���[�Ƃ���
		// ���Z�b�gSW�������ꂽ��SWITCH_ON_DISABLE�ɑJ�ڂ�����\��
		break;
	case SAFETY:
		if (ck_emerg_stop() == NOT_EMERGENCY) {
			// �G���[���������ꂽ��STO������ & SWITCH_ON_DISABLE�֑J�ڂ�����
			reset_STO();
			set_control_word(DISABLE_VOLTAGE);
		}
		else {
		}
		break;
	case SYNC_ERR:
		// ���[�^�[�h���C�o�[�G���[�Ƃ���
		// ���Z�b�gSW�������ꂽ��SWITCH_ON_DISABLE�ɑJ�ڂ�����\��
		break;
	}

	// ��~��������
	judge_wheel_stopped();
	// ���J�u���[�L�̐���
	if(master_status == OPERATION_ENABLED){
		// OPERATION_ENABLED�ł̓��J�u���[�L���������[�h�ɐݒ�
		if((ck_l_wheel_brake_mode() != AUTO_BRAKE) || (ck_r_wheel_brake_mode() != AUTO_BRAKE)){
			change_wheel_brake_mode(AUTO_BRAKE);
		}
	}
	else if((master_status != NOT_READY_TO_SWITCH_ON) && (ck_wheel_stopped() != 0)){
		// OPERATION_ENABLED�ȊO�Œ�~���̓��J�u���[�L���蓮�ɐݒ�
		change_wheel_brake_mode(MANUAL_BRAKE);
		if(check_wheel_brake() != 0){
			// ���J�u���[�L�����v��
			wheel_set_brake(BRAKE_OFF);
		}
		else{
			// ���J�u���[�L�쓮�v��
			wheel_set_brake(BRAKE_ON);
		}
	}

	// �f�o�b�O�p�ɃX�e�[�^�X��UART�ŏo��
	//dump_master_status(master_status);
}
