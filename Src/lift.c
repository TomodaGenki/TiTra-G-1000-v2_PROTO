/*
 * lift.c
 *
 *  Created on: 2019/05/14
 *      Author: ayabe
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "conf.h"
#include "lift.h"
#include "nuc.h"
#include "Cntrl_Proc.h"
#include "stdlib.h"
#include "common_func.h"

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart6;
/* Private define ------------------------------------------------------------*/
enum LiftSequence {
	Init,
	Ready,
	Wait,
	Keep,
	Decel,
	Stay,
	Ready_M,
	Wait_M,
	Keep_M,
	Emerge_Stay,
	Emerge_Ready,
	Emerge_Wait,
	Emerge_Keep
};

enum Write_Driver {
	Slave_Add,
	Func_Code,
	Resister_High,
	Resister_Low,
	Resister_Num_High,
	Resister_Num_Low,
	Byte_Num,
	Val_High,
	Val_Mdl_High,
	Val_Mdl_Low,
	Val_Low,
	Crc_High,
	Crc_Low,
	Write_Buff_Size
};

enum Initial_Setting {
	Drive_Param1,
	Drive_Param2,
	Drive_Param3,
	Drive_Param4,
	Drive_Param5,
	Drive_Param6,
	Drive_Param7,
	IO_Setting1,
	IO_Setting2,
	IO_Setting3,
	IO_Setting4,
	IO_Setting5,
	IO_Setting6,
	Configuration1,
	Configuration2,
	IO_Output,
	Setting_End
};

// define Lift motor communication timing
#define WAIT_WAKE_UP	10000
#define WAIT_TB3		15
#define WAIT_CONFIGURATION 1500

#define W_RESISTER_NUM	0x02
#define W_BYTE_NUM		0x04

// define resister address
#define ROT_SPEED2		0x0484
#define ROT_SPEED3		0x0486
#define ROT_SPEED4		0x0488
#define ROT_SPEED5		0x048A

#define ACC_TIME2		0x0604
#define ACC_TIME3		0x0606
#define ACC_TIME4		0x0608
#define ACC_TIME5		0x060A

#define DEC_TIME2		0x0684
#define DEC_TIME3		0x0686
#define DEC_TIME4		0x0688
#define DEC_TIME5		0x068A

#define REMOTE_IO		0x007C
#define ALARM_RST		0x0180
#define SPD_THRESH		0x114E
#define CONFIGRATION	0x018C

#define DIRECT_IO_X0	0x1100
#define DIRECT_IO_X1	0x1102
#define DIRECT_IO_X2	0x1104
#define DIRECT_IO_X3	0x1106
#define DIRECT_IO_X4	0x1108
#define DIRECT_IO_X5	0x110A
#define DIRECT_IO_Y0	0x1140
#define DIRECT_IO_Y1	0x1142
#define NET_IN0			0x1160
#define NET_IN1			0x1162
#define NET_IN2			0x1164
#define NET_IN3			0x1166
#define NET_IN4			0x1168
#define NET_IN5			0x116A
#define NET_IN6			0x116C
#define NET_IN7			0x116E
#define NET_IN8			0x1170
#define NET_IN9			0x1172
#define NET_IN10		0x1174
#define NET_IN11		0x1176

// define motor control parameter
//#define MAX_SPEED			0x00000BB8		// Lift motor max speed (3000rpm)
#define MAX_SPEED			0x000009C4		// Lift motor max speed (2500rpm)
#define MIN_SPEED			0x000003E8		// Lift motor min speed (1000rpm)
#define MANUAL_SPEED		0x000003E8		// Lift motor manual operation speed (1000rpm)
#define ACC_TIME			0x0000000A		// Lift motor acceleration time from stop state to max speed(10 ⇒ 1.0s)
#define DEC_TIME			0x0000000A		// Lift motor acceleration time from max speed to minimum speed(10 ⇒ 1.0s)
#define THRESH_VAL			0x00000001		// 目標回転数と現在回転数の許容誤差 (1rpm)

// define remote I/O command
#define MB_FREE_CMD			0x00000080
#define STOP_MODE_CMD		0x00000040
#define CONTROL_NO0			0x00000000
#define CONTROL_NO1			0x00000001
#define CONTROL_NO2			0x00000002
#define CONTROL_NO3			0x00000003
#define CONTROL_NO4			0x00000004
#define CONTROL_NO5			0x00000005
#define CONTROL_NO6			0x00000006
#define CONTROL_NO7			0x00000007
#define CONTROL_CHANGE		0xFFFFFFF8
#define PIN_NO_USE			0x00000000
#define EXCUTE_CONFIG		0x00000001
#define STOP_CONFIG			0x00000000

#define ALM_RST_CMD			0x00000001
#define ALM_RCV_CMD			0x00000000
/* Private variables ---------------------------------------------------------*/
static uint32_t remote_io_bit = 0;
static uint16_t	lift1_enc = 0;
static uint16_t	lift2_enc = 0;
static uint16_t	brakeWaitCntr = BRAKE_WAIT_TIM;
static uint8_t liftSeq = Init;
static uint8_t status = DOWN_STATE;
static uint8_t lift_sensor = 0;
static uint8_t lift_button1 = 0;
static uint8_t lift_button2 = 0;
static uint8_t lift_button1_old = 0;
static uint8_t lift_button2_old = 0;
static uint8_t lift_limit_state = 0;
static uint8_t lift_motor_alarm = 0;

/******************************************************************************/
/*           define GPIO control											  */
/******************************************************************************/
void lift1_direction_cw(void) {
	HAL_GPIO_WritePin(O_Lift1CCW_GPIO_Port, O_Lift1CCW_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(O_Lift1CW_GPIO_Port, O_Lift1CW_Pin, GPIO_PIN_RESET);
}

void lift1_direction_ccw(void) {
	HAL_GPIO_WritePin(O_Lift1CCW_GPIO_Port, O_Lift1CCW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(O_Lift1CW_GPIO_Port, O_Lift1CW_Pin, GPIO_PIN_SET);
}

void lift2_direction_cw(void) {
	HAL_GPIO_WritePin(O_Lift2CCW_GPIO_Port, O_Lift2CCW_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(O_Lift2CW_GPIO_Port, O_Lift2CW_Pin, GPIO_PIN_RESET);
}

void lift2_direction_ccw(void) {
	HAL_GPIO_WritePin(O_Lift2CCW_GPIO_Port, O_Lift2CCW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(O_Lift2CW_GPIO_Port, O_Lift2CW_Pin, GPIO_PIN_SET);
}

void lift_relay_on(void) {
	HAL_GPIO_WritePin(O_LiftRelay_GPIO_Port, O_LiftRelay_Pin, GPIO_PIN_SET);
}

void lift_relay_off(void) {
	HAL_GPIO_WritePin(O_LiftRelay_GPIO_Port, O_LiftRelay_Pin, GPIO_PIN_RESET);}

void lift1_motor_stop(void) {
	HAL_GPIO_WritePin(O_Lift1CCW_GPIO_Port, O_Lift1CCW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(O_Lift1CW_GPIO_Port, O_Lift1CW_Pin, GPIO_PIN_RESET);
}

void lift2_motor_stop(void) {
	HAL_GPIO_WritePin(O_Lift2CCW_GPIO_Port, O_Lift2CCW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(O_Lift2CW_GPIO_Port, O_Lift2CW_Pin, GPIO_PIN_RESET);
}

void lift_uart_trans(void) {
	HAL_GPIO_WritePin(O_USART6_TR_GPIO_Port, O_USART6_TR_Pin, GPIO_PIN_SET);
}

void lift_uart_reveive(void) {
	HAL_GPIO_WritePin(O_USART6_TR_GPIO_Port, O_USART6_TR_Pin, GPIO_PIN_RESET);
}

/******************************************************************************/
/*           Lift photo sensor function										  */
/******************************************************************************/
void set_lift_sens(void){

	static uint8_t lift_sensor_buff[2] = {0,0};

	lift_sensor_buff[1] = lift_sensor_buff[0];
	lift_sensor_buff[0] = 0;

	if (HAL_GPIO_ReadPin(I_Lift1UpSens_GPIO_Port, I_Lift1UpSens_Pin) == GPIO_PIN_RESET) {
		lift_sensor_buff[0] |= LIFT1_UP_SENSOR;
	}
	if (HAL_GPIO_ReadPin(I_Lift1DownSens_GPIO_Port, I_Lift1DownSens_Pin) == GPIO_PIN_RESET) {
		lift_sensor_buff[0] |= LIFT1_DOWN_SENSOR;
	}
	if (HAL_GPIO_ReadPin(I_Lift2UpSens_GPIO_Port, I_Lift2UpSens_Pin) == GPIO_PIN_RESET) {
		lift_sensor_buff[0] |= LIFT2_UP_SENSOR;
	}
	if (HAL_GPIO_ReadPin(I_Lift2DownSens_GPIO_Port, I_Lift2DownSens_Pin) == GPIO_PIN_RESET) {
		lift_sensor_buff[0] |= LIFT2_DOWN_SENSOR;
	}
	if (HAL_GPIO_ReadPin(I_Lift1UpLimitSens_GPIO_Port, I_Lift1UpLimitSens_Pin) == GPIO_PIN_RESET) {
//		lift_sensor_buff[0] |= LIFT1_UP_LIMIT_SENSOR;
	}
	if (HAL_GPIO_ReadPin(I_Lift1DownLimitSens_GPIO_Port, I_Lift1DownLimitSens_Pin) == GPIO_PIN_RESET) {
//		lift_sensor_buff[0] |= LIFT1_DOWN_LIMIT_SENSOR;
	}
	if (HAL_GPIO_ReadPin(I_Lift2UpLimitSens_GPIO_Port, I_Lift2UpLimitSens_Pin) == GPIO_PIN_RESET) {
//		lift_sensor_buff[0] |= LIFT2_UP_LIMIT_SENSOR;
	}
	if (HAL_GPIO_ReadPin(I_Lift2DownLimitSens_GPIO_Port, I_Lift2DownLimitSens_Pin) == GPIO_PIN_RESET) {
//		lift_sensor_buff[0] |= LIFT2_DOWN_LIMIT_SENSOR;
	}

	if (((lift_sensor_buff[0] & LIFT1_UP_SENSOR) == LIFT1_UP_SENSOR) && ((lift_sensor_buff[1] & LIFT1_UP_SENSOR) == LIFT1_UP_SENSOR)) {
		lift_sensor |= LIFT1_UP_SENSOR;
	} else if (((lift_sensor_buff[0] & LIFT1_UP_SENSOR) != LIFT1_UP_SENSOR) && ((lift_sensor_buff[1] & LIFT1_UP_SENSOR) != LIFT1_UP_SENSOR)) {
		lift_sensor &= ~LIFT1_UP_SENSOR;
	}

	if (((lift_sensor_buff[0] & LIFT1_DOWN_SENSOR) == LIFT1_DOWN_SENSOR) && ((lift_sensor_buff[1] & LIFT1_DOWN_SENSOR) == LIFT1_DOWN_SENSOR)) {
		lift_sensor |= LIFT1_DOWN_SENSOR;
	} else if (((lift_sensor_buff[0] & LIFT1_DOWN_SENSOR) != LIFT1_DOWN_SENSOR) && ((lift_sensor_buff[1] & LIFT1_DOWN_SENSOR) != LIFT1_DOWN_SENSOR)) {
		lift_sensor &= ~LIFT1_DOWN_SENSOR;
	}

	if (((lift_sensor_buff[0] & LIFT2_UP_SENSOR) == LIFT2_UP_SENSOR) && ((lift_sensor_buff[1] & LIFT2_UP_SENSOR) == LIFT2_UP_SENSOR)) {
		lift_sensor |= LIFT2_UP_SENSOR;
	} else if (((lift_sensor_buff[0] & LIFT2_UP_SENSOR) != LIFT2_UP_SENSOR) && ((lift_sensor_buff[1] & LIFT2_UP_SENSOR) != LIFT2_UP_SENSOR)) {
		lift_sensor &= ~LIFT2_UP_SENSOR;
	}

	if (((lift_sensor_buff[0] & LIFT2_DOWN_SENSOR) == LIFT2_DOWN_SENSOR) && ((lift_sensor_buff[1] & LIFT2_DOWN_SENSOR) == LIFT2_DOWN_SENSOR)) {
		lift_sensor |= LIFT2_DOWN_SENSOR;
	} else if (((lift_sensor_buff[0] & LIFT2_DOWN_SENSOR) != LIFT2_DOWN_SENSOR) && ((lift_sensor_buff[1] & LIFT2_DOWN_SENSOR) != LIFT2_DOWN_SENSOR)) {
		lift_sensor &= ~LIFT2_DOWN_SENSOR;
	}

	if (((lift_sensor_buff[0] & LIFT1_UP_LIMIT_SENSOR) == LIFT1_UP_LIMIT_SENSOR) && ((lift_sensor_buff[1] & LIFT1_UP_LIMIT_SENSOR) == LIFT1_UP_LIMIT_SENSOR)) {
		lift_sensor |= LIFT1_UP_LIMIT_SENSOR;
	} else if (((lift_sensor_buff[0] & LIFT1_UP_LIMIT_SENSOR) != LIFT1_UP_LIMIT_SENSOR) && ((lift_sensor_buff[1] & LIFT1_UP_LIMIT_SENSOR) != LIFT1_UP_LIMIT_SENSOR)) {
		lift_sensor &= ~LIFT1_UP_LIMIT_SENSOR;
	}

	if (((lift_sensor_buff[0] & LIFT1_DOWN_LIMIT_SENSOR) == LIFT1_DOWN_LIMIT_SENSOR) && ((lift_sensor_buff[1] & LIFT1_DOWN_LIMIT_SENSOR) == LIFT1_DOWN_LIMIT_SENSOR)) {
		lift_sensor |= LIFT1_DOWN_LIMIT_SENSOR;
	} else if (((lift_sensor_buff[0] & LIFT1_DOWN_LIMIT_SENSOR) != LIFT1_DOWN_LIMIT_SENSOR) && ((lift_sensor_buff[1] & LIFT1_DOWN_LIMIT_SENSOR) != LIFT1_DOWN_LIMIT_SENSOR)) {
		lift_sensor &= ~LIFT1_DOWN_LIMIT_SENSOR;
	}

	if (((lift_sensor_buff[0] & LIFT2_UP_LIMIT_SENSOR) == LIFT2_UP_LIMIT_SENSOR) && ((lift_sensor_buff[1] & LIFT2_UP_LIMIT_SENSOR) == LIFT2_UP_LIMIT_SENSOR)) {
		lift_sensor |= LIFT2_UP_LIMIT_SENSOR;
	} else if (((lift_sensor_buff[0] & LIFT2_UP_LIMIT_SENSOR) != LIFT2_UP_LIMIT_SENSOR) && ((lift_sensor_buff[1] & LIFT2_UP_LIMIT_SENSOR) != LIFT2_UP_LIMIT_SENSOR)) {
		lift_sensor &= ~LIFT2_UP_LIMIT_SENSOR;
	}

	if (((lift_sensor_buff[0] & LIFT2_DOWN_LIMIT_SENSOR) == LIFT2_DOWN_LIMIT_SENSOR) && ((lift_sensor_buff[1] & LIFT2_DOWN_LIMIT_SENSOR) == LIFT2_DOWN_LIMIT_SENSOR)) {
		lift_sensor |= LIFT2_DOWN_LIMIT_SENSOR;
	} else if (((lift_sensor_buff[0] & LIFT2_DOWN_LIMIT_SENSOR) != LIFT2_DOWN_LIMIT_SENSOR) && ((lift_sensor_buff[1] & LIFT2_DOWN_LIMIT_SENSOR) != LIFT2_DOWN_LIMIT_SENSOR)) {
		lift_sensor &= ~LIFT2_DOWN_LIMIT_SENSOR;
	}

	if ((((lift_sensor & LIFT1_UP_SENSOR) == LIFT1_UP_SENSOR) && ((lift_sensor & LIFT2_UP_SENSOR) == LIFT2_UP_SENSOR))
		|| ((lift_sensor & LIFT1_UP_LIMIT_SENSOR) == LIFT1_UP_LIMIT_SENSOR)
		|| ((lift_sensor & LIFT2_UP_LIMIT_SENSOR) == LIFT2_UP_LIMIT_SENSOR)) {
		set_LiftUpper_state();
	} else {
		reset_LiftUpper_state();
	}

	if ((((lift_sensor & LIFT1_DOWN_SENSOR) == LIFT1_DOWN_SENSOR) && ((lift_sensor & LIFT2_DOWN_SENSOR) == LIFT2_DOWN_SENSOR))
		|| ((lift_sensor & LIFT1_DOWN_LIMIT_SENSOR) == LIFT1_DOWN_LIMIT_SENSOR)
		|| ((lift_sensor & LIFT2_DOWN_LIMIT_SENSOR) == LIFT2_DOWN_LIMIT_SENSOR)) {
		set_LiftLower_state();
	} else {
		reset_LiftLower_state();
	}
}

uint8_t get_lift1_up_sensor(void) {
	return lift_sensor & LIFT1_UP_SENSOR;
}

uint8_t get_lift1_down_sensor(void) {
	return lift_sensor & LIFT1_DOWN_SENSOR;
}

uint8_t get_lift2_up_sensor(void) {
	return lift_sensor & LIFT2_UP_SENSOR;
}

uint8_t get_lift2_down_sensor(void) {
	return lift_sensor & LIFT2_DOWN_SENSOR;
}

uint8_t get_lift1_limit_sensor(void) {
	return lift_sensor & (LIFT1_UP_LIMIT_SENSOR + LIFT1_DOWN_LIMIT_SENSOR);
}

uint8_t get_lift1_up_limit_sensor(void) {
	return lift_sensor & LIFT1_UP_LIMIT_SENSOR;
}

uint8_t get_lift1_down_limit_sensor(void) {
	return lift_sensor & LIFT1_DOWN_LIMIT_SENSOR;
}

uint8_t get_lift2_limit_sensor(void) {
	return lift_sensor & (LIFT2_UP_LIMIT_SENSOR + LIFT2_DOWN_LIMIT_SENSOR);
}

uint8_t get_lift2_up_limit_sensor(void) {
	return lift_sensor & LIFT2_UP_LIMIT_SENSOR;
}

uint8_t get_lift2_down_limit_sensor(void) {
	return lift_sensor & LIFT2_DOWN_LIMIT_SENSOR;
}

void set_lift_upper_limit(void) {
	lift_limit_state |= LIFT_LIMIT_UPPER;
}

void reset_lift_upper_limit(void) {
	lift_limit_state &= ~LIFT_LIMIT_UPPER;
}

void set_lift_lower_limit(void) {
	lift_limit_state |= LIFT_LIMIT_LOWER;
}

void reset_lift_lower_limit(void) {
	lift_limit_state &= ~LIFT_LIMIT_LOWER;
}

uint8_t ck_lift_limit(void) {
	return lift_limit_state;
}

uint8_t ck_sensor_fail(void) {

	uint8_t ret = 0;

//	右リフトが上にあるにも関わらず、左リフトが下にある時
	if (((get_lift1_up_sensor() != 0) || (get_lift1_up_limit_sensor() != 0))
		&& ((get_lift2_down_sensor() != 0) || (get_lift2_down_limit_sensor() != 0))) {
		ret = 1;
	}

//	右リフトが下にあるにも関わらず、左リフトが上にある時
	if (((get_lift1_down_sensor() != 0) || (get_lift1_down_limit_sensor() != 0))
		&& ((get_lift2_up_sensor() != 0) || (get_lift2_up_limit_sensor() != 0))) {
		ret = 1;
	}

//	右リフトが上にも下にある時
	if (((get_lift1_up_sensor() != 0) || (get_lift1_up_limit_sensor() != 0))
		&& ((get_lift1_down_sensor() != 0) || (get_lift1_down_limit_sensor() != 0))) {
		ret = 1;
	}

//	左リフトが上にも下にある時
	if (((get_lift2_up_sensor() != 0) || (get_lift2_up_limit_sensor() != 0))
		&& ((get_lift2_down_sensor() != 0) || (get_lift2_down_limit_sensor() != 0))) {
		ret = 1;
	}
	return ret;
}

uint8_t ck_lift1_sensor(void) {

	uint8_t ret = 0;

	if (((ck_up_state() == UP_STATE) && ((get_lift1_up_sensor() != 0) || (get_lift1_up_limit_sensor() != 0)))
			|| ((ck_up_state() == DOWN_STATE) && ((get_lift1_down_sensor() != 0) || (get_lift1_down_limit_sensor() != 0)))) {
		ret = 1;
	}
	return ret;
}

uint8_t ck_lift2_sensor(void) {

	uint8_t ret = 0;

	if (((ck_up_state() == UP_STATE) && ((get_lift2_up_sensor() != 0) || (get_lift2_up_limit_sensor() != 0)))
			|| ((ck_up_state() == DOWN_STATE) && ((get_lift2_down_sensor() != 0) || (get_lift2_down_limit_sensor() != 0)))) {
		ret = 1;
	}
	return ret;
}

uint8_t ck_lift_sensor(void) {

	uint8_t ret = 0;

	if ((ck_lift1_sensor() != 0) && (ck_lift2_sensor() != 0)) {
		ret = 1;
	} else if (ck_lift1_sensor() != 0) {
		lift1_motor_stop();
	} else if (ck_lift2_sensor() != 0) {
		lift2_motor_stop();
	}
	return ret;
}

void judge_lift_limit(void) {

	if ((get_lift1_limit_sensor() == LIFT1_UP_LIMIT_SENSOR) || (get_lift2_limit_sensor() == LIFT2_UP_LIMIT_SENSOR)) {
		set_lift_upper_limit();
	}

	if ((get_lift1_limit_sensor() == LIFT1_DOWN_LIMIT_SENSOR) || (get_lift2_limit_sensor() == LIFT2_DOWN_LIMIT_SENSOR)) {
		set_lift_lower_limit();
	}
}

/******************************************************************************/
/*           Lift motor alarm control function								  */
/******************************************************************************/
void scan_liftmotor_alarm(void) {

	static uint8_t lift1_alarm_tmp = 0;
	static uint8_t lift2_alarm_tmp = 0;

	lift1_alarm_tmp = lift1_alarm_tmp << 1;
	lift2_alarm_tmp = lift2_alarm_tmp << 1;

	if (HAL_GPIO_ReadPin(I_Lift1Alm_GPIO_Port, I_Lift1Alm_Pin) == GPIO_PIN_SET) {
//		lift1_alarm_tmp |= BIT00;
	}
	if (HAL_GPIO_ReadPin(I_Lift2Alm_GPIO_Port, I_Lift2Alm_Pin) == GPIO_PIN_SET) {
//		lift2_alarm_tmp |= BIT00;
	}

	if (lift1_alarm_tmp == 0xFF) {
		lift_motor_alarm |= LIFT_MOTOR1_ALARM;
	} else if (lift1_alarm_tmp == 0x00) {
		lift_motor_alarm &= (~LIFT_MOTOR1_ALARM);
	}

	if (lift2_alarm_tmp == 0xFF) {
		lift_motor_alarm |= LIFT_MOTOR2_ALARM;
	} else if (lift2_alarm_tmp == 0x00) {
		lift_motor_alarm &= (~LIFT_MOTOR2_ALARM);
	}
}

uint8_t ck_lift_motor1_alarm(void) {
	return lift_motor_alarm & LIFT_MOTOR1_ALARM;
}

uint8_t ck_lift_motor2_alarm(void) {
	return lift_motor_alarm & LIFT_MOTOR2_ALARM;
}

/******************************************************************************/
/*           Lift encoder control function									  */
/******************************************************************************/
void add_lift1_enc(void) {
	lift1_enc++;
}

void add_lift2_enc(void) {
	lift2_enc++;
}

void clr_lift_enc(void) {
	lift1_enc = 0;
	lift2_enc = 0;
}

uint16_t get_lift1_enc(void) {
	return lift1_enc;
}

uint16_t get_lift2_enc(void) {
	return lift2_enc;
}

uint16_t max_diff_debug = 0;

uint8_t ck_lift_position_diff(void) {

	uint16_t diff;
	uint8_t ret = 0;

	diff = abs(lift1_enc - lift2_enc);

	if (max_diff_debug > diff) {
		max_diff_debug = diff;
	}

	if (diff > LIFT_DIFF_THRESH) {
		ret = 1;
	}
	return ret;
}

/******************************************************************************/
/*           Lift direction control function								  */
/******************************************************************************/
void set_lift_state(uint8_t st) {
	status = st;
}

void set_lift_stat(uint8_t ctrl1) {
	if((ctrl1 & UP_COMMAND) != UP_COMMAND) {
		set_lift_state(DOWN_STATE);								// set down status
	} else {
		set_lift_state(UP_STATE);								// set up status
	}
}

void set_lift_stat_button(void) {
	if ((get_lift1_down_limit_sensor() != 0) || (get_lift2_down_limit_sensor() != 0)) {
		set_lift_state(UP_STATE);
	} else if ((get_lift1_up_limit_sensor() != 0) || (get_lift2_up_limit_sensor() != 0)) {
		set_lift_state(DOWN_STATE);
	} else if ((get_lift1_down_sensor() != 0) || (get_lift2_down_sensor() != 0)) {
		set_lift_state(UP_STATE);
	} else {
		set_lift_state(DOWN_STATE);
	}
}

uint8_t ck_up_state(void){
	return (status & UP_STATE);
}

void set_motor_direction(void) {
	if (ck_up_state() != 0) {
		// up state : CW
		lift1_direction_cw();
		lift2_direction_cw();
	} else {
		// down state : CCW
		lift1_direction_ccw();
		lift2_direction_ccw();
	}
}

/******************************************************************************/
/*           Lift control function											  */
/******************************************************************************/
void set_wait_brakelease_time(void) {
	brakeWaitCntr = BRAKE_WAIT_TIM;
}

uint8_t wait_brake_release(void){

	uint8_t ret = 0;

	if (brakeWaitCntr == 0) {
		ret = 1;
	} else {
		brakeWaitCntr--;
	}
	return ret;
}

void set_button_state(void) {

	lift_button1_old = lift_button1;
	lift_button2_old = lift_button2;

	if(ck_lift_updown1_key() != 0) { // push button1
		lift_button1 = 1;
	} else if (ck_lift_updown2_key() != 0) { // push button2
		lift_button2 = 1;
	} else {
		lift_button1 = 0;
		lift_button2 = 0;
	}
}

uint8_t ck_lift_button_push(void) {

	uint8_t ret = 0;

	if (((lift_button1_old == 0) && (lift_button1 == 1)) || ((lift_button2_old == 0) && (lift_button2 == 1))) {
		if ((ck_RxDrvSpeed() == 0) && (get_TurnTable_Speed() == 0)) {
			ret = 1;
		}
	}
	return ret;
}

uint8_t ck_lift_button_release(void) {

	uint8_t ret = 0;

	if (((lift_button1_old == 1) && (lift_button1 == 0)) || ((lift_button2_old == 1) && (lift_button2 == 0))) {
		ret = 1;
	}
	return ret;
}

/******************************************************************************/
/*           lift motor UART control									      */
/******************************************************************************/
// 2つのレジスタ(上位・下位)に対して書き込みを行うためのインターフェース
// slave_add : 書き込みを行うスレーブアドレス(0:両方、1:右モーター、2:左モーター)
// resister : 書き込みの起点となるレジスタアドレス
// control : 書き込む値
void write_mtrdriver_data(uint8_t slave_add, uint16_t resister, uint32_t control) {

	static uint8_t tx_buff[Write_Buff_Size];
	union LongByte tx_cnv;
	uint16_t crc;

	// set trans data
	tx_buff[Slave_Add] = slave_add;					// set slave address
	tx_buff[Func_Code] = WRITE_MULTI_MTR_VALS;		// set function code (0x10 : multiple write)

	tx_cnv.w_val[0] = resister;
	tx_buff[Resister_High] = tx_cnv.b_val[1];		// set resister address high byte
	tx_buff[Resister_Low] = tx_cnv.b_val[0];		// set resister address low byte

	tx_cnv.w_val[0] = W_RESISTER_NUM;
	tx_buff[Resister_Num_High] = tx_cnv.b_val[1];	// set number of resister for write high byte
	tx_buff[Resister_Num_Low] = tx_cnv.b_val[0];	// set number of resister for write low byte

	tx_buff[Byte_Num] = W_BYTE_NUM;

	tx_cnv.l_val = control;
	tx_buff[Val_High] = tx_cnv.b_val[3];		// set drive input high byte
	tx_buff[Val_Mdl_High] = tx_cnv.b_val[2];	// set drive input middle-high byte
	tx_buff[Val_Mdl_Low] = tx_cnv.b_val[1];		// set drive input middle-low byte
	tx_buff[Val_Low] = tx_cnv.b_val[0];			// set drive input low byte

	crc = crc16_calc(tx_buff, sizeof(tx_buff));

	tx_cnv.w_val[0] = crc;
	tx_buff[Crc_High] = tx_cnv.b_val[0];			// set error check high byte
	tx_buff[Crc_Low] = tx_cnv.b_val[1];			// set error check low byte

	lift_uart_trans();

	HAL_UART_Transmit_DMA(&huart6, tx_buff, sizeof(tx_buff));
}

void select_lift_stop_mode(uint8_t mode) {
	if (mode == Deceleration) {
		remote_io_bit |= STOP_MODE_CMD;
	} else {
		remote_io_bit &= ~STOP_MODE_CMD;
	}
}

void select_lift_brake_mode(uint8_t mode) {
	if (mode == Free) {
		remote_io_bit |= MB_FREE_CMD;
	} else {
		remote_io_bit &= ~MB_FREE_CMD;
	}
}

void set_control_no(uint32_t data) {
	remote_io_bit &= CONTROL_CHANGE;
	remote_io_bit |= data;
}

void lift_alarm_reset(void) {
	lift1_motor_stop();
	lift2_motor_stop();
	write_mtrdriver_data(BROAD_CAST, ALARM_RST, ALM_RST_CMD);
}

void lift_alarm_recover(void) {
	write_mtrdriver_data(BROAD_CAST, ALARM_RST, ALM_RCV_CMD);
}

void lift1_alarm_reset(void) {
	lift1_motor_stop();
	write_mtrdriver_data(SLAVE_ADD1, ALARM_RST, ALM_RST_CMD);
}

void lift1_alarm_recover(void) {
	write_mtrdriver_data(SLAVE_ADD1, ALARM_RST, ALM_RCV_CMD);
}

void lift2_alarm_reset(void) {
	lift2_motor_stop();
	write_mtrdriver_data(SLAVE_ADD2, ALARM_RST, ALM_RST_CMD);
}

void lift2_alarm_recover(void) {
	write_mtrdriver_data(SLAVE_ADD2, ALARM_RST, ALM_RCV_CMD);
}

/******************************************************************************/
/*           initialize													      */
/******************************************************************************/
void lift_init(void) {
	select_lift_stop_mode(Immediate);
	select_lift_brake_mode(Lock);
	set_control_no(CONTROL_NO2);
	set_utimer(CNT_LIFT_WAIT, WAIT_WAKE_UP);
}

uint8_t lift_parameter_set(void) {

	static uint8_t setting_state = Drive_Param1;
	uint8_t ret = 0;

	// モーター制御のための初期値をセットする
	switch(setting_state) {
	case Drive_Param1:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 運転データNo.3 に最高速(3000rpm)を設定
			write_mtrdriver_data(BROAD_CAST, ROT_SPEED3, MAX_SPEED);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = Drive_Param2;
		}
		break;
	case Drive_Param2:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 運転データNo.3 に加速時間を設定
			write_mtrdriver_data(BROAD_CAST, ACC_TIME3, ACC_TIME);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = Drive_Param3;
		}
		break;
	case Drive_Param3:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 運転データNo.4 に減速時の最低速度(1000rpm)を設定
			write_mtrdriver_data(BROAD_CAST, ROT_SPEED4, MIN_SPEED);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = Drive_Param4;
		}
		break;
	case Drive_Param4:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 運転データNo.4 に減速時間を設定
			write_mtrdriver_data(BROAD_CAST, DEC_TIME4, DEC_TIME);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = Drive_Param5;
		}
		break;
	case Drive_Param5:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 運転データNo.5 に手動運転時の速度(1000rpm)を設定
			write_mtrdriver_data(BROAD_CAST, ROT_SPEED5, MANUAL_SPEED);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = Drive_Param6;
		}
		break;
	case Drive_Param6:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 運転データNo.5 に手動運転時の加速時間を設定
			write_mtrdriver_data(BROAD_CAST, ACC_TIME5, ACC_TIME);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = Drive_Param7;
		}
		break;
	case Drive_Param7:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 回転数の許容誤差を1rpmに設定
			write_mtrdriver_data(BROAD_CAST, SPD_THRESH, THRESH_VAL);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = IO_Setting1;
		}
		break;
	case IO_Setting1:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 使用しないダイレクトI/Oを0(未使用)に設定:X2(STOP-MODE)
			write_mtrdriver_data(BROAD_CAST, DIRECT_IO_X2, PIN_NO_USE);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = IO_Setting2;
		}
		break;
	case IO_Setting2:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 使用しないダイレクトI/Oを0(未使用)に設定:X3(M0)
			write_mtrdriver_data(BROAD_CAST, DIRECT_IO_X3, PIN_NO_USE);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = IO_Setting3;
		}
		break;
	case IO_Setting3:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 使用しないダイレクトI/Oを0(未使用)に設定:X4(ALARM-RESET)
			write_mtrdriver_data(BROAD_CAST, DIRECT_IO_X4, PIN_NO_USE);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = IO_Setting4;
		}
		break;
	case IO_Setting4:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 使用しないダイレクトI/Oを0(未使用)に設定:X5(MB-FREE)
			write_mtrdriver_data(BROAD_CAST, DIRECT_IO_X5, PIN_NO_USE);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = IO_Setting5;
		}
		break;
	case IO_Setting5:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 使用しないリモートI/Oを0(未使用)に設定:NET-IN3(FWD)
			write_mtrdriver_data(BROAD_CAST, NET_IN3, PIN_NO_USE);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = IO_Setting6;
		}
		break;
	case IO_Setting6:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 使用しないリモートI/Oを0(未使用)に設定:NET-IN4(REV)
			write_mtrdriver_data(BROAD_CAST, NET_IN4, PIN_NO_USE);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = Configuration1;
		}
		break;
	case Configuration1:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// Configuration を実行
			write_mtrdriver_data(BROAD_CAST, CONFIGRATION, PIN_NO_USE);
			set_utimer(CNT_LIFT_WAIT, WAIT_CONFIGURATION);
			setting_state = Configuration2;
		}
		break;
	case Configuration2:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// Configuration の実行を待つ
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = IO_Output;
		}
		break;
	case IO_Output:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// 最後にNet-I/Oを設定する
			write_mtrdriver_data(BROAD_CAST, REMOTE_IO, remote_io_bit);
			set_utimer(CNT_LIFT_WAIT, WAIT_TB3);
			setting_state = Setting_End;
		}
		break;
	case Setting_End:
		if (check_timer(CNT_LIFT_WAIT) == TIME_UP) {
			// ここまで来れば設定終了
			ret = 1;
		}
	}
	return ret;
}

/******************************************************************************/
/*           lift state control											      */
/******************************************************************************/
void lift_cntrl(void) {
	
	static uint8_t lift_emerge_old = NOT_EMERGENCY;
	static uint8_t lift_cmd_old = 0;
	uint8_t lift_cmd = check_lift_order();
	uint8_t lift_emerge = ck_emerg_stop();

	set_button_state();	// button push?

	if (liftSeq != Init) {
		if (((lift_emerge != NOT_EMERGENCY) && (lift_emerge_old == NOT_EMERGENCY))
				|| ((ck_lift_motor1_alarm() != 0) || (ck_lift_motor2_alarm() != 0))) {
			liftSeq = Emerge_Stay;
		}
		lift_emerge_old = lift_emerge;
	}

	switch(liftSeq) {
	case Ready:
		if ((ck_lift_sensor() != 0) || (ck_sensor_fail() != 0)) {
			liftSeq = Stay;
		} else {
			set_motor_direction();
			set_wait_brakelease_time();
			set_LiftActive_state();
			liftSeq = Wait;
		}
		break;

	case Wait:		// waiting for brake off, then start
		if (wait_brake_release() != 0) {
			liftSeq = Keep;
			// UARTで自動運転時のモードを指定
			set_control_no(CONTROL_NO3);
			write_mtrdriver_data(BROAD_CAST, REMOTE_IO, remote_io_bit);
		}
		break;

	case Keep:		// keep max speed
		if ((ck_lift_sensor() != 0) || (ck_lift_position_diff() != 0)) {
			liftSeq = Stay;
			set_control_no(CONTROL_NO2);
			write_mtrdriver_data(BROAD_CAST, REMOTE_IO, remote_io_bit);
		} else if ((get_lift1_enc() > LIFT_DESEL_POINT) || (get_lift2_enc() > LIFT_DESEL_POINT)) {
			liftSeq = Decel;
			// UARTで減速運転時のモードを指定
			set_control_no(CONTROL_NO4);
			write_mtrdriver_data(BROAD_CAST, REMOTE_IO, remote_io_bit);
		}
		break;

	case Decel:
		if (ck_lift_sensor() != 0) {
			liftSeq = Stay;
			set_control_no(CONTROL_NO2);
			write_mtrdriver_data(BROAD_CAST, REMOTE_IO, remote_io_bit);
		}
		break;

	case Stay:
		if (ck_motor_active() == 0) {
			liftSeq = Init;
		}
		clr_lift_enc();
		lift1_motor_stop();
		lift2_motor_stop();
		set_LiftComplete_state();
		if (lift_cmd != lift_cmd_old) {
			lift_cmd_old = lift_cmd;
			if ((lift_cmd & LIFT_ACT) == LIFT_ACT) {
				liftSeq = Ready;
				set_lift_stat(lift_cmd);
			}
		} else if (ck_lift_button_push() != 0) {
			liftSeq = Ready_M;
			set_lift_stat_button();
		}
		break;

	case Ready_M:	//button control start
		if ((ck_lift_sensor() != 0) || (ck_sensor_fail() != 0)) {
			liftSeq = Stay;
		} else {
			set_motor_direction();
			set_wait_brakelease_time();
			set_LiftActive_state();
			liftSeq = Wait_M;
		}
		break;

	case Wait_M:	// waiting for brake off, then start
		if ((ck_lift_button_release() != 0) || (ck_lift_sensor() != 0)) {
			liftSeq = Stay;
		} else if (wait_brake_release() != 0) {							// wait for break-off
			liftSeq = Keep_M;
			// UARTで手動運転時のモードを指定
			set_control_no(CONTROL_NO5);
			write_mtrdriver_data(BROAD_CAST, REMOTE_IO, remote_io_bit);
		}
		break;

	case Keep_M:	// keep max speed
		if ((ck_lift_button_release() != 0) || (ck_lift_sensor() != 0) || (ck_lift_position_diff() != 0)){
			liftSeq = Stay;
			set_control_no(CONTROL_NO2);
			write_mtrdriver_data(BROAD_CAST, REMOTE_IO, remote_io_bit);
		}
		break;

	case Emerge_Stay:
		clr_lift_enc();
		lift1_motor_stop();
		lift2_motor_stop();
		set_LiftComplete_state();
		if (lift_emerge == NOT_EMERGENCY) {
			liftSeq = Stay;
		} else if (ck_lift_button_push() != 0) {
			liftSeq = Emerge_Ready;
			set_lift_stat_button();
		}
		break;

	case Emerge_Ready:
		if ((ck_lift_sensor() != 0) || (ck_sensor_fail() != 0)) {
			liftSeq = Emerge_Stay;
		} else {
			set_motor_direction();
			set_wait_brakelease_time();
			liftSeq = Emerge_Wait;
		}
		break;

	case Emerge_Wait:
		if ((ck_lift_button_release() != 0) || (ck_lift_sensor() != 0)) {
			liftSeq = Emerge_Stay;
		} else if (wait_brake_release() != 0) {							// wait for break-off
			liftSeq = Emerge_Keep;
			// UARTで手動運転時のモードを指定
			set_control_no(CONTROL_NO5);
			write_mtrdriver_data(BROAD_CAST, REMOTE_IO, remote_io_bit);
		}
		break;

	case Emerge_Keep:
		if ((ck_lift_button_release() != 0) || (ck_lift_sensor() != 0) || (ck_lift_position_diff() != 0)) {
			liftSeq = Emerge_Stay;
			set_control_no(CONTROL_NO2);
			write_mtrdriver_data(BROAD_CAST, REMOTE_IO, remote_io_bit);
		}
		break;

	case Init:
 		if (ck_motor_active() != 0) {
 			if (lift_parameter_set() != 0) {
 				liftSeq = Stay;
 			}
 		}
		break;

	default:
		set_LiftComplete_state();				// set lift completed flag
		liftSeq = Stay;
		break;
	}
}
