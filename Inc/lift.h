/*
 * lift.h
 *
 *  Created on: 2019/05/31
 *      Author: ayabe
 */

#ifndef LIFT_H_
#define LIFT_H_

// ---------------------------

// define lift sensor
#define LIFT1_UP_SENSOR		0x01
#define LIFT1_DOWN_SENSOR	0x02
#define LIFT2_UP_SENSOR		0x04
#define LIFT2_DOWN_SENSOR	0x08
#define LIFT1_UP_LIMIT_SENSOR	0x10
#define LIFT1_DOWN_LIMIT_SENSOR	0x20
#define LIFT2_UP_LIMIT_SENSOR	0x40
#define LIFT2_DOWN_LIMIT_SENSOR	0x80

#define LIFT_LIMIT_LOWER	0x01
#define LIFT_LIMIT_UPPER	0x02

#define	DOWN_STATE			0x00
#define	UP_STATE			0xFF

#define LIFT_MOTOR1_ALARM	0x01
#define LIFT_MOTOR2_ALARM	0x02

// 左右エンコーダー差分の許容量
// エンコーダーの差分がこの量を超えるとリフト動作を停止する
#define LIFT_DIFF_THRESH	50	//
//#define LIFT_DIFF_THRESH	5	// 左右の4出力軸の差が4度以上になると止める

// define motor control parameter
// Wait time for lift brake release  unit : msec
#define BRAKE_WAIT_TIM		0

// slope down point
#define LIFT_DESEL_POINT		4210
#define LIFT_OVER_POINT

#define READ_MTR_VAL			0x03
#define WRITE_MTR_VAL			0x06
#define WRITE_MULTI_MTR_VALS	0x10
#define READ_WRITE_MTR_VAL		0x17

// define slave address
#define BROAD_CAST	0x00
#define SLAVE_ADD1	0x01
#define SLAVE_ADD2	0x02

enum StopMode {
	Deceleration,
	Immediate
};

enum MbMode {
	Free,
	Lock
};

void lift_cntrl(void);
uint8_t ck_up_state(void);
void set_lift_sens(void);
void lift_init(void);
void lift_relay_on(void);
void lift_relay_off(void);
void add_lift1_enc(void);
void add_lift2_enc(void);
void lift1_alarm_reset(void);
void lift1_alarm_recover(void);
void lift2_alarm_reset(void);
void lift2_alarm_recover(void);
uint8_t get_lift1_limit_sensor(void);
uint8_t get_lift2_limit_sensor(void);
void set_lift_upper_limit(void);
void reset_lift_upper_limit(void);
void set_lift_lower_limit(void);
void reset_lift_lower_limit(void);
uint8_t ck_lift_limit(void);
void judge_lift_limit(void);
void scan_liftmotor_alarm(void);
uint8_t ck_lift_motor1_alarm(void);
uint8_t ck_lift_motor2_alarm(void);
uint8_t ck_lift_position_diff(void);
void lift_uart_trans(void);
void lift_uart_reveive(void);
void lift_alarm_reset(void);
void lift_alarm_recover(void);
#endif /* LIFT_H_ */
