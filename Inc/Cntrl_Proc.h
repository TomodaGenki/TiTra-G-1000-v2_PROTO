/*
 * Cntrl_Proc.h
 *
 *  Created on: 2019/07/05
 *      Author: Takumi
 */
#ifndef CNTRL_H_
#define CNTRL_H_

// ----------- extern ------------
void cmnd_proc(void);
void scan_proc1(void);
void read_HardwareID(void);
void scan_bumper(void);
void emerge_key_scan(void);
uint8_t ck_emerg_stop(void);
void HAL_UART_ResponseData(uint8_t);

void time_run(void);
void set_utimer(uint8_t t, uint32_t tim);
void init_timer(void);
uint8_t check_timer(uint8_t t);
uint8_t ck_lift_updown1_key(void);
uint8_t ck_lift_updown2_key(void);
uint8_t ck_alarm_release1_key(void);
uint8_t ck_alarm_release2_key(void);
uint8_t ck_brake_release1_key(void);
uint8_t ck_brake_release2_key(void);
uint8_t ck_turn_brake1_key(void);
uint8_t ck_turn_brake2_key(void);
uint8_t get_sensor1_state(void);
uint8_t get_sensor2_state(void);
uint8_t send_mt_alarm(void);
uint8_t get_mcu_state(void);

HAL_StatusTypeDef read_battery_status(uint8_t batt_add, uint8_t cmmnd, uint8_t *pD);
void ck_sensor1_state(void);
void ck_mcu_state(void);
void set_LiftActive_state(void);
void set_LiftComplete_state(void);
void set_LiftUpper_state(void);
void reset_LiftUpper_state(void);
void set_LiftLower_state(void);
void reset_LiftLower_state(void);
void set_TurnAngle0_state(void);
void reset_TurnAngle0_state(void);
void set_TurnAngle90_state(void);
void reset_TurnAngle90_state(void);
void set_TurnAngle180_state(void);
void reset_TurnAngle180_state(void);
void set_TurnAngle270_state(void);
void reset_TurnAngle270_state(void);
void set_TurnActive_state(void);
void set_TurnComplete_state(void);
void power_off(void);
uint8_t get_HardwareID_High(void);
uint8_t get_HardwareID_Low(void);
void recovery_emergency(void);
uint8_t ck_mt_alarm(void);
uint8_t check_bumper(void);
uint8_t ck_emgncy_key(void);
uint8_t check_wheel_brake(void);
uint8_t check_turn_brake(void);
uint8_t get_turn_sensor_pos(void);
uint8_t get_lift_pos(void);
void set_bumper1_flag(void);
void reset_bumper1_flag(void);
uint8_t get_bumper1_flag(void);
void set_bumper2_flag(void);
void reset_bumper2_flag(void);
uint8_t get_bumper2_flag(void);
void set_emergekey_flag(void);
void reset_emergekey_flag(void);
uint8_t get_emergekey_flag(void);
void ck_emerge_sens(void);
void set_wake_up_flag(void);
void reset_wake_up_flag(void);
uint8_t get_wake_up_flag(void);
void set_motor_active(void);
void reset_motor_active(void);
uint8_t ck_motor_active(void);
void motor_wakeup_cntrl(void);
void monitor_emerg_stop(void);
void set_emergency(uint8_t emergency);
void set_command_data(uint8_t data, uint8_t i);
void set_cmd_process_flag(void);
void reset_cmd_process_flag(void);
uint8_t get_cmd_process_flag(void);

//--- timer ---
enum cntrl_timer{
	CNT_GO1	= 0,
	CNT_GO2,
	CNT_GO3,
	CNT_GO4,
	CNT_GO5,
	CNT_GO6,
	CNT_GO7,
	CDT_CHRGROUT,
	CNT_PROCESS,
	CNT_IRDACOMM,
	CNT_TURN,								// Turn table
	CNT_SYNCTURN,							// Synchronous turn
	CNT_LIFT_WAIT,							// リフトモーターへの通信待ち時間
	CNT_WHEEL_WAIT,							// 動輪へのCAN送信待ち時間
	CNT_MOTOR_WAKEUP,
	CNT_WHEEL_ENCODER,
	CNT_LAST
};

#define		NO_TIMER	CNT_LAST			// No. of user timer
#define		TIME_UP		1					// timer expires
#define		TIME_NOT_UP	0					// timer is not running or not expired

#define	CNT_PERIOD1		20					// 20msec
#define	CNT_PERIOD2		100					// 100msec
#define	CNT_PERIOD3		300					// 300msec
#define CNT_PERIOD4 	20000				// 20000msec
#define CNT_PERIOD5 	500					// 500msec
#define CNT_PERIOD6 	60000				// 60000msec
#define CNT_CHRGROUT	20000				// 20sec
#define CNT_IRDADOWN	5000				// 5sec
#define	CNT_NUCCOM		100					// 100msec
#define CNT_PROCPERIOD	10					// 10msec process execution
#define	CNT_FIRMTIME	5000				// submit firmware version after power on
#define CNT_TURNPERIOD	1					// Turn table
#define CNT_SYNCTURN_VAL	10
#define MOTOR_WAKEUP_START	5000
#define MOTOR_WAKEUP_TIMER	3000
#define ONE_SHOT_TIMER		100
#define RESET_WAIT_TIMER	10000
#define WHEEL_MOTOR_WAKE_UP		10000
#define WHEEL_CAN_WAIT_TIME		50
#define CNT_1MSEC		1					// 1msec

#define	BIT07				0x80
#define	BIT06				0x40
#define	BIT05				0x20
#define	BIT04				0x10
#define	BIT03				0x08
#define	BIT02				0x04
#define	BIT01				0x02
#define	BIT00				0x01

// ---
#define	B_STATUS_SIZE		0x02			// battery status size(2byte+CRC)

// ---
enum emergency_level{
	NOT_EMERGENCY = 0,
	EMERGENCY_CT0,
	EMERGENCY_CT1,
	EMERGENCY_CT2,
	EMERGENCY_CT3,							// Takumi original category(not stop to run)
	EMERGENCY_CT4,							// Takumi original category(lift limit sensor on)
};

#define	RESET_ALARM_MANUAL	0x01
#define	RESET_ALARM_AUTO	0x02

#define RELEASE_WHEEL_ERROR		0x01
#define RELEASE_TURN_ERROR		0x02
#define RELEASE_LIFT_ERROR		0x04
#define RELEASE_LIFT_LIMIT		0x08
#define RELEASE_EMERGENCY_KEY	0x10
#define RELEASE_COMMERROR		0x20
#define RELEASE_BUMPER			0x40
#define RELEASE_LIDAR			0x80

#define RELEASE_END				0x00
#define RELEASE_WAIT_TIME		100		// 200msec
// --- debug ---
#define	COMSIZE		18						// Tx Buffer size

#endif
