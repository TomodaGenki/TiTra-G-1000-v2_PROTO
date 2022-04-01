/*
 * wheel.h
 *
 *  Created on: 2021/03/28
 *      Author: Takumi
 */

#ifndef WHEEL_H_
#define WHEEL_H_

/* function ----------------------------------------------------------*/
void wheel_cntrl(int16_t left, int16_t right);
void wheel_set_speed(int16_t l, int16_t r);
void reset_alarm(void);
void reset_motor1_alarm(void);
void reset_motor2_alarm(void);
void recover_alarm(void);
void recover_motor1_alarm(void);
void recover_motor2_alarm(void);
void reset_motor_alarm_auto(void);
void wheel_init(void);
uint8_t ck_wheel1_motor_alarm(void);
uint8_t ck_wheel2_motor_alarm(void);
void scan_wheelmotor_alarm(void);
void wheel_relay_off(void);
void wheel_relay_on(void);
void add_relay_off(void);
void add_relay_on(void);
void receive_wheel_motor_data(uint8_t *receive_data, uint8_t l_r);
void receive_wheel_motor_error_data(uint8_t *receive_data, uint8_t l_r);
uint32_t get_l_wheel_encoder(void);
uint32_t get_r_wheel_encoder(void);
void monitor_wheel_encoder(void);

/* define ------------------------------------------------------------*/
// CAN ID ‚Ì’è‹`
#define L_WHEEL_ID		0x001
#define R_WHEEL_ID		0x003
#define SDO_TX_ID		0x600
#define SDO_RX_ID		0x580
#define EMCY_ID			0x080

typedef struct whl_mtr_data {
	uint16_t	whl_status;
	uint8_t		brake_mode;
	uint8_t		brake_status;
	uint32_t	whl_encoder;
	uint16_t	whl_error;

} WHL_MTR_DATA;


#endif /* WHEEL_H_ */
