/*
 * nuc.h
 *
 *  Created on: 2019/05/31
 *      Author: ayabe
 */

#ifndef NUC_H_
#define NUC_H_

// -- packet--
//#define RxBufSize 13
#define RxBufSize 	14
#define RxBufNo		8
#define	RxBufMask	0x07
#define TxBufSize 	24					// 16->24
#define AllSesponce 0x11

// --- RX ---
/* USER CODE BEGIN Private defines */
// Header/Footer
#define	RX_HEADER	0x02
#define	RX_LENGTH	0x0B
#define	RX_FOOTER	0x03
#define	TX_HEADER	0x02
#define	TX_FOOTER	0x03


// command
#define NUC_CMND	0x01
#define	NUC_CMND2	0x02
#define	NUC_CMND3	0x03
#define	NUC_R_ID	0x04				// read ID of micon board from eeprom
#define	NUC_W_ID	0x05				// write ID of micon board to eeprom
#define	NUC_R_EP	0x06				// read data from eeprom
#define	NUC_W_EP	0x07				// write data to eeprom
#define	NUC_R_BATT_ST	0x08			// read status of battery
#define NUC_R_BATT_ADD	0x0A
#define	NUC_R_BATT_LBL	0x0B			//Å@read battery label
#define	NUC_W_BATT_LBL	0x0C			//Å@write battery label
#define NUC_W_EMRGNCY_RESET	0xAA		// reset emergency status
#define NUC_R_FW_VER	0xBB			// get firmware version
#define NUC_MIN_CMD NUC_CMND			// min command no.
#define NUC_MAX_CMD NUC_CMND3			// max command no.
#define SYNCTURN_LOG_CMD	0x21		// sync turn log output command
#define TURN_LOG_CMD		0x22		// turn table log output command
#define WHEEL_LOG_CMD		0x23		// wheel log output command
#define LIFT_LOAD_LOG_CMD		0x24
#define TURN_LOAD_LOG_CMD		0x25
#define WHEEL_TEST_LOG_CMD		0x26

#define	NUC_R_TEST	0xf0
#define	NUC_W_TEST	0xf1
#define	NUC_RW_TEST	0xf2
#define	NUC_FILL_TEST	0xf3
#define	NUC_FREAD_TEST	0xf4
#define	NUC_BATT_TEST	0xf8
#define LOOPBACK		0xf9

// cntrl1
#define	POWER_RELAY			0x01
#define	REQUEST_CHARGE		0x02
#define	CHGR_START_REQ		REQUEST_CHARGE
#define	STOP_CHARGE			0x04
#define	CHK_CHG_RQST		0x06

#define	CHGR_STOP_REQ		STOP_CHARGE
#define	LIFT_ACT			0x20
#define UP_COMMAND 			0x40
#define WHEELBRAKE			0x80
#define LIFT_ALL_FLAG		(LIFT_ACT+UP_COMMAND)
#define WBRAKE_LIFT_CNT		(LIFT_ACT+UP_COMMAND+WHEELBRAKE)

// cntrl2
#define LEFT_COMMAND 		0x01
#define DEG180_COMMAND 		0x02
#define TURN_ACTIVE			0x04
#define TURN_NOT_ACTIVE		0x00
#define TURN_ALL_FLAG		(LEFT_COMMAND+DEG180_COMMAND+TURN_ACTIVE)

// cntrl4
#define RESET_MOTOR1_ALARM	0x01
#define RESET_MOTOR2_ALARM	0x02
#define RESET_LIFT_ALARM	0x04
#define RESET_TURN_ALARM	0x08

// --- TX ---
#define	TRUN_POSI_ANG0		0x80
#define	TRUN_POSI_ANG90		0x40
#define	TRUN_POSI_ANG180	0x20
#define	TRUN_POSI_ANG270	0x10
#define TURN_SENS_ALL		(TRUN_POSI_ANG0 + TRUN_POSI_ANG90 + TRUN_POSI_ANG180 + TRUN_POSI_ANG270)

#define	NON_DEFINE			0x08
#define	LIFT_DOWN_STATE		0x04
#define LIFT_UP_STATE		0x08
#define LIFT_COMPLTE		0x02
#define TRUN_COMPLTE		0x01

#define SENS_ALARM_SET		0x04				// alarm set
#define	SENS_BUMPER1		0x20				// bumper1 status
#define	SENS_BUMPER2		0x40				// bumper2 status

#define COM_ERROR_BIT		0x08

void init_RXBuffer(void);
uint8_t check_wheelbrake(void);
uint8_t check_lift_order(void);
uint8_t get_Lidar_Area(void);
uint8_t get_RxCntout4(void);
uint8_t get_RxCommand(void);
int16_t ck_RxDrvSpeed(void);
uint8_t ck_charge_req(void);
uint8_t ck_power_relay_req(void);
uint8_t check_wheelbrake(void);
int16_t get_LeftWheel_Speed(void);
int16_t get_RightWheel_Speed(void);
int16_t get_TurnTable_Speed(void);
uint8_t led_get_indication(void);
uint8_t get_sound_order(void);
void set_com_error_flag(void);
void reset_com_error_flag(void);
uint8_t get_com_error_flag(void);
void set_com_start_flag(void);
uint8_t get_com_start_flag(void);
void ck_com_error(void);
void check_nuc_receive_data(void);
void nuc_init(void);
void monitor_uart_dma_error(void);
void send_firmware_version(void);

#endif /* NUC_H_ */
