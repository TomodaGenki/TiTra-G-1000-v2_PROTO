/*
 * CNTRL_Proc.c
 * 
 *
 *  Created on: 2019/07/05
 *      Author: Takumi
 */


/* Includes ------------------------------*/
#include "main.h"
#include "stm32f4xx_hal_i2c.h"
#include "Cntrl_Proc.h"
#include "nuc.h"
#include "lift.h"
#include <limits.h> 							// CHAR_BIT
#include "wheel.h"
#include "conf.h"
#include "battery.h"
#include "charge.h"
#include "lidar.h"
#include "turn.h"
#include "optcom.h"
#include "task_charger_link.h"
#include "syncturn.h"
#include "load.h"
#include "wheel_test.h"

// ------------ define --------------------
#define	RETRY				0x03
#define	EEPROM_WRITE		0x0a				// 10msec delay for eeprom
#define I2C_EEPROM_ADRS		0xa0
#define READ_FLG			0x01
#define WRITE_SIZE			3
#define LIMIT_TIME			10
#define	EEPROM_MAXPKT		8
#define	SMBUS_ADDRESS		0x2f				// change device address command
#define	BATT_CHNG_SIZE		3					// packet size of changing the battery device address
#define	BATT_LABEL_SIZE		32					// max battery label size
#define	BATT_LABEL_SETTING_SIZE	7				// setting battery label size

#define	EMRGNCY_STOP		0x01				// emergency stop key
#define RELEASE_BRAKE1		0x01				// release brake key1
#define RELEASE_BRAKE2		0x02				// release brake key2
#define LIFT_UPDOWN1		0x04				//　lift up/down key1 manually
#define LIFT_UPDOWN2		0x08				//　lift up/down key2 manually
#define ALARM_RESET1		0x10				// alarm reset key1
#define ALARM_RESET2		0x20				// alarm reset key2
#define TURN_BRAKE1			0x40				// turn brake release key1
#define TURN_BRAKE2			0x80				// turn brake release key1
#define	BUMPER1				0x02				// bumper1 status
#define	BUMPER2				0x04				// bumper2 status
#define	HARD_ID_HIGH		0x00				// Hardware ID memory upper address
#define	HARD_ID_LOW			0x00				// Hardware ID memory lower address
#define	HW_ID_SIZE			0x02				// Hardware ID size
#define	B_LABEL_SIZE		0x0c				// battery label size

#define	PRESS_WHEEL_KEY		0x01
#define	PRESS_TURN_KEY		0x02
#define	WHEEL_BRAKE_RELEASE	0x04
#define	TURN_BRAKE_RELEASE	0x08
#define SENSOR1_MASK		0xFC

#define MCU_NOTHING			0x00
#define MCU_ACTION			0x01
#define MCU_EMRGNCY			0x02
#define MCU_FAILURE			0x03
#define MCU_ALARM			0x04

#define SENSOR1_TURN_BRAKE	0x02
#define	SENSOR1_MANU_BRAKE	0x04
#define SENSOR1_EMGNY_BTTRN	0x08
#define	SENSOR1_BUMPER_FRNT	0x10
#define	SENSOR1_BUMPER_BACK	0x20
#define	SENSOR1_LIMIT_LOWER	0x40
#define	SENSOR1_LIMIT_UPPER	0x80

#define MOTOR1_ALARM		0x01
#define MOTOR2_ALARM		0x02
#define MOTOR_LIFT_ALARM	0x04
#define MOTOR_TURN_ALARM	0x08

// ------------ define --------------------
extern	I2C_HandleTypeDef	hi2c2;

uint8_t	ID_buf[2];
static uint8_t cmd_data[RxBufSize];
static uint8_t mcu_state = 0;
static uint8_t sensor1 = 0;
static uint8_t sensor2 = 0;
static uint8_t emerge_key = 0;
static uint8_t key_buffer = 0;
static uint8_t brake_status = 0;
static uint8_t emergency_status = NOT_EMERGENCY;
static uint8_t bumper_sensor = 0;
static uint8_t alarm_reset_flg = 0;
static uint8_t alarm_status = 0;
static uint8_t bumper1_flag = 0;
static uint8_t bumper2_flag = 0;
static uint8_t emergekey_flag = 0;
static uint8_t wake_up_flag = 1;
static uint8_t motor_active_flg = 0;
static uint8_t cmd_process_flag = 0;

// --- timer ---
uint32_t	freerun = 0;

struct	trb {
	uint32_t	timer_tick;
	uint32_t	setting_time;
};
struct trb utimer[NO_TIMER];

// ---- for work ---------------------
uint8_t ComBuf[COMSIZE];

union u16 {
	uint16_t un16bit;
	uint8_t un8bit[2];
};

/*
 *  free run timer
 */
void time_run(void){
	freerun++;
}

/*
 *  set user timer
 */
void set_utimer(uint8_t t, uint32_t tim){
	utimer[t].timer_tick = tim;
	utimer[t].setting_time = freerun;
}

/*
 *  initiate timer
 */
void init_timer(void){
	set_utimer(CNT_GO1, CNT_PERIOD1);
	set_utimer(CNT_GO2, CNT_PERIOD2);
	set_utimer(CNT_GO3, CNT_PERIOD3);
	set_utimer(CNT_GO4, CNT_PERIOD4);
	set_utimer(CNT_GO5, CNT_PERIOD6);		// for NUC start up
	set_utimer(CNT_GO6, CNT_PERIOD5);		// for Charge Seaquence
	set_utimer(CNT_GO7, CNT_1MSEC);
	set_utimer(CDT_CHRGROUT, CNT_CHRGROUT);	// for Charge Seaquence
	set_utimer(CNT_TURN, CNT_TURNPERIOD);
	set_utimer(CNT_MOTOR_WAKEUP, MOTOR_WAKEUP_START);
	set_utimer(CNT_PROCESS, CNT_PROCPERIOD);
	// シンクロターンの関数は10ms毎にコールするが、位相をずらすため初期値を15msecに設定
	set_utimer(CNT_SYNCTURN, 15);
	set_utimer(CNT_WHEEL_WAIT, WHEEL_MOTOR_WAKE_UP);
	// エンコーダーコール関数は10ms毎にコールするが、位相をずらすため初期値を15msecに設定
	set_utimer(CNT_WHEEL_ENCODER, 15);
}

/*
 *  Timer check
 */
uint8_t check_timer(uint8_t t){
	uint8_t ret = TIME_NOT_UP;

	if ((utimer[t].timer_tick == 0) || ((freerun - utimer[t].setting_time) >= utimer[t].timer_tick)){
		utimer[t].timer_tick = 0;
		ret = TIME_UP;
	}
	return ret;
}

/*
 *  Write EEPROM
 */
void write_eeprom(uint8_t *p_mem_adrs, uint8_t *pD){
	uint16_t Size = 1;
	HAL_StatusTypeDef ret;
	union u16 type_conversion;

	type_conversion.un8bit[1] = *p_mem_adrs;		// upper address
	p_mem_adrs++;
	type_conversion.un8bit[0] = *p_mem_adrs;		// lower address
	uint16_t mem_adrs = type_conversion.un16bit;

	for (uint8_t i=0; i < RETRY; i++){
		ret = HAL_I2C_Mem_Write(&hi2c2, I2C_EEPROM_ADRS, mem_adrs, I2C_MEMADD_SIZE_16BIT, pD, Size, 2*LIMIT_TIME);
		HAL_Delay(2*EEPROM_WRITE);
		if (ret == HAL_OK) {
			break;
		}
		HAL_Delay(10*EEPROM_WRITE);	// caution-- this function is blocking,　so not used in interruption
	}
	return;
}

/*
 * Read EEPROM
 *
 */
void read_eeprom(uint8_t *p_mem_adrs, uint8_t *pD, uint8_t Size){

	uint16_t Counter = 0;
	HAL_StatusTypeDef ret;
	union u16 type_conversion;

	type_conversion.un8bit[1] = *p_mem_adrs;		// upper address
	p_mem_adrs++;
	type_conversion.un8bit[0] = *p_mem_adrs;		// lower address
	uint16_t mem_adrs = type_conversion.un16bit;

	//while (Counter < Size && Result == HAL_OK)
	while (Counter < Size)
	{
		int16_t Diff = Size - Counter;

		if (Diff >= EEPROM_MAXPKT)
		{
			//Multi-Byte
			ret = HAL_I2C_Mem_Read(&hi2c2, I2C_EEPROM_ADRS, mem_adrs + Counter, I2C_MEMADD_SIZE_16BIT, &pD[Counter], EEPROM_MAXPKT, LIMIT_TIME);
			Counter += EEPROM_MAXPKT;
		}
		else
		{
			//and the remaining ones...low packet size
			ret = HAL_I2C_Mem_Read(&hi2c2, I2C_EEPROM_ADRS, mem_adrs + Counter, I2C_MEMADD_SIZE_16BIT, &pD[Counter], Diff, LIMIT_TIME);
			Counter += Diff;
		}
		// HAL_Delay(EEPROM_WRITE / 5);
	}
	return;
}

/*
 *  read hardware ID(unique ID:identify the microcomputer board)
 *  1 time when system starting
 */
void read_HardwareID(void){
	// load hardware ID
	uint8_t id_address[2];
	id_address[0] = HARD_ID_HIGH;
	id_address[1] = HARD_ID_LOW;
	read_eeprom(&id_address[0], &ID_buf[0], HW_ID_SIZE);
}

uint8_t get_HardwareID_High(void) {
	return ID_buf[1];
}

uint8_t get_HardwareID_Low(void) {
	return ID_buf[0];
}

/*
 *  write hardware ID(unique ID:identify the microcomputer board)
 *  1 time when system starting
 */
void write_HardwareID(uint8_t *p){
	uint8_t	id;
	union u16 type_conversion;

	type_conversion.un8bit[0] = (uint8_t)*(p+4);	// set memory upper address
	type_conversion.un8bit[1] = (uint8_t)*(p+5);	// set memory lower address
	id = (uint8_t)*(p+7);
	write_eeprom(&type_conversion.un8bit[0], &id);							// ID upper

	type_conversion.un8bit[1]++;
	id = (uint8_t)*(p+6);
	write_eeprom(&type_conversion.un8bit[0], &id);							// ID lower
}

/*
 *  calculate CRC8
 */
uint8_t bCrc(uint8_t remainder, uint8_t byte)
{
	char i = 0;
	remainder ^= byte;

	for (i = 8; i > 0; --i){
		if (remainder & 0x80){
			remainder = (remainder << 1) ^ 0x7;
		}
		else{
			remainder = (remainder << 1);
		}
	}
	return remainder;
}

/*
 *  calc crc8 of packet
 */
uint8_t calc_CRC8(uint8_t *buff, uint8_t size){
	uint8_t remain=0;

	for (uint8_t i = 0; i < size ; i++){
		remain = bCrc(remain, *buff);
		buff++;
	}
	return remain;
}

/*
 *  battery read test for debug
 */
HAL_StatusTypeDef read_battery_test(uint16_t batt_add, uint8_t cmmnd, uint8_t *pD, uint8_t size){
	HAL_StatusTypeDef Result = HAL_OK;

	for (uint8_t i =0; i < 2; i++){
		//and the remaining ones...low packet size
		//HAL_I2C_Mem_Read(&hi2c2, batt_add, BATTERY_STATUS_CMD, I2C_MEMADD_SIZE_16BIT, pD, Size, LIMIT_TIME);
		Result = HAL_I2C_Mem_Read(&hi2c2, batt_add, cmmnd, I2C_MEMADD_SIZE_8BIT, pD, size, LIMIT_TIME);

		if (Result == HAL_OK) break;
		HAL_Delay(EEPROM_WRITE);
	}

	return	Result;
}

/*
 *  read device
 */
HAL_StatusTypeDef read_battery_address(uint16_t batt_add, uint8_t cmmnd, uint8_t *pD){
	HAL_StatusTypeDef Result = HAL_OK;
	uint8_t size =1;

	Result = read_battery_test(batt_add, cmmnd, pD, size);

	return Result;
}

/*
 *  Read battery status
 */
HAL_StatusTypeDef read_battery_status(uint8_t batt_add, uint8_t cmmnd, uint8_t *pD){
	HAL_StatusTypeDef Result = HAL_OK;
	uint16_t Size = B_STATUS_SIZE;													// + 1byte(for CRC8)
	uint8_t temp[Size+1];

	//and the remaining ones...low packet size
	//HAL_I2C_Mem_Read(&hi2c2, batt_add, BATTERY_STATUS_CMD, I2C_MEMADD_SIZE_16BIT, pD, Size, LIMIT_TIME);
	Result = HAL_I2C_Mem_Read(&hi2c2, batt_add, cmmnd, I2C_MEMADD_SIZE_8BIT, temp, (Size+1), LIMIT_TIME);

	if (Result == HAL_OK){
		for (int i = 0 ; i < Size ; i++){
			*pD++ = temp[i];
		}
	}
	// HAL_Delay(EEPROM_WRITE / 2);

	return	Result;
}

/*
 *  Command Control for debug
 */
void set_cmd_process_flag(void) {
	cmd_process_flag = 1;
}

void reset_cmd_process_flag(void) {
	cmd_process_flag = 0;
}

uint8_t get_cmd_process_flag(void) {
	return cmd_process_flag;
}

void set_command_data(uint8_t data, uint8_t i) {
	cmd_data[i] = data;
}

void cmnd_proc(void){

	if (get_cmd_process_flag() != 0) {
		reset_cmd_process_flag();

		switch(cmd_data[2]) {
		case NUC_R_ID:										// Read ID of micon into eeprom
			read_HardwareID();
			break;

		case NUC_W_ID:										// Write ID of micon into eeprom
//			write_HardwareID(&RxWork[pRtRx][0]);
			break;

		case NUC_R_EP:										// Read 1 byte data of micon into eeprom
//			read_eeprom(&RxWork[pRtRx][4], ComBuf, 1);
			break;

		case NUC_W_EP:										// write 1 byte data of micon into eeprom
//			write_eeprom(&RxWork[pRtRx][4], &RxWork[pRtRx][6]);
			break;

		case NUC_R_BATT_ST:									// get battery status
//			read_battery_status((uint8_t)RxWork[pRtRx][3], (uint8_t)RxWork[pRtRx][4], ComBuf);
			break;

		case NUC_R_BATT_ADD:								// get battery device address(for I2C)
//			read_battery_address((uint8_t)RxWork[pRtRx][3], (uint8_t)RxWork[pRtRx][4], ComBuf);
			break;

		case NUC_W_EMRGNCY_RESET:												// receive recovery command from jetson
			recovery_emergency();
			break;

		case SYNCTURN_LOG_CMD:
			set_sync_dump_req();
			break;

		case TURN_LOG_CMD:
			set_turn_dump_req();
			break;

		case WHEEL_LOG_CMD:
			set_wheel_dump_req();
			break;

		case WHEEL_TEST_LOG_CMD:
			set_wheel_test_dump_req();
			break;

		case NUC_R_FW_VER:
			send_firmware_version();
			break;

		case LIFT_LOAD_LOG_CMD:
			set_lift_load_log_flag();
			break;

		case TURN_LOAD_LOG_CMD:
			set_turn_load_log_flag();
			break;

		default:
			break;
		}
//		HAL_UART_ResponseData(RxWork[pRtRx][2]);									// completed
	}
}

/*
 * manual brake operation
 */
void manul_brake(void){

	if ((ck_brake_release1_key() != 0) || (ck_brake_release2_key() != 0)) {
		brake_status |= PRESS_WHEEL_KEY;													// push wheel brake release key
	}

	if ((ck_turn_brake1_key() != 0) || (ck_turn_brake2_key() != 0)) {
		brake_status |= PRESS_TURN_KEY;													// push turn brake release key
	}

	// ON ⇒ OFF エッジを検出してブレーキを解除する
	if (brake_status & PRESS_WHEEL_KEY) {
		if ((ck_brake_release1_key() == 0) && (ck_brake_release2_key() == 0)){
			brake_status ^= WHEEL_BRAKE_RELEASE;											// toggle
			brake_status &= ~PRESS_WHEEL_KEY;
		}
	}

	if (brake_status & PRESS_TURN_KEY) {
		if ((ck_turn_brake1_key() == 0) && (ck_turn_brake2_key() == 0)){
			brake_status ^= TURN_BRAKE_RELEASE;											// toggle
			brake_status &= ~PRESS_TURN_KEY;
		}
	}
}

uint8_t check_wheel_brake(void) {
	return brake_status & WHEEL_BRAKE_RELEASE;
}

uint8_t check_turn_brake(void) {
	return brake_status & TURN_BRAKE_RELEASE;
}

/*
 *  reset alarm manually
 */

uint8_t alarm_reset_select(void) {

	uint8_t reset_module = RELEASE_END;

	if ((get_bumper1_flag() != 0) || (get_bumper2_flag() != 0)) {
		reset_module |= RELEASE_BUMPER;
	}
	if (get_com_error_flag() != 0) {
		reset_module |= RELEASE_COMMERROR;
	}
	if (get_emergekey_flag() != 0) {
		reset_module |= RELEASE_EMERGENCY_KEY;
	}
	if (ck_mt_alarm() != 0) {
		reset_module |= RELEASE_WHEEL_ERROR;
	}
	if (ck_lidar_failure() != 0) {
		reset_module |= RELEASE_LIDAR;
	}
	if ((ck_lift_motor1_alarm() != 0) || (ck_lift_motor2_alarm() != 0)){
		reset_module |= RELEASE_LIFT_ERROR;
	}
	if (ck_turn_motor_alarm() != 0) {
		reset_module |= RELEASE_TURN_ERROR;
	}
	if (ck_lift_limit() != 0) {
		reset_module |= RELEASE_LIFT_LIMIT;
	}
	return reset_module;
}

void alarm_reset_control(uint8_t reset_select) {

	if (reset_select == RELEASE_END) {
		set_emergency(NOT_EMERGENCY);
	} else {
		if ((reset_select & RELEASE_WHEEL_ERROR) == RELEASE_WHEEL_ERROR) {
			reset_alarm();
		}
		if ((reset_select & RELEASE_LIFT_ERROR) == RELEASE_LIFT_ERROR) {
			lift_alarm_reset();
		}
		if ((reset_select & RELEASE_TURN_ERROR) == RELEASE_TURN_ERROR) {
			turn_alarm_reset();
		}
		if ((reset_select & RELEASE_LIFT_LIMIT) == RELEASE_LIFT_LIMIT) {
			if ((get_lift1_limit_sensor() == 0) && (get_lift2_limit_sensor() == 0)) {
				reset_lift_upper_limit();
				reset_lift_lower_limit();
			}
		}
	}
}

uint8_t wait_error_release(uint8_t error_release) {

	uint8_t error_remain = error_release;

	if ((error_remain & RELEASE_BUMPER) == RELEASE_BUMPER) {
		if (check_bumper() == 0) {
			error_remain &= ~RELEASE_BUMPER;
			reset_bumper1_flag();
			reset_bumper2_flag();
		} else {
			if ((check_bumper() & SENS_BUMPER1) == 0) {
				reset_bumper1_flag();
			}
			if ((check_bumper() & SENS_BUMPER2) == 0) {
				reset_bumper2_flag();
			}
		}
	}
	if ((error_remain & RELEASE_COMMERROR) == RELEASE_COMMERROR) {
		if (check_timer(CNT_GO5) != TIME_UP) {
			error_remain &= ~RELEASE_COMMERROR;
			reset_com_error_flag();
		}
	}
	if ((error_remain & RELEASE_EMERGENCY_KEY) == RELEASE_EMERGENCY_KEY) {
		if (ck_emgncy_key() == 0) {
			error_remain &= ~RELEASE_EMERGENCY_KEY;
			reset_emergekey_flag();
		}
	}
	if ((error_remain & RELEASE_LIDAR) == RELEASE_LIDAR) {
		if (ck_lidar_failsens() == 0) {
			error_remain &= ~RELEASE_LIDAR;
			reset_lidar1_fail_flag();
			reset_lidar2_fail_flag();
		} else {
			if ((ck_lidar_failsens() & LIDAR1_FAILURE) == 0) {
				reset_lidar1_fail_flag();
			}
			if ((ck_lidar_failsens() & LIDAR2_FAILURE) == 0) {
				reset_lidar2_fail_flag();
			}
		}
	}
	if ((error_remain & RELEASE_WHEEL_ERROR) == RELEASE_WHEEL_ERROR) {
		recover_alarm();
		if (ck_mt_alarm() == 0) {
			error_remain &= ~RELEASE_WHEEL_ERROR;
		}
	}
	if ((error_remain & RELEASE_LIFT_ERROR) == RELEASE_LIFT_ERROR) {
		lift_alarm_recover();
		if ((ck_lift_motor1_alarm() == 0) && (ck_lift_motor2_alarm() == 0)) {
			error_remain &= ~RELEASE_LIFT_ERROR;
		}
	}
	if ((error_remain & RELEASE_TURN_ERROR) == RELEASE_TURN_ERROR) {
		turn_alarm_recover();
		if (ck_turn_motor_alarm() == 0) {
			error_remain &= ~RELEASE_TURN_ERROR;
		}
	}
	if ((error_remain & RELEASE_LIFT_LIMIT) == RELEASE_LIFT_LIMIT) {
		if (ck_lift_limit() == 0) {
			error_remain &= ~RELEASE_LIFT_LIMIT;
		}
	}

	return error_remain;
}

void manul_reset_alarm(void){

	static uint8_t error_select = RELEASE_END;
	static uint8_t error_wait_timer = 0;

	if ((alarm_reset_flg & RESET_ALARM_MANUAL) == RESET_ALARM_MANUAL) {
		error_select = wait_error_release(error_select);
		error_wait_timer--;
		if (error_select == RELEASE_END) {
			alarm_reset_flg &= (~RESET_ALARM_MANUAL);
			set_emergency(NOT_EMERGENCY);
		} else if (error_wait_timer == 0) {
			alarm_reset_flg &= (~RESET_ALARM_MANUAL);
			recover_alarm();
			error_select = RELEASE_END;
			set_emergency(NOT_EMERGENCY);	//状態再確認のため、一旦解除
		}
	}

	if ((ck_alarm_release1_key() != 0) || (ck_alarm_release2_key() != 0)) {
		if (((alarm_reset_flg & (RESET_ALARM_MANUAL + RESET_ALARM_AUTO)) == 0) && (ck_emerg_stop() != NOT_EMERGENCY)) {
			init_RXBuffer();									// initiate communication buffer of UART
			error_select = alarm_reset_select();
			alarm_reset_control(error_select);
			if (error_select != RELEASE_END) {
				error_wait_timer = RELEASE_WAIT_TIME;
				alarm_reset_flg |= RESET_ALARM_MANUAL;
			}
		}
	}
}

/*
 *  reset alarm automatically
 */
void auto_reset_alarm(void){

	static uint8_t alarm_mng = 0;
	static uint8_t error_wait_timer = 0;
	uint8_t cntrlout4 = get_RxCntout4();

	cntrlout4 = cntrlout4 & (RESET_MOTOR1_ALARM + RESET_MOTOR2_ALARM + RESET_LIFT_ALARM + RESET_TURN_ALARM);

	if ((alarm_reset_flg & RESET_ALARM_AUTO) == RESET_ALARM_AUTO) {
		error_wait_timer--;
		alarm_mng = wait_error_release(alarm_mng);
		if (alarm_mng == RELEASE_END) {
			alarm_reset_flg &= (~RESET_ALARM_AUTO);
			if ((ck_emerg_stop() == EMERGENCY_CT3) || (ck_emerg_stop() == EMERGENCY_CT4)) {
				set_emergency(NOT_EMERGENCY);
			}
		} else if (error_wait_timer == 0) {
			alarm_reset_flg &= (~RESET_ALARM_AUTO);
			recover_alarm();
		}
	}

	if (((alarm_reset_flg & (RESET_ALARM_MANUAL + RESET_ALARM_AUTO)) == 0) && (cntrlout4 != 0)) {
		alarm_reset_flg |= RESET_ALARM_AUTO;
		error_wait_timer = RELEASE_WAIT_TIME;

		if ((cntrlout4 & RESET_MOTOR1_ALARM) == RESET_MOTOR1_ALARM) {
			if (ck_wheel1_motor_alarm() != 0) {
				reset_motor1_alarm();
				alarm_mng |= RELEASE_WHEEL_ERROR;
			}
		}

		if ((cntrlout4 & RESET_MOTOR2_ALARM) == RESET_MOTOR2_ALARM) {
			if (ck_wheel2_motor_alarm() != 0) {
				reset_motor2_alarm();
				alarm_mng |= RELEASE_WHEEL_ERROR;
			}
		}

		if ((cntrlout4 & RESET_LIFT_ALARM) == RESET_LIFT_ALARM) {
			if ((ck_lift_motor1_alarm() != 0) || (ck_lift_motor2_alarm() != 0)) {
				lift_alarm_reset();
				alarm_mng |= RELEASE_LIFT_ERROR;
			}
		}

		if ((cntrlout4 & RESET_TURN_ALARM) == RESET_TURN_ALARM) {
			if (ck_turn_motor_alarm() != 0) {
				turn_alarm_reset();
				alarm_mng |= RELEASE_TURN_ERROR;
			}
		}
	}
}

void recovery_emergency(void){
	if (ck_emerg_stop() == EMERGENCY_CT1) {
		init_RXBuffer();									// initiate communication buffer of UART
		set_emergency(NOT_EMERGENCY);						// reset emergency status
		reset_com_error_flag();
	}
}

/*
 * scan_bumper
 * scan period 20msec
 */
void scan_bumper(void){

	static uint8_t sense[2] = {0,0};

	sense[1] = sense[0];
	// bumper1
	if(HAL_GPIO_ReadPin(I_Bumper1_GPIO_Port, I_Bumper1_Pin) == GPIO_PIN_RESET){
		sense[0] = sense[0] | BUMPER1;				// bumper1 status
	} else {
		sense[0] = sense[0] & ~BUMPER1;				// bumper1 status
	}

	// bumper2
	if(HAL_GPIO_ReadPin(I_Bumber2_GPIO_Port, I_Bumber2_Pin) == GPIO_PIN_RESET){
		sense[0] = sense[0] | BUMPER2;				// bumper2 status
	} else {
		sense[0] = sense[0] & ~BUMPER2;				// bumper2 status
	}

	// check chattering
	if ((BUMPER1 == (sense[0] & BUMPER1)) && (BUMPER1 == (sense[1] & BUMPER1))){
		bumper_sensor = bumper_sensor | SENS_BUMPER1;
	} else if ((0 == (sense[0] & BUMPER1)) && (0 == (sense[1] & BUMPER1))){
		bumper_sensor = bumper_sensor & ~SENS_BUMPER1;
	}

	if ((BUMPER2 == (sense[0] & BUMPER2)) && (BUMPER2 == (sense[1] & BUMPER2))){
		bumper_sensor = bumper_sensor | SENS_BUMPER2;
	} else if ((0 == (sense[0] & BUMPER2)) && (0 == (sense[1] & BUMPER2))){
		bumper_sensor = bumper_sensor & ~SENS_BUMPER2;
	}
}

void emerge_key_scan(void) {

	static uint8_t emerge_key_sense = 0;

	emerge_key_sense = emerge_key_sense << 1;

	if(HAL_GPIO_ReadPin(I_EmergencyMoni_GPIO_Port, I_EmergencyMoni_Pin) == GPIO_PIN_RESET){
		emerge_key_sense |= 0x01;
	}

	// check chattering
	// emergency key
	if (emerge_key_sense == 0xFF){
		emerge_key = emerge_key | EMRGNCY_STOP;
	} else if (emerge_key_sense == 0x00){
		emerge_key = emerge_key & ~EMRGNCY_STOP;
	}
}

/*
 *  keyScan
 *  scan period 20msec
 */
void keyScan(void){

	static uint8_t key_sense[2] = {0,0};

	key_sense[1] = key_sense[0];

	// release brake
	if(HAL_GPIO_ReadPin(I_BrakeReleSw_R_GPIO_Port, I_BrakeReleSw_R_Pin) == GPIO_PIN_RESET){
		key_sense[0] = key_sense[0] | RELEASE_BRAKE1;
	} else {
		key_sense[0] = key_sense[0] & ~RELEASE_BRAKE1;
	}

	if(HAL_GPIO_ReadPin(I_BrakeReleSw_F_GPIO_Port, I_BrakeReleSw_F_Pin) == GPIO_PIN_RESET){
		key_sense[0] = key_sense[0] | RELEASE_BRAKE2;
	} else {
		key_sense[0] = key_sense[0] & ~RELEASE_BRAKE2;
	}

	// lift up/down manually
	if(HAL_GPIO_ReadPin(I_LiftSw_R_GPIO_Port, I_LiftSw_R_Pin) == GPIO_PIN_RESET){
		key_sense[0] = key_sense[0] | LIFT_UPDOWN1;
	} else {
		key_sense[0] = key_sense[0] & ~LIFT_UPDOWN1;
	}

	if(HAL_GPIO_ReadPin(I_LiftSw_F_GPIO_Port, I_LiftSw_F_Pin) == GPIO_PIN_RESET){
		key_sense[0] = key_sense[0] | LIFT_UPDOWN2;
	} else {
		key_sense[0] = key_sense[0] & ~LIFT_UPDOWN2;
	}

	// reset alarm
	if(HAL_GPIO_ReadPin(I_ResetSw_R_GPIO_Port, I_ResetSw_R_Pin) == GPIO_PIN_RESET){
		key_sense[0] = key_sense[0] | ALARM_RESET1;
	} else {
		key_sense[0] = key_sense[0] & ~ALARM_RESET1;
	}

	if(HAL_GPIO_ReadPin(I_ResetSw_F_GPIO_Port, I_ResetSw_F_Pin) == GPIO_PIN_RESET){
		key_sense[0] = key_sense[0] | ALARM_RESET2;
	} else {
		key_sense[0] = key_sense[0] & ~ALARM_RESET2;
	}

	if(HAL_GPIO_ReadPin(I_TurnBrakeSw_R_GPIO_Port, I_TurnBrakeSw_R_Pin) == GPIO_PIN_RESET){
		key_sense[0] = key_sense[0] | TURN_BRAKE1;
	} else {
		key_sense[0] = key_sense[0] & ~TURN_BRAKE1;
	}

	if(HAL_GPIO_ReadPin(I_TurnBrakeSw_F_GPIO_Port, I_TurnBrakeSw_F_Pin) == GPIO_PIN_RESET){
		key_sense[0] = key_sense[0] | TURN_BRAKE2;
	} else {
		key_sense[0] = key_sense[0] & ~TURN_BRAKE2;
	}

	// release brake key
	if ((RELEASE_BRAKE1 == (key_sense[0] & RELEASE_BRAKE1)) && (RELEASE_BRAKE1 == (key_sense[1] & RELEASE_BRAKE1))){
		key_buffer = key_buffer | RELEASE_BRAKE1;
	} else if ((0 == (key_sense[0] & RELEASE_BRAKE1)) && (0 == (key_sense[1] & RELEASE_BRAKE1))){
		key_buffer = key_buffer & ~RELEASE_BRAKE1;
	}

	if ((RELEASE_BRAKE2 == (key_sense[0] & RELEASE_BRAKE2)) && (RELEASE_BRAKE2 == (key_sense[1] & RELEASE_BRAKE2))){
		key_buffer = key_buffer | RELEASE_BRAKE2;
	} else if ((0 == (key_sense[0] & RELEASE_BRAKE2)) && (0 == (key_sense[1] & RELEASE_BRAKE2))){
		key_buffer = key_buffer & ~RELEASE_BRAKE2;
	}

	// lift up/down key
	if ((LIFT_UPDOWN1 == (key_sense[0] & LIFT_UPDOWN1)) && (LIFT_UPDOWN1 == (key_sense[1] & LIFT_UPDOWN1))){
		key_buffer = key_buffer | LIFT_UPDOWN1;
	} else if ((0 == (key_sense[0] & LIFT_UPDOWN1)) && (0 == (key_sense[1] & LIFT_UPDOWN1))){
		key_buffer = key_buffer & ~LIFT_UPDOWN1;
	}

	if ((LIFT_UPDOWN2 == (key_sense[0] & LIFT_UPDOWN2)) && (LIFT_UPDOWN2 == (key_sense[1] & LIFT_UPDOWN2))){
		key_buffer = key_buffer | LIFT_UPDOWN2;
	} else if ((0 == (key_sense[0] & LIFT_UPDOWN2)) && (0 == (key_sense[1] & LIFT_UPDOWN2))){
		key_buffer = key_buffer & ~LIFT_UPDOWN2;
	}

	// reset alarm key
	if ((ALARM_RESET1 == (key_sense[0] & ALARM_RESET1)) && (ALARM_RESET1 == (key_sense[1] & ALARM_RESET1))){
		key_buffer = key_buffer | ALARM_RESET1;
	} else if ((0 == (key_sense[0] & ALARM_RESET1)) && (0 == (key_sense[1] & ALARM_RESET1))){
		key_buffer = key_buffer & ~ALARM_RESET1;
	}

	if ((ALARM_RESET2 == (key_sense[0] & ALARM_RESET2)) && (ALARM_RESET2 == (key_sense[1] & ALARM_RESET2))){
		key_buffer = key_buffer | ALARM_RESET2;
	} else if ((0 == (key_sense[0] & ALARM_RESET2)) && (0 == (key_sense[1] & ALARM_RESET2))){
		key_buffer = key_buffer & ~ALARM_RESET2;
	}

	// turn brake release key
	if ((TURN_BRAKE1 == (key_sense[0] & TURN_BRAKE1)) && (TURN_BRAKE1 == (key_sense[1] & TURN_BRAKE1))){
		key_buffer = key_buffer | TURN_BRAKE1;
	} else if ((0 == (key_sense[0] & TURN_BRAKE1)) && (0 == (key_sense[1] & TURN_BRAKE1))){
		key_buffer = key_buffer & ~TURN_BRAKE1;
	}

	if ((TURN_BRAKE2 == (key_sense[0] & TURN_BRAKE2)) && (TURN_BRAKE2 == (key_sense[1] & TURN_BRAKE2))){
		key_buffer = key_buffer | TURN_BRAKE2;
	} else if ((0 == (key_sense[0] & TURN_BRAKE2)) && (0 == (key_sense[1] & TURN_BRAKE2))){
		key_buffer = key_buffer & ~TURN_BRAKE2;
	}
}

uint8_t ck_emgncy_key(void){
	return emerge_key & EMRGNCY_STOP;
}

uint8_t ck_lift_updown1_key(void){
	return key_buffer & LIFT_UPDOWN1;
}

uint8_t ck_lift_updown2_key(void){
	return key_buffer & LIFT_UPDOWN2;
}

uint8_t ck_alarm_release1_key(void) {
	return key_buffer & ALARM_RESET1;
}

uint8_t ck_alarm_release2_key(void) {
	return key_buffer & ALARM_RESET2;
}

uint8_t ck_brake_release1_key(void) {
	return key_buffer & RELEASE_BRAKE1;
}

uint8_t ck_brake_release2_key(void) {
	return key_buffer & RELEASE_BRAKE2;
}

uint8_t ck_turn_brake1_key(void) {
	return key_buffer & TURN_BRAKE1;
}

uint8_t ck_turn_brake2_key(void) {
	return key_buffer & TURN_BRAKE2;
}

void set_bumper1_flag(void) {
	bumper1_flag = 1;
}

void reset_bumper1_flag(void) {
	bumper1_flag = 0;
}

uint8_t get_bumper1_flag(void) {
	return bumper1_flag;
}

void set_bumper2_flag(void) {
	bumper2_flag = 1;
}

void reset_bumper2_flag(void) {
	bumper2_flag = 0;
}

uint8_t get_bumper2_flag(void) {
	return bumper2_flag;
}

void set_emergekey_flag(void) {
	emergekey_flag = 1;
}

void reset_emergekey_flag(void) {
	emergekey_flag = 0;
}

uint8_t get_emergekey_flag(void) {
	return emergekey_flag;
}

void set_LiftActive_state(void) {
	sensor2 &= ~LIFT_COMPLTE;
}

void set_LiftComplete_state(void) {
	sensor2 |=LIFT_COMPLTE;
}

void set_LiftUpper_state(void) {
	sensor2 |= LIFT_UP_STATE;
}

void reset_LiftUpper_state(void) {
	sensor2 &= ~LIFT_UP_STATE;
}

void set_LiftLower_state(void) {
	sensor2 |= LIFT_DOWN_STATE;
}

void reset_LiftLower_state(void) {
	sensor2 &= ~LIFT_DOWN_STATE;
}

void set_TurnAngle0_state(void) {
	sensor2 |= TRUN_POSI_ANG0;
}

void reset_TurnAngle0_state(void) {
	sensor2 &= ~TRUN_POSI_ANG0;
}

void set_TurnAngle90_state(void) {
	sensor2 |= TRUN_POSI_ANG90;
}

void reset_TurnAngle90_state(void) {
	sensor2 &= ~TRUN_POSI_ANG90;
}

void set_TurnAngle180_state(void) {
	sensor2 |= TRUN_POSI_ANG180;
}

void reset_TurnAngle180_state(void) {
	sensor2 &= ~TRUN_POSI_ANG180;
}

void set_TurnAngle270_state(void) {
	sensor2 |= TRUN_POSI_ANG270;
}

void reset_TurnAngle270_state(void) {
	sensor2 &= ~TRUN_POSI_ANG270;
}

void set_TurnActive_state(void) {
	sensor2 &= ~TRUN_COMPLTE;
}

void set_TurnComplete_state(void) {
	sensor2 |=TRUN_COMPLTE;
}

uint8_t get_sensor2_state(void) {
	return sensor2;
}

uint8_t get_turn_sensor_pos(void) {
	return sensor2 & TURN_SENS_ALL;
}

uint8_t get_lift_pos(void) {
	return sensor2 & LIFT_DOWN_STATE;
}

/*
 * check bumper on/off
 *
 */
uint8_t check_bumper(void){
	return bumper_sensor & (SENS_BUMPER1+SENS_BUMPER2);
}

/*
 *  scan motor alarm signal(traveling, lift, turn table)
 */
void scan_motor_alarm(void){

	if (ck_wheel1_motor_alarm() != 0) {
		alarm_status |= MOTOR1_ALARM;
	} else {
		alarm_status &= (~MOTOR1_ALARM);
	}

	if (ck_wheel2_motor_alarm() != 0) {
		alarm_status |= MOTOR2_ALARM;
	} else {
		alarm_status &= (~MOTOR2_ALARM);
	}

	if ((ck_lift_motor1_alarm() != 0) || (ck_lift_motor2_alarm() != 0)) {
		alarm_status |= MOTOR_LIFT_ALARM;
	} else {
		alarm_status &= (~MOTOR_LIFT_ALARM);
	}

	if (ck_turn_motor_alarm() != 0) {
		alarm_status |= MOTOR_TURN_ALARM;
	} else {
		alarm_status &= (~MOTOR_TURN_ALARM);
	}
}

/*
 *  scan period 20msec
 */
void scan_proc1(void){
	// read optical communication sensor
	scan_optdata();

	// bumper
	scan_bumper();

	// emergency stop key
	emerge_key_scan();

	// manual key
	keyScan();

	// key processing
	manul_brake();					// brake release

	// scan lift motor alarm
	scan_liftmotor_alarm();

	// scan turn motor alarm
	scan_turnmotor_alarm();

	// scan wheel motor alarm
	scan_wheelmotor_alarm();

	// scan lift sensor
	set_lift_sens();

	// lift limit
	judge_lift_limit();

	//scan turn angle photo sensor
	turn_sensor_check();

	// sensor of obstacles
	scan_lidar();

	// judge emergency flag
	ck_emerge_sens();

	// reset motor driver alarm manually
	manul_reset_alarm();

	// reset motor drive alarm automatically
	auto_reset_alarm();

	// monitor emergency stop event
	monitor_emerg_stop();

	// set sensor1 state
	ck_sensor1_state();

	// set mcu state
	ck_mcu_state();

	// motor power-on control
	motor_wakeup_cntrl();

	if (ck_motor_active() != 0) {		// モーター電源OFF時、アラームの検出は行わない
		scan_motor_alarm();			// scan every motor alarm signal
	}

	// charger data read
	charger_data_receive();

	// write optical communication sensor
	output_optdata();

	// monitoring uart error and reset
	monitor_uart_dma_error();

	// monitoring dma error and reset
	monitor_irda_dma_error();

	monitor_driver_dma_error();
}

uint8_t ck_mt_alarm(void){

	uint8_t ret = (alarm_status & (MOTOR1_ALARM + MOTOR2_ALARM));

	return ret;
}

uint8_t ck_lift_alarm(void) {

	uint8_t ret = (alarm_status & MOTOR_LIFT_ALARM);

	return ret;
}

uint8_t ck_turn_alarm(void) {

	uint8_t ret = (alarm_status & MOTOR_TURN_ALARM);

	return ret;
}

uint8_t send_mt_alarm(void){
	return alarm_status;
}

/*
 *  monitor emergency event
 */

void monitor_emerg_stop(void){
	// check emergency level
	switch (emergency_status){
	case NOT_EMERGENCY:
		if (check_bumper() != 0) {
			// bumper on?
			emergency_status = EMERGENCY_CT0;
		} else if (check_timer(CNT_GO5) == TIME_UP) {
			// communication error with NUC?
			emergency_status = EMERGENCY_CT1;
		} else if (ck_emgncy_key() != 0) {
			// Emergency stop button is pushed?
			emergency_status = EMERGENCY_CT2;
		} else if((ck_mt_alarm() != 0) || (get_lidar1_fail_flag() != 0) || (get_lidar2_fail_flag() != 0)){
			emergency_status = EMERGENCY_CT3;
		} else if ((ck_lift_limit() != 0) || (ck_lift_alarm() != 0) || (ck_turn_alarm() != 0)) {
			emergency_status = EMERGENCY_CT4;
		}
		break;

	case EMERGENCY_CT4:
		if (check_bumper() != 0) {
			// bumper on?
			emergency_status = EMERGENCY_CT0;
		} else if (check_timer(CNT_GO5) == TIME_UP) {
			// communication error with NUC?
			emergency_status = EMERGENCY_CT1;
		} else if (ck_emgncy_key() != 0) {
			// Emergency stop button is pushed?
			emergency_status = EMERGENCY_CT2;
		} else if((ck_mt_alarm() != 0) || (get_lidar1_fail_flag() != 0) || (get_lidar2_fail_flag() != 0)){
			emergency_status = EMERGENCY_CT3;
		}
		break;

	case EMERGENCY_CT3:
		if (check_bumper() != 0) {
			// bumper on?
			emergency_status = EMERGENCY_CT0;
		} else if (check_timer(CNT_GO5) == TIME_UP) {
			// communication error with NUC?
			emergency_status = EMERGENCY_CT1;
		} else if (ck_emgncy_key() != 0) {
			// Emergency stop button is pushed?
			emergency_status = EMERGENCY_CT2;
		}
		break;

	case EMERGENCY_CT2:
		if (check_bumper() != 0) {
			// bumper on?
			emergency_status = EMERGENCY_CT0;
		} else if (check_timer(CNT_GO5) == TIME_UP) {
			// communication error with NUC?
			emergency_status = EMERGENCY_CT1;
		}
		break;

	case EMERGENCY_CT1:
		if (check_bumper() != 0) {
			// bumper on?
			emergency_status = EMERGENCY_CT0;
		}
		break;

	case EMERGENCY_CT0:
		// Most serious emergency
		break;

	default:
		if (check_bumper() != 0) {
			// bumper on?
			emergency_status = EMERGENCY_CT0;
		} else if (check_timer(CNT_GO5) == TIME_UP) {
			// communication error with NUC?
			emergency_status = EMERGENCY_CT1;
		} else if (ck_emgncy_key() != 0) {
			// Emergency stop button is pushed?
			emergency_status = EMERGENCY_CT2;
		} else if((ck_mt_alarm() != 0) || (get_lidar1_fail_flag() != 0) || (get_lidar2_fail_flag() != 0)){
			emergency_status = EMERGENCY_CT3;
		} else if ((ck_lift_limit() != 0) || (ck_lift_alarm() != 0) || (ck_turn_alarm() != 0)) {
			emergency_status = EMERGENCY_CT4;
		}
		break;
	}
	return;
}

/*
 *  check emergency stop event
 */
uint8_t ck_emerg_stop(void){
	return emergency_status;
}

void set_emergency(uint8_t emergency){
	emergency_status = emergency;
}

void ck_mcu_state(void) {

	if((ck_emerg_stop() == EMERGENCY_CT0) || (ck_emerg_stop() == EMERGENCY_CT1) || (ck_emerg_stop() == EMERGENCY_CT2) || (ck_emerg_stop() == EMERGENCY_CT3)) {
		mcu_state = MCU_EMRGNCY;
	} else if (ck_emerg_stop() == EMERGENCY_CT4) {
		mcu_state = MCU_FAILURE;
	} else if (ck_mt_alarm() != 0) {
		mcu_state = MCU_ALARM;
	} else if (ck_RxDrvSpeed() != 0) {
		mcu_state = MCU_ACTION;
	} else {
		mcu_state = MCU_NOTHING;
	}
}

uint8_t get_mcu_state(void) {
	return mcu_state;
}

void ck_emerge_sens(void) {

	if ((check_bumper() & SENS_BUMPER1) == SENS_BUMPER1) {
		set_bumper1_flag();
	}
	if ((check_bumper() & SENS_BUMPER2) == SENS_BUMPER2) {
		set_bumper2_flag();
	}
	if (ck_emgncy_key() != 0) {
		set_emergekey_flag();
	}
}

void ck_sensor1_state(void) {

	sensor1 = sensor1 & SENSOR1_MASK;

	// check manual turn brake off
	if (check_turn_brake() != 0) {
		sensor1 |= SENSOR1_TURN_BRAKE;
	} else {
		sensor1 &= ~SENSOR1_TURN_BRAKE;
	}

	// check manual brake off
	if (check_wheel_brake() != 0) {
		sensor1 |= SENSOR1_MANU_BRAKE;
	} else {
		sensor1 &= ~SENSOR1_MANU_BRAKE;
	}

	// emergency key
	if (get_emergekey_flag() != 0) {
		sensor1 |= SENSOR1_EMGNY_BTTRN;
	} else {
		sensor1 &= ~SENSOR1_EMGNY_BTTRN;
	}

	// bumper1
	if (get_bumper1_flag() != 0) {
		sensor1 |= SENSOR1_BUMPER_FRNT;
	} else {
		sensor1 &= ~SENSOR1_BUMPER_FRNT;
	}

	// bumper2
	if (get_bumper2_flag() != 0) {
		sensor1 |= SENSOR1_BUMPER_BACK;
	} else {
		sensor1 &= ~SENSOR1_BUMPER_BACK;
	}

	// lift limit lower sensor
	if ((ck_lift_limit() & LIFT_LIMIT_LOWER) == LIFT_LIMIT_LOWER) {
		sensor1 |= SENSOR1_LIMIT_LOWER;
	} else {
		sensor1 &= ~SENSOR1_LIMIT_LOWER;
	}

	// lift limit upper sensor
	if ((ck_lift_limit() & LIFT_LIMIT_UPPER) == LIFT_LIMIT_UPPER) {
		sensor1 |= SENSOR1_LIMIT_UPPER;
	} else {
		sensor1 &= ~SENSOR1_LIMIT_UPPER;
	}
}

uint8_t get_sensor1_state(void) {
	return sensor1;
}

/* モーターの電源制御関数群*/

void power_off(void){
	wheel_relay_off();	// WHEEL MOTOR POWER OFF
	turn_relay_off();	// TURN MOTOR POWER OFF
	lift_relay_off();	// LIFT MOTOR POWER OFF
}

void set_wake_up_flag(void) {
	wake_up_flag = 1;
}

void reset_wake_up_flag(void) {
	wake_up_flag = 0;
}

uint8_t get_wake_up_flag(void) {
	return wake_up_flag;
}

void set_motor_active(void) {
	motor_active_flg = 1;
}

void reset_motor_active(void) {
	motor_active_flg = 0;
}

uint8_t ck_motor_active(void) {
	return motor_active_flg;
}

enum wakeup_sequence {
	Wheel_Motor,
	Add_Relay1,
	Add_Relay2,
	Lift_Motor,
	Turn_Motor,
	Alm_Reset,
	Wait_Alm_Reset,
	WakeUp_Comp,
	WakeUp_End
};

// 起動時及び充電後のモーター立ち上げ時に立ち上げをコントロールする関数
// それぞれのモーターを1秒間隔で起動する
// 起動時、モータードライバエラーが発生していればそれを解除し、起動シーケンスを完了させる
void motor_wakeup_cntrl(void) {

	static uint8_t wake_up_state = Turn_Motor;
	static uint8_t reset_control = 0;

	switch(wake_up_state) {

	case Turn_Motor:
		if (check_timer(CNT_MOTOR_WAKEUP) == TIME_UP) {
			turn_relay_on();
			wake_up_state = Lift_Motor;
			set_utimer(CNT_MOTOR_WAKEUP, MOTOR_WAKEUP_TIMER);
		}
		break;

	case Lift_Motor:
		if (check_timer(CNT_MOTOR_WAKEUP) == TIME_UP) {
			lift_relay_on();
			wake_up_state = Wheel_Motor;
			set_utimer(CNT_MOTOR_WAKEUP, MOTOR_WAKEUP_TIMER);
		}
		break;

	case Wheel_Motor:
		if (check_timer(CNT_MOTOR_WAKEUP) == TIME_UP) {
			wheel_relay_on();
			wake_up_state = Add_Relay1;
			set_utimer(CNT_MOTOR_WAKEUP, MOTOR_WAKEUP_TIMER);
		}
		break;

	case Add_Relay1:
		if (check_timer(CNT_MOTOR_WAKEUP) == TIME_UP) {
			add_relay_off();
			wake_up_state = Add_Relay2;
			set_utimer(CNT_MOTOR_WAKEUP, ONE_SHOT_TIMER);
		}
		break;

	case Add_Relay2:
		if (check_timer(CNT_MOTOR_WAKEUP) == TIME_UP) {
			add_relay_on();
			wake_up_state = Alm_Reset;
			set_utimer(CNT_MOTOR_WAKEUP, MOTOR_WAKEUP_TIMER);
		}

		break;

	case Alm_Reset:
		if ((ck_wheel1_motor_alarm() != 0) || (ck_wheel2_motor_alarm() != 0)) {
			reset_control |= MOTOR1_ALARM;
			reset_control |= MOTOR2_ALARM;
			reset_motor_alarm_auto();
		}
		if ((ck_lift_motor1_alarm() != 0) || (ck_lift_motor2_alarm() != 0)) {
			reset_control |= MOTOR_LIFT_ALARM;
			lift_alarm_reset();
		}
		if (ck_turn_motor_alarm() != 0) {
			reset_control |= MOTOR_TURN_ALARM;
			turn_alarm_reset();
		}

		if (reset_control == 0) {
			wake_up_state = WakeUp_Comp;
		} else {
			wake_up_state = Wait_Alm_Reset;
			set_utimer(CNT_MOTOR_WAKEUP, RESET_WAIT_TIMER);
		}
		break;

	case Wait_Alm_Reset:
		if ((reset_control & MOTOR1_ALARM) || (reset_control & MOTOR2_ALARM)) {
			reset_motor_alarm_auto();
			if (ck_wheel1_motor_alarm() == 0) {
				reset_control &= ~MOTOR1_ALARM;
			}
			if (ck_wheel2_motor_alarm() == 0) {
				reset_control &= ~MOTOR2_ALARM;
			}
		}
		if (reset_control & MOTOR_LIFT_ALARM) {
			lift_alarm_recover();
			if ((ck_lift_motor1_alarm() == 0) && (ck_lift_motor2_alarm() == 0)) {
				reset_control &= ~MOTOR_LIFT_ALARM;
			}
		}
		if (reset_control & MOTOR_TURN_ALARM) {
			turn_alarm_recover();
			if (ck_turn_motor_alarm() == 0) {
				reset_control &= ~MOTOR_TURN_ALARM;
			}
		}
		if ((reset_control == 0) || (check_timer(CNT_MOTOR_WAKEUP) == TIME_UP)) {
			// アラームリセットが全て終わるか時間切れとなった場合に抜ける
			reset_control = 0;
			wake_up_state = WakeUp_Comp;
		}
		break;

	case WakeUp_Comp:
		set_motor_active();
		reset_wake_up_flag();
		wake_up_state = WakeUp_End;
		break;

	case WakeUp_End:
		if (get_wake_up_flag() != 0) {
			wake_up_state = Wheel_Motor;
			set_utimer(CNT_MOTOR_WAKEUP, MOTOR_WAKEUP_TIMER);
		}
		break;
	}
}
