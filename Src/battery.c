/*
 * battery.c
 *
 *  Created on: 2019/12/04
 *      Author: katte
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "conf.h"
#include "battery.h"
#include "Cntrl_Proc.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_can.h"
#include <string.h>

/* Private function ----------------------------------------------------------*/
uint8_t cmd_select(uint8_t);
uint8_t adr_select(uint8_t);
void value_set(uint8_t, uint8_t, uint16_t);

/* External variables --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
enum bat_num {
	battery1,
	battery2,
	battery3,
	battery4,
	battery_num
};

enum bat_cmd {
	Capacity,
	Status,
	Cmd_Last
};

#define MAX_CAPACITY 100
#define ERROR_TRESH 4	// 4回連続エラーになったら、復活のシーケンスへ以降する

// can filter
#define CAN_FILTER_ID	0x100		// CANのフィルターID
#define CAN_FILTER_MASK	0xFFE		// CANのフィルターマスク
// data index
#define FAIL_STATUS1			0
#define	LEADER_BATTERY_STATUS	1
#define	RSOC_MIN				3
#define FAIL_STATUS2			6
// data bit
#define OVER_CHARGE_PROTECT		0x4
#define OVER_CURRENT_CHARGE_45A	0x8
#define FULLY_CHARGE			0x40
#define OVER_CURRENT_CHARGE_65A	0x2
#define OVER_TEMP_CHARGE		0x4

/* Private variables ---------------------------------------------------------*/
static uint16_t battery_info[Cmd_Last][battery_num];
static uint8_t i2c2_init_flag = 0;
static uint8_t i2c2_state = I2C_Normal;
static uint8_t error_count = 0;
static uint8_t	RxData_id100[8] = {};
static uint8_t	RxData_id101[8] = {};

void battery_init(void) {
	for (int i = 0; i < battery_num; i++) {
		battery_info[Capacity][i] = MAX_CAPACITY;
	}
	RxData_id100[RSOC_MIN] = MAX_CAPACITY;
}

void battery_busy_recovery(void) {

	if ((get_i2c2_state() == I2C_Busy) || (get_i2c2_state() == I2C_Many_Error)) {
		// I2Cにリセットを掛ける
		__HAL_RCC_I2C2_FORCE_RESET();
		i2c2_state = I2C_Reset;
	} else if (get_i2c2_state() == I2C_Reset) {
		// リセットを解除する
		__HAL_RCC_I2C2_RELEASE_RESET();
		i2c2_state = I2C_Recover;
	} else if (get_i2c2_state() == I2C_Recover) {
		// I2C2のイニシャル処理を行う
		set_i2c2_init();
		i2c2_state = I2C_Init;
	} else if (get_i2c2_state() == I2C_Init) {
		// 通信再開まで待つ
		i2c2_state = I2C_Normal;
	}
}

void battery_val_read(void) {

	static uint8_t cmd_set = Capacity;
	static uint8_t bat_set = battery1;
	uint16_t battery_status_tmp = 0;
	uint8_t bat_adr = 0;
	uint8_t cmd = 0;
	uint8_t	get_status[BATTERY_STATUS_SIZE];
	uint8_t i;
	HAL_StatusTypeDef Result = HAL_OK;

	for (i = 0; i < BATTERY_STATUS_SIZE; i++) {
		get_status[i] = 0;
	}

	cmd = cmd_select(cmd_set);
	bat_adr = adr_select(bat_set);

	Result = read_battery_status(bat_adr, cmd, &get_status[0]);

	if (Result == HAL_OK) {
		battery_status_tmp = ((get_status[1] << 8) | (get_status[0]));
		value_set(bat_adr, cmd, battery_status_tmp);
		i2c2_state = I2C_Normal;
		error_count = 0;
	} else if (Result == HAL_BUSY) {
		i2c2_state = I2C_Busy;
		error_count = 0;
	} else if (Result == HAL_ERROR) {
		i2c2_state = I2C_Error;
		error_count++;
		if (error_count >= ERROR_TRESH) {
			error_count = 0;
			i2c2_state = I2C_Many_Error;
		}
	}
	bat_set++;
	if (bat_set >= battery_num) {
		bat_set = battery1;
		cmd_set++;
		if (cmd_set >= Cmd_Last) {
			cmd_set = Capacity;
		}
	}
}

void battery_monitor(void) {
#if BATTERY_TYPE == NEC_BATTERY
	if (get_i2c2_state() >= I2C_Many_Error) {
		battery_busy_recovery();
	} else {
		battery_val_read();
	}
#endif
}

uint8_t cmd_select(uint8_t select) {

	uint8_t cmd = 0;

	switch (select) {
	case Capacity:
		cmd = BATTERY_REMAIN_CAP_CMD;
		break;

	case Status:
		cmd = BATTERY_STATUS_CMD;
		break;
	}
	return cmd;
}

uint8_t adr_select(uint8_t select) {

	uint8_t adr = 0;

	if (select == battery1) {
		adr = BATTERY_DEVICE_ADD1;
	} else if (select == battery2) {
		adr = BATTERY_DEVICE_ADD2;
	} else if (select == battery3) {
		adr = BATTERY_DEVICE_ADD3;
	} else if (select == battery4) {
		adr = BATTERY_DEVICE_ADD4;
	}
	return adr;
}

void value_set(uint8_t adr, uint8_t cmd, uint16_t value) {

	uint8_t bat = battery1;
	uint8_t status = Capacity;

	if (adr == BATTERY_DEVICE_ADD1) {
		bat = battery1;
	} else if (adr == BATTERY_DEVICE_ADD2) {
		bat = battery2;
	} else if (adr == BATTERY_DEVICE_ADD3) {
		bat = battery3;
	} else if (adr == BATTERY_DEVICE_ADD4) {
		bat = battery4;
	}

	if (cmd == BATTERY_REMAIN_CAP_CMD) {
		status = Capacity;
	} else if (cmd == BATTERY_STATUS_CMD) {
		status = Status;
	}
	battery_info[status][bat] = value;
}

void set_i2c2_init(void) {
	i2c2_init_flag = 1;
}

void reset_i2c2_init(void) {
	i2c2_init_flag = 0;
}

uint8_t get_i2c2_init(void) {
	return i2c2_init_flag;
}

uint8_t get_i2c2_state(void) {
	return i2c2_state;
}

uint16_t get_battery1_capcity(void) {
#if BATTERY_TYPE == NEC_BATTERY
	uint16_t ret;
	ret = (battery_info[Capacity][battery1] + battery_info[Capacity][battery3]) / 2;

	return ret;
#elif BATTERY_TYPE == MURATA_BATTERY
	return RxData_id100[RSOC_MIN];
#endif
}

uint16_t get_battery2_capcity(void) {
#if BATTERY_TYPE == NEC_BATTERY
	uint16_t ret;
	ret = (battery_info[Capacity][battery2] + battery_info[Capacity][battery4]) / 2;

	return ret;
#elif BATTERY_TYPE == MURATA_BATTERY
	return RxData_id100[RSOC_MIN];
#endif
}

uint8_t ck_battery_full(void) {
#if BATTERY_TYPE == NEC_BATTERY
	uint8_t ret = 0;

	if ((battery_info[Status][battery1] & BATTERY_FULL_CHARGE) &&
		(battery_info[Status][battery2] & BATTERY_FULL_CHARGE) &&
		(battery_info[Status][battery3] & BATTERY_FULL_CHARGE) &&
		(battery_info[Status][battery4] & BATTERY_FULL_CHARGE)) {
		ret = 1;
	}
	return ret;
#elif BATTERY_TYPE == MURATA_BATTERY
	return RxData_id100[FAIL_STATUS1] & FULLY_CHARGE;
#endif
}

uint8_t ck_battery_overcharged(void) {
#if BATTERY_TYPE == NEC_BATTERY
	uint8_t ret = 0;

	if ((battery_info[Status][battery1] & BATTERY_OVER_CHARGE_AlARM) ||
		(battery_info[Status][battery2] & BATTERY_OVER_CHARGE_AlARM) ||
		(battery_info[Status][battery3] & BATTERY_OVER_CHARGE_AlARM) ||
		(battery_info[Status][battery4] & BATTERY_OVER_CHARGE_AlARM)) {
		ret = 1;
	}
	return ret;
#elif BATTERY_TYPE == MURATA_BATTERY
	return RxData_id100[FAIL_STATUS1] & OVER_CHARGE_PROTECT;
#endif
}

uint8_t ck_battery_chargestop(void) {
#if BATTERY_TYPE == NEC_BATTERY
	uint8_t ret = 0;

	if ((battery_info[Status][battery1] & BATTERY_TERMINATE_CHARGE_ALARM) ||
		(battery_info[Status][battery2] & BATTERY_TERMINATE_CHARGE_ALARM) ||
		(battery_info[Status][battery3] & BATTERY_TERMINATE_CHARGE_ALARM) ||
		(battery_info[Status][battery4] & BATTERY_TERMINATE_CHARGE_ALARM)) {
		ret = 1;
	}
	return ret;
#elif BATTERY_TYPE == MURATA_BATTERY
	uint8_t ret = 0;
	if ((RxData_id100[FAIL_STATUS1] & OVER_CURRENT_CHARGE_45A) ||
		(RxData_id101[FAIL_STATUS2] & OVER_CURRENT_CHARGE_65A)) {
		ret = 1;
	}
	return ret;
#endif
}

uint8_t ck_battery_overtmp(void) {
#if BATTERY_TYPE == NEC_BATTERY
	uint8_t ret = 0;

	if ((battery_info[Status][battery1] & BATTERY_OVER_TEMP_ALARM) ||
		(battery_info[Status][battery2] & BATTERY_OVER_TEMP_ALARM) ||
		(battery_info[Status][battery3] & BATTERY_OVER_TEMP_ALARM) ||
		(battery_info[Status][battery4] & BATTERY_OVER_TEMP_ALARM)) {
		ret = 1;
	}
	return ret;
#elif BATTERY_TYPE == MURATA_BATTERY
	return RxData_id101[FAIL_STATUS2] & OVER_TEMP_CHARGE;
#endif
}
