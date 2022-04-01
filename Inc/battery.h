/*
 * battery.h
 *
 *  Created on: 2019/12/04
 *      Author: katte
 */

#ifndef BATTERY_H_
#define BATTERY_H_

/* function ----------------------------------------------------------*/
void battery_monitor(void);
void battery_init(void);
void set_i2c2_init(void);
void reset_i2c2_init(void);
uint8_t get_i2c2_init(void);
uint8_t get_i2c2_state(void);
void CAN_filter_setting(void);
uint16_t get_battery1_capcity(void);
uint16_t get_battery2_capcity(void);
uint8_t ck_battery_full(void);
uint8_t ck_battery_overcharged(void);
uint8_t ck_battery_chargestop(void);
uint8_t ck_battery_overtmp(void);

/* define ------------------------------------------------------------*/
enum Bat_i2c_state {
	I2C_Normal,
	I2C_Error,
	I2C_Many_Error,
	I2C_Busy,
	I2C_Reset,
	I2C_Recover,
	I2C_Init
};

#define NEC_BATTERY		0
#define MURATA_BATTERY	1

#define BATTERY_STATUS_SIZE		0x02

#define	BATTERY_DEVICE_ADD1		0x14				// device address 0x0a << 1
#define	BATTERY_DEVICE_ADD2		0x16				// device address 0x0b << 1
#define	BATTERY_DEVICE_ADD3		0x1A				// device address 0x0d << 1
#define	BATTERY_DEVICE_ADD4		0x1C				// device address 0x0e << 1

#define BATTERY_TMP_CMD			0x08				// return temperature of battery(0.1K)
#define BATTERY_VOLTAGE_CMD		0x09				// return voltage of battery(1mV)
#define BATTERY_CURRENT_CMD		0x0A				// return current of battery(1mA)
#define BATTERY_AVR_CURRENT_CMD	0x0B				// return average current while 1 minutes of battery(1mA)
#define BATTERY_REMAIN_CAP_CMD	0x0D				// return remained capacity of battery
#define BATTERY_CAP_ABS_CMD		0x0E				// return remained capacity of battery(Absolute of design battery)
#define BATTERY_REMAIN_TIM_CMD 	0x11				// return Run Time To Empty
#define BATTERY_REMAIN_AVETIME_CMD 0x12				// return Average Time To Empty
#define	BATTERY_STATUS_CMD		0x16				// return battery status

#define BATTERY_OVER_CHARGE_AlARM		0x8000
#define BATTERY_TERMINATE_CHARGE_ALARM	0x4000
#define BATTERY_OVER_TEMP_ALARM			0x1000

#define BATTERY_DISCHARGE				0x0040
#define BATTERY_FULL_CHARGE				0x0020

#define BATTERY_PARTNO_CMD		0xF2

#endif /* BATTERY_H_ */
