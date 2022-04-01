/*
 * charge.h
 *
 *  Created on: 2019/11/27
 *      Author: Takumi
 */
#ifndef CHARGE_H_
#define CHARGE_H_

//////// Charge state /////////////////
enum Charge_State_Info {
	Charge_Info_Idle,
	Charge_Info_Wait,
	Charge_Info_Ready,
	Charge_Info_Charging
};

#define BATT_LOW       		   	0x00
#define BATT_MAX       		   	0x01

#define ERROR_KEEP_TIME			2000
/* function ----------------------------------------------------------*/
void charge_proc_init(void);
void charge_proc(void);

uint8_t ck_charge_state(void);
uint8_t get_charge_state_info(void);

#endif
