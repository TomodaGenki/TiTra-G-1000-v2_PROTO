/*
 * charge.c
 * 
 *
 *  Created on: 2019/11/27
 *      Author: Takumi
 */


/* Includes ------------------------------*/
#include "main.h"
#include "Cntrl_Proc.h"
#include "nuc.h"
#include "charge.h"
#include "task_charger_link.h"
#include "conf.h"
#include "battery.h"
#include "wheel.h"

// ------------ define --------------------
// response for NUC
#define CHG_STATE_NORMAL			0x00	// charge state:"000"(Do not charge state)
#define CHG_STATE_START_READY		0x10	// charge state:"001"(charge start ready)
#define CHG_STATE_CHARGING			0x20	// charge state:"010"(charging)
#define CHG_STATE_CHARGE_COMP		0x30	// charge state:"011"(charge complete)
#define CHG_STATE_OVERCHARGE		0x40	// charge state:"100"(over charge)
#define CHG_STATE_STOP_READY		0x50	// charge state:"101"(charge stop ready)
#define CHG_STATE_STOP				0x60	// charge state:"110"(charge suspending)
#define CHG_STATE_ERROR				0x70	// charge state:"111"(charge error)

// charge wait time
#define RELAY_OFF_TIMER				1000
#define CHARGE_OFF_TIMER			1000

// charging state
enum Charge_State {
	Charge_Idle,
	Charge_Ready,
	Charge_Start_Wait,
	Charging,
	Charge_Stop_Wait,
	Charge_Stop,
	Charge_Stop_Keep,
	Manual_Charge_Ready,
	Manual_Charging,
	Manual_Charge_Stop,
	Charge_Error,
	Charge_Error_Wait
};

/* Private variables ---------------------------------------------------------*/
static uint8_t nuc_rsp_comm = CHG_STATE_NORMAL;
static uint8_t chg_st = Charge_Idle;
static uint8_t charge_st_info = Charge_Info_Idle;

/* Private function ----------------------------------------------------------*/
void charge_relay_on(void) {
	HAL_GPIO_WritePin(O_ChargeRelay_GPIO_Port, O_ChargeRelay_Pin, GPIO_PIN_SET);
}

void charge_relay_off(void) {
	HAL_GPIO_WritePin(O_ChargeRelay_GPIO_Port, O_ChargeRelay_Pin, GPIO_PIN_RESET);
}

void set_charge_state_info(uint8_t state) {
	charge_st_info = state;
}

uint8_t get_charge_state_info(void) {
	return charge_st_info;
}

void change_charge_state(uint8_t state) {
	chg_st = state;
}

uint8_t get_chg_state(void){
	return chg_st;
}

void set_nuc_response(uint8_t response) {
	nuc_rsp_comm = response;
}

uint8_t ck_charge_state(void) {
	return nuc_rsp_comm;
}

uint8_t ck_battry(void){

	uint8_t batt_jdg = BATT_LOW;

	if (ck_battery_full() != 0) {		// battery full charge
		batt_jdg = BATT_MAX;
	} else if (ck_battery_overcharged() != 0) {	// over charging
		batt_jdg = BATT_MAX;
	} else if (ck_battery_chargestop() != 0) {	// charge stop alarm from battery
		batt_jdg = BATT_MAX;
	} else if (ck_battery_overtmp() != 0) {			// charge stop alarm for battery for over temperature
		batt_jdg = BATT_MAX;
	}
	return batt_jdg;
}

void charge_proc_init(void) {
	clr_irda_comdata();
	charge_relay_off();
}

void charge_proc(void) {

	uint8_t state = get_chg_state();

	switch(state) {

	case Charge_Idle:
		clr_irda_comdata();
		if ((ck_charge_req() == REQUEST_CHARGE) && (ck_emerg_stop() == NOT_EMERGENCY)) {
			// NUCからの要求による自動充電
			power_off();
			reset_motor_active();
			set_nuc_response(CHG_STATE_START_READY);
			set_utimer(CNT_GO6, RELAY_OFF_TIMER);
			set_utimer(CDT_CHRGROUT, CNT_CHRGROUT);
			change_charge_state(Charge_Ready);
			set_charge_state_info(Charge_Info_Wait);
		} else if ((ck_power_relay_req() == POWER_RELAY) && (ck_emerg_stop() == NOT_EMERGENCY)) {
			power_off();								// motor relay off
			reset_motor_active();
			set_utimer(CNT_GO6, RELAY_OFF_TIMER);
			change_charge_state(Manual_Charge_Ready);
			set_charge_state_info(Charge_Info_Ready);
		}
		break;

	case Charge_Ready:
		if (ck_charge_req() == STOP_CHARGE) {
			set_utimer(CNT_GO6, CHARGE_OFF_TIMER);
			set_utimer(CDT_CHRGROUT, CNT_CHRGROUT);
			change_charge_state(Charge_Stop_Wait);
		} else if ((check_timer(CDT_CHRGROUT) == TIME_UP) || (ck_irda_error() != 0) || (ck_emerg_stop() != NOT_EMERGENCY)) {
			set_nuc_response(CHG_STATE_ERROR);
			set_utimer(CNT_GO6, ERROR_KEEP_TIME);
			change_charge_state(Charge_Error);
		} else if (check_timer(CNT_GO6) == TIME_UP) {
			set_irda_comm_req();
			if (ck_irda_comm() != 0) {
				charge_relay_on();
				set_irda_charge_req();
				set_utimer(CDT_CHRGROUT, CNT_CHRGROUT);
				change_charge_state(Charge_Start_Wait);
				set_charge_state_info(Charge_Info_Ready);
			}
		}
		break;

	case Charge_Start_Wait:
		if ((check_timer(CDT_CHRGROUT) == TIME_UP) || (ck_irda_error() != 0) || (ck_irda_comm() == 0) || (ck_emerg_stop() != NOT_EMERGENCY)) {
			set_nuc_response(CHG_STATE_ERROR);
			set_utimer(CNT_GO6, ERROR_KEEP_TIME);
			change_charge_state(Charge_Error);
		} else if (ck_charge_req() == STOP_CHARGE) {
			set_utimer(CNT_GO6, CHARGE_OFF_TIMER);
			set_utimer(CDT_CHRGROUT, CNT_CHRGROUT);
			change_charge_state(Charge_Stop_Wait);
		} else if (ck_irda_charging() != 0) {
			set_nuc_response(CHG_STATE_CHARGING);
			change_charge_state(Charging);
			set_charge_state_info(Charge_Info_Charging);
		}
		break;

	case Charging:
		if ((ck_irda_error() != 0) || (ck_irda_comm() == 0) || (ck_emerg_stop() != NOT_EMERGENCY)) {
			set_nuc_response(CHG_STATE_ERROR);
			set_utimer(CNT_GO6, ERROR_KEEP_TIME);
			change_charge_state(Charge_Error);
		} else if ((ck_irda_charging() == 0) || (ck_charge_req() == STOP_CHARGE) || (ck_battry() == BATT_MAX)) {
			reset_irda_charge_req();
			set_nuc_response(CHG_STATE_CHARGE_COMP);
			set_utimer(CNT_GO6, CHARGE_OFF_TIMER);
			set_utimer(CDT_CHRGROUT, CNT_CHRGROUT);
			change_charge_state(Charge_Stop_Wait);
			set_charge_state_info(Charge_Info_Ready);
		}
		break;

	case Charge_Stop_Wait:
		if (check_timer(CNT_GO6) == TIME_UP) {
			set_nuc_response(CHG_STATE_STOP_READY);
			if (ck_irda_charging() == 0) {
				reset_irda_comm_req();
				charge_relay_off();
				set_wake_up_flag();
				set_charge_state_info(Charge_Info_Wait);
				change_charge_state(Charge_Stop);
			}
		}
		break;

	case Charge_Stop:
		if (ck_motor_active() != 0) {
			set_nuc_response(CHG_STATE_STOP);
			set_utimer(CNT_GO6, RELAY_OFF_TIMER);
			change_charge_state(Charge_Stop_Keep);
			set_charge_state_info(Charge_Info_Idle);
		}
		break;

	case Charge_Stop_Keep:
		if ((ck_RxDrvSpeed() != 0) || (check_timer(CNT_GO6) == TIME_UP)) {
			set_nuc_response(CHG_STATE_NORMAL);
			change_charge_state(Charge_Idle);
		}
		break;

	case Manual_Charge_Ready:
		if (check_timer(CNT_GO6) == TIME_UP){
			charge_relay_on();					// charge relay on
			change_charge_state(Manual_Charging);
			set_charge_state_info(Charge_Info_Charging);
		}
		break;

	case Manual_Charging:
		if ((ck_power_relay_req() != POWER_RELAY) || (ck_emerg_stop() != NOT_EMERGENCY)) {
			charge_relay_off();
			set_utimer(CNT_GO6, RELAY_OFF_TIMER);
			set_wake_up_flag();
			change_charge_state(Manual_Charge_Stop);
			set_charge_state_info(Charge_Info_Ready);
		}
		break;

	case Manual_Charge_Stop:
		if (check_timer(CNT_GO6) == TIME_UP) {
			if (ck_motor_active() != 0) {
				change_charge_state(Charge_Idle);
				set_charge_state_info(Charge_Info_Idle);
			}
		}
		break;

	case Charge_Error:
		set_charge_state_info(Charge_Info_Wait);
		reset_irda_charge_req();								// charge stop request
		if (ck_irda_error() == 0) {
			// ロボット側起因により充電エラーとなった場合、充電器に充電エラーを送信する
			set_irda_error_req();
		}
		if (ck_irda_charging() == 0) {
			// 充電が終了していれば、充電エラーを解除するための準備を行う
			charge_relay_off();
			reset_irda_comm_req();
			change_charge_state(Charge_Error_Wait);
			set_utimer(CNT_GO6, ERROR_KEEP_TIME);
		}
		break;

	case Charge_Error_Wait:
		// 一定時間経過後、充電エラーを解除する
		if (check_timer(CNT_GO6) == TIME_UP) {
			reset_irda_error_req();
			set_nuc_response(CHG_STATE_STOP_READY);
			set_utimer(CNT_GO6, CHARGE_OFF_TIMER);
			change_charge_state(Charge_Stop);
			set_wake_up_flag();
		}
		break;
	}
}
