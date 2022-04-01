/*
 * led.c
 * 
 *
 *  Created on: 2021/04/05
 *      Author: Takumi
 */


/* Includes ------------------------------*/
#include "main.h"
#include "Cntrl_Proc.h"
#include "nuc.h"
#include "charge.h"
#include "conf.h"
#include "led.h"

// ------------ define --------------------
#define LED_OFF        		0x00
#define LED_ON         		0x05
#define RIGHT_LED			0x01
#define LEFT_LED			0x02

// ------------ variables -----------------

// ---- for work ---------------------

void Left_Red_Led(uint8_t Led) {
	if (Led == LedOn) {
		HAL_GPIO_WritePin(O_RedLed_L_GPIO_Port, O_RedLed_L_Pin, SET);
	} else {
		HAL_GPIO_WritePin(O_RedLed_L_GPIO_Port, O_RedLed_L_Pin, RESET);
	}
}

void Right_Red_Led(uint8_t Led) {
	if (Led == LedOn) {
		HAL_GPIO_WritePin(O_RedLed_R_GPIO_Port, O_RedLed_R_Pin, SET);
	} else {
		HAL_GPIO_WritePin(O_RedLed_R_GPIO_Port, O_RedLed_R_Pin, RESET);
	}
}

void Left_Green_Led(uint8_t Led) {
	if (Led == LedOn) {
		HAL_GPIO_WritePin(O_GreenLed_L_GPIO_Port, O_GreenLed_L_Pin, SET);
	} else {
		HAL_GPIO_WritePin(O_GreenLed_L_GPIO_Port, O_GreenLed_L_Pin, RESET);
	}
}

void Right_Green_Led(uint8_t Led) {
	if (Led == LedOn) {
		HAL_GPIO_WritePin(O_GreenLed_R_GPIO_Port, O_GreenLed_R_Pin, SET);
	} else {
		HAL_GPIO_WritePin(O_GreenLed_R_GPIO_Port, O_GreenLed_R_Pin, RESET);
	}
}

void Left_Blue_Led(uint8_t Led) {
	if (Led == LedOn) {
		HAL_GPIO_WritePin(O_BlueLed_L_GPIO_Port, O_BlueLed_L_Pin, SET);
	} else {
		HAL_GPIO_WritePin(O_BlueLed_L_GPIO_Port, O_BlueLed_L_Pin, RESET);
	}
}

void Right_Blue_Led(uint8_t Led) {
	if (Led == LedOn) {
		HAL_GPIO_WritePin(O_BlueLed_R_GPIO_Port, O_BlueLed_R_Pin, SET);
	} else {
		HAL_GPIO_WritePin(O_BlueLed_R_GPIO_Port, O_BlueLed_R_Pin, RESET);
	}
}

void Table_Led(uint8_t Led) {
	if (Led == LedOn) {
		HAL_GPIO_WritePin(O_TableLed_GPIO_Port, O_TableLed_Pin, SET);
	} else {
		HAL_GPIO_WritePin(O_TableLed_GPIO_Port, O_TableLed_Pin, RESET);
	}
}

/*
 *  LED Inital(OFF)
 */
void LED_GPIO_INIT(void)
{
	// LED-Green
	Left_Green_Led(LedOff);
	Right_Green_Led(LedOff);

	// LED-Red
	Left_Red_Led(LedOff);
	Right_Red_Led(LedOff);

	// LED-Blue
	Left_Blue_Led(LedOff);
	Right_Blue_Led(LedOff);

	Table_Led(LedOff);
}


void led_color_cntrl(uint8_t dir, uint8_t color)
{
	/*  LEDの色を制御する
	 *  dir:制御対象のLED(RIGHT_LED or LEFT_LED)
	 *  color:光らせる色をビットで指定 複数指定も可(LED_COLOR_GREEN or LED_COLOR_RED or
	 *  	LED_COLOR_BLUE or LED_TURN_OFF)
	 *  pwm_limtを大きくするとDuty比が低下し輝度ダウン  現在は最大輝度で固定
	 */

	static uint8_t pwm_cnt = 0;
	uint8_t pwm_limt = 0;

	if(pwm_cnt >= pwm_limt){
		pwm_cnt = 0;
		// A-LED-Green
		if ((color & LED_COLOR_GREEN) == LED_COLOR_GREEN){
			if(dir == RIGHT_LED){
				Right_Green_Led(LedOn);
			}
			else if(dir == LEFT_LED){
				Left_Green_Led(LedOn);
			}
		} else {
			if(dir == RIGHT_LED){
				Right_Green_Led(LedOff);
			}
			else if(dir == LEFT_LED){
				Left_Green_Led(LedOff);
			}
		}

		// A-LED-Red
		if ((color & LED_COLOR_RED) == LED_COLOR_RED){
			if(dir == RIGHT_LED){
				Right_Red_Led(LedOn);
			}
			else if(dir == LEFT_LED){
				Left_Red_Led(LedOn);
			}
		} else {
			if(dir == RIGHT_LED){
				Right_Red_Led(LedOff);
			}
			else if(dir == LEFT_LED){
				Left_Red_Led(LedOff);
			}
		}

		// A-LED-Blue
		if ((color & LED_COLOR_BLUE) == LED_COLOR_BLUE){
			if(dir == RIGHT_LED){
				Right_Blue_Led(LedOn);
			}
			else if(dir == LEFT_LED){
				Left_Blue_Led(LedOn);
			}
		} else {
			if(dir == RIGHT_LED){
				Right_Blue_Led(LedOff);
			}
			else if(dir == LEFT_LED){
				Left_Blue_Led(LedOff);
			}
		}
	}
	else{
		LED_GPIO_INIT();
		pwm_cnt++;
	}
}

void led_brink_cntrl(uint8_t dir, uint8_t ptrn, uint16_t timer, uint16_t period, uint8_t color)
{
	/*	点滅周期に応じてLEDをON/OFF
	 *  dir:制御対象のLED(RIGHT_LED or LEFT_LED)
	 *  ptrn:点滅パターン(LED_BLINK_QUICK or LED_BLINK_MIDIUM or LED_BLINK_SLOW or LED_ON or LED_OFF)
	 *  period:点滅周期(整数)
	 *  color:光らせる色をビットで指定 複数指定も可(LED_COLOR_GREEN or LED_COLOR_RED or
	 *  	LED_COLOR_BLUE or LED_TURN_OFF)
	 */
	if (ptrn == LED_ON){
		led_color_cntrl(dir, color);
	}
	else if(ptrn == LED_OFF){
		led_color_cntrl(dir, LED_TURN_OFF);
	}
	else if (timer > period / 2){
		led_color_cntrl(dir, color);
	}
	else{
		led_color_cntrl(dir, LED_TURN_OFF);
	}
}

void brake_release_led(uint8_t prev, uint8_t *color, uint8_t *pattern) {
#if USE_BRAKE_RELEASE
	// 手動スイッチによりブレーキ解放されている場合に黄色を表示する
	if (check_wheel_brake() != 0) {
		if ((ck_emerg_stop() == EMERGENCY_CT0) || (ck_emerg_stop() == EMERGENCY_CT1)||(ck_emerg_stop() == EMERGENCY_CT2)) {
			// ブレーキリリース状態でのエラー時、黄色と赤の交互点滅
			if (prev == LED_COLOR_YELLOW) {
				*color = LED_COLOR_RED;
			} else {
				*color = LED_COLOR_YELLOW;
			}
			*pattern = LED_BLINK_QUICK;
		} else if ((ck_emerg_stop() == EMERGENCY_CT3) || (ck_emerg_stop() == EMERGENCY_CT4)) {
			// ブレーキリリース状態でのエラー時、黄色と赤の交互点滅
			if (prev == LED_COLOR_YELLOW) {
				*color = LED_COLOR_RED;
			} else {
				*color = LED_COLOR_YELLOW;
			}
			*pattern = LED_BLINK_SLOW;
		} else {
			// エラーではない時は黄色点灯
			*color = LED_COLOR_YELLOW;
			*pattern = LED_ON;
		}
	}
#endif
}

uint8_t chk_led(void)
{
	uint8_t outbuf = LED_TURN_OFF;

	if ((ck_emerg_stop() == EMERGENCY_CT0) || (ck_emerg_stop() == EMERGENCY_CT1)||(ck_emerg_stop() == EMERGENCY_CT2)){
		outbuf = (LED_COLOR_RED + LED_BLINK_QUICK);			// (RED) Emergency stop
	} else if (get_charge_state_info() != Charge_Info_Idle) {
		if (get_charge_state_info() == Charge_Info_Ready) {
			outbuf = LED_COLOR_GREEN;						// (GREEN)
		} else if (get_charge_state_info() == Charge_Info_Charging) {
			outbuf = (LED_COLOR_GREEN + LED_BLINK_SLOW);
		}
	} else if ((ck_emerg_stop() == EMERGENCY_CT3) || (ck_emerg_stop() == EMERGENCY_CT4)) {
		outbuf = (LED_COLOR_RED + LED_BLINK_SLOW);
	} else {
		outbuf = led_get_indication();
	}
	return outbuf;
}

uint16_t update_period(uint8_t ptrn){
	/*	点滅周期をセットする
	 *  ptrn:点滅パターン(LED_BLINK_QUICK or LED_BLINK_MIDIUM or LED_BLINK_SLOW or LED_ON or LED_OFF)
	 *  color:光らせる色をビットで指定 複数指定も可(LED_COLOR_GREEN or LED_COLOR_RED or
	 *  	LED_COLOR_BLUE or LED_TURN_OFF)
	 *  戻り値：period:点滅周期(整数)
	 */
	uint16_t period = 0;
	switch(ptrn){
		case LED_BLINK_QUICK:
			period = LED_BLINK_PERIOD_QUICK;
			break;
		case LED_BLINK_MIDIUM:
			period = LED_BLINK_PERIOD_MIDIUM;
			break;
		case LED_BLINK_SLOW:
			period = LED_BLINK_PERIOD_SLOW;
			break;
		case LED_ON:
			// 切り替わり時のチラツキを防ぐために非点滅時も更新周期を設定
			period = LED_BLINK_PERIOD_QUICK;
			break;
		case LED_OFF:
			// 切り替わり時のチラツキを防ぐために非点滅時も更新周期を設定
			period = LED_BLINK_PERIOD_QUICK;
			break;
	}
	return period;
}

void led_cntrol(uint8_t ptrn_right, uint8_t ptrn_left, uint8_t color_right, uint8_t color_left)
{
	/*	LEDの点滅周期を決定
	 *  ptrn_right:右LEDの点灯/点滅パターン (LED_BLINK_QUICK or LED_BLINK_MIDIUM or LED_BLINK_SLOW or LED_ON or LED_OFF)
	 *  ptrn_left:左LEDの点灯/点滅パターン (LED_BLINK_QUICK or LED_BLINK_MIDIUM or LED_BLINK_SLOW or LED_ON or LED_OFF)
	 *  color_right:光らせる色をビットで指定 複数指定も可(LED_COLOR_GREEN or LED_COLOR_RED or
	 *  	LED_COLOR_BLUE or LED_TURN_OFF)
	 *  color_left:光らせる色をビットで指定 複数指定も可(LED_COLOR_GREEN or LED_COLOR_RED or
	 *  	LED_COLOR_BLUE or LED_TURN_OFF)
	 */
	static uint16_t timer_right = 0;	// 点滅制御用タイマ
	static uint16_t timer_left = 0;
	static uint16_t period_right = 0;	// 点滅周期
	static uint16_t period_left = 0;
	static uint8_t ptrn_right_buf = 0;	// 点滅パターン
	static uint8_t ptrn_left_buf = 0;
	static uint8_t color_right_buf = 0;	// 色
	static uint8_t color_left_buf = 0;
	static uint8_t flg_update_right = 0;	// 点滅パターン更新許可フラグ
	static uint8_t flg_update_left = 0;

	// 右LED用タイマ
	if (timer_right < period_right){
		timer_right ++;
	}
	else{
		timer_right = 0;
		// 点滅周期更新フラグをセット
		flg_update_right = 1;
	}

	// 左LED用タイマ
	if (timer_left < period_left){
		timer_left ++;
	}
	else{
		timer_left = 0;
		// 点滅周期更新フラグをセット
		flg_update_left = 1;
	}

	// 左右の同期を取る
	if (ptrn_right == ptrn_left){
		if(flg_update_right == 1){
			timer_left = 0;
			flg_update_left = 1;
		}
		if(flg_update_left == 1){
			timer_right = 0;
			flg_update_right = 1;
		}
	}

	// 点滅周期と色を更新
	if(flg_update_right == 1){
		brake_release_led(color_right_buf, &color_right, &ptrn_right);
		color_right_buf = color_right;
		ptrn_right_buf = ptrn_right;
		period_right = update_period(ptrn_right_buf);
		flg_update_right = 0;
	}
	if(flg_update_left == 1){
		brake_release_led(color_left_buf, &color_left, &ptrn_left);
		color_left_buf = color_left;
		ptrn_left_buf = ptrn_left;
		period_left = update_period(ptrn_left_buf);
		flg_update_left = 0;
	}

	// LED点滅制御
	led_brink_cntrl(RIGHT_LED, ptrn_right_buf, timer_right, period_right, color_right_buf);
	led_brink_cntrl(LEFT_LED, ptrn_left_buf, timer_left, period_left, color_left_buf);
}

void led_proc(void)
{
	uint8_t led_ptrn = 0;
	uint8_t led_ptrn_right = 0;
	uint8_t led_ptrn_left = 0;
	uint8_t ctrlout3 = 0;
	uint8_t color = 0;
	uint8_t color_right = 0;
	uint8_t color_left = 0;

	// turn table LED
	uint8_t table_led = led_get_indication();
	if (table_led & LED_TABLE_FLASH) {
		Table_Led(LedOn);
	} else {
		Table_Led(LedOff);
	}

	//led command set
	ctrlout3 = chk_led();

	// select color
	color = ctrlout3 & (LED_COLOR_BLUE + LED_COLOR_GREEN + LED_COLOR_RED);

	// select blink
	switch (ctrlout3 & LED_BLINK_REQ){
		case LED_BLINK_QUICK:
			led_ptrn = LED_BLINK_QUICK;
			break;
		case LED_BLINK_MIDIUM:
			led_ptrn = LED_BLINK_MIDIUM;
			break;
		case LED_BLINK_SLOW:
			led_ptrn = LED_BLINK_SLOW;
			break;
		default:
			//! flick bit & winker bit OFF
			if((ctrlout3 & (LED_COLOR_BLUE + LED_COLOR_GREEN + LED_COLOR_RED)) != 0){
				led_ptrn = LED_ON;
			}
			else{
				led_ptrn = LED_OFF;
			}
			break;
	}

	// winker
	switch (ctrlout3 & LED_WINKER_REQ){
		case LED_WINKER_RIGHT:
			led_ptrn_right = LED_BLINK_MIDIUM;
			led_ptrn_left = led_ptrn;
			color_right = LED_COLOR_YELLOW;
			color_left = color;
			break;
		case LED_WINKER_LEFT:
			led_ptrn_right = led_ptrn;
			led_ptrn_left = LED_BLINK_MIDIUM;
			color_right = color;
			color_left = LED_COLOR_YELLOW;
			break;
		case (LED_WINKER_RIGHT + LED_WINKER_LEFT):
			led_ptrn_right = LED_BLINK_MIDIUM;
			led_ptrn_left = LED_BLINK_MIDIUM;
			color_right = LED_COLOR_YELLOW;
			color_left = LED_COLOR_YELLOW;
			break;
		default:
			led_ptrn_right = led_ptrn;
			led_ptrn_left = led_ptrn;
			color_right = color;
			color_left = color;
			break;
	}

	// brink control
	led_cntrol(led_ptrn_right, led_ptrn_left, color_right, color_left);
}
