/*
 * sound.c
 *
 *  Created on: 2020/5/14
 *      Author: katte
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sound.h"
#include "nuc.h"
#include "Cntrl_Proc.h"
#include "conf.h"
/* Private function ----------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

void channel_select(uint8_t ch) {
#if SOUND_USE

	if (ch & SOUND_CH1) {
		HAL_GPIO_WritePin(O_AlarmCh1_GPIO_Port, O_AlarmCh1_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(O_AlarmCh1_GPIO_Port, O_AlarmCh1_Pin, GPIO_PIN_RESET);
	}

	if (ch & SOUND_CH2) {
		//HAL_GPIO_WritePin(O_AlarmCh2_GPIO_Port, O_AlarmCh2_Pin, GPIO_PIN_SET);
	} else {
		//HAL_GPIO_WritePin(O_AlarmCh2_GPIO_Port, O_AlarmCh2_Pin, GPIO_PIN_RESET);
	}

	if (ch & SOUND_CH3) {
		HAL_GPIO_WritePin(O_AlarmCh3_GPIO_Port, O_AlarmCh3_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(O_AlarmCh3_GPIO_Port, O_AlarmCh3_Pin, GPIO_PIN_RESET);
	}

	if (ch & SOUND_CH4) {
		HAL_GPIO_WritePin(O_AlarmCh4_GPIO_Port, O_AlarmCh4_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(O_AlarmCh4_GPIO_Port, O_AlarmCh4_Pin, GPIO_PIN_RESET);
	}
#endif
}

void sound_cntrol(void) {

	static uint8_t last_sound_order = 0x00;
	uint8_t sound_order = get_sound_order();

	if (ck_emerg_stop() != NOT_EMERGENCY) {
		// 緊急停止時はマイコン主導で音を鳴らす。音はメッセージNo.1にする
		sound_order = SOUND_CH1;
	}

	if (sound_order != last_sound_order) {
		channel_select(sound_order);
		last_sound_order = sound_order;
	}
}
