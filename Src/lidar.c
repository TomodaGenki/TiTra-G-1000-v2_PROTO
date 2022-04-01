/*
 * lidar.c
 *
 *  Created on: 2020/12/07
 *      Author: katte
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "conf.h"
#include "lidar.h"
#include "nuc.h"
#include "Cntrl_Proc.h"

/* Private function ----------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define LIDAR1_AREA			0x07
#define LIDAR2_AREA			0xF8

#define LIDAR_GUARD			0x1F

#define LIDAR_THRESH 25

#define LIDAR_NO_USE	0
#define LIDAR_USE_FRONT	1
#define LIDAR_USE_BACK	2
#define LIDAR_USE_BOTH	3

enum lidar_select{
	Lidar1,
	Lidar2
};

enum lidar_number {
	Lidar1_Long,
	Lidar1_Medium,
	Lidar1_Short,
	Lidar1_Fail,
	Lidar2_Long,
	Lidar2_Medium,
	Lidar2_Short,
	Lidar2_Fail,
	Lidar_Last
};

/* Private variables ---------------------------------------------------------*/
static uint8_t lidar_wait_time[Lidar_Last];
static uint8_t sensor3 = 0;
static uint8_t lidar1_area_command_old = 0xFF;
static uint8_t lidar2_area_command_old = 0xFF;
static uint8_t lidar_failure = 0;
static uint8_t lidar1_fail_flag = 0;
static uint8_t lidar2_fail_flag = 0;


void init_lidar_wait_time(void) {

	for (int i = Lidar1_Long; i < Lidar_Last; i++) {
		lidar_wait_time[i] = LIDAR_THRESH;
	}
}


void change_lidar_area(uint8_t area, uint8_t select) {

	if (select == Lidar1) {
#if (LIDAR_USE == LIDAR_USE_BACK) || (LIDAR_USE == LIDAR_USE_BOTH)
		if((area & BIT00) == BIT00) {
			HAL_GPIO_WritePin(O_Lider1In1_GPIO_Port, O_Lider1In1_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(O_Lider1In1_GPIO_Port, O_Lider1In1_Pin, GPIO_PIN_SET);
		}

		if((area & BIT01) == BIT01) {
			HAL_GPIO_WritePin(O_Lidar1In2_GPIO_Port, O_Lidar1In2_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(O_Lidar1In2_GPIO_Port, O_Lidar1In2_Pin, GPIO_PIN_SET);
		}

		if((area & BIT02) == BIT02) {
			HAL_GPIO_WritePin(O_Lidar1In3_GPIO_Port, O_Lidar1In3_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(O_Lidar1In3_GPIO_Port, O_Lidar1In3_Pin, GPIO_PIN_SET);
		}

		if((area & BIT03) == BIT03) {
			// lidar1_area_commandは下位3ビットを取り出しているのでBIT3は必ず0になる
			HAL_GPIO_WritePin(O_Lidar1In4_GPIO_Port, O_Lidar1In4_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(O_Lidar1In4_GPIO_Port, O_Lidar1In4_Pin, GPIO_PIN_SET);
		}
#endif
	} else {
#if (LIDAR_USE == LIDAR_USE_FRONT) || (LIDAR_USE == LIDAR_USE_BOTH)
		if((area & BIT00) == BIT00) {
			HAL_GPIO_WritePin(O_Lidar2In1_GPIO_Port, O_Lidar2In1_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(O_Lidar2In1_GPIO_Port, O_Lidar2In1_Pin, GPIO_PIN_SET);
		}

		if((area & BIT01) == BIT01) {
			HAL_GPIO_WritePin(O_Lidar2In2_GPIO_Port, O_Lidar2In2_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(O_Lidar2In2_GPIO_Port, O_Lidar2In2_Pin, GPIO_PIN_SET);
		}

		if((area & BIT02) == BIT02) {
			HAL_GPIO_WritePin(O_Lidar2In3_GPIO_Port, O_Lidar2In3_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(O_Lidar2In3_GPIO_Port, O_Lidar2In3_Pin, GPIO_PIN_SET);
		}

		if((area & BIT03) == BIT03) {
			HAL_GPIO_WritePin(O_Lidar2In4_GPIO_Port, O_Lidar2In4_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(O_Lidar2In4_GPIO_Port, O_Lidar2In4_Pin, GPIO_PIN_SET);
		}

		if((area & BIT04) == BIT04) {
			HAL_GPIO_WritePin(O_Lidar2In5_GPIO_Port, O_Lidar2In5_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(O_Lidar2In5_GPIO_Port, O_Lidar2In5_Pin, GPIO_PIN_SET);
		}
#endif
	}
}


void set_lidar_area(void) {

	// NUCからのコマンドを格納
	uint8_t lidar1_area_command = get_Lidar_Area() & LIDAR1_AREA;
	uint8_t	lidar2_area_command = (get_Lidar_Area() & LIDAR2_AREA) >> 3;

	//0x00をエリア1に割り当てたいので1オフセット
	lidar1_area_command = (lidar1_area_command + 1);
	lidar2_area_command = (lidar2_area_command + 1);

	// エリアは1~31までしか無いのでエリア32以上を指定された場合にガード処理を追加
	if (lidar2_area_command > LIDAR_GUARD) {
		// 例外エリア指定の場合、エリア1とする
		lidar2_area_command = BIT00;
	}

	if (lidar1_area_command != lidar1_area_command_old) {
		change_lidar_area(lidar1_area_command, Lidar1);
		lidar1_area_command_old = lidar1_area_command;
	}

	if(lidar2_area_command != lidar2_area_command_old) {
		change_lidar_area(lidar2_area_command, Lidar2);
		lidar2_area_command_old = lidar2_area_command;
	}
}


void add_lidar_wait_time(uint8_t lidar_num) {

	if (lidar_wait_time[lidar_num] < LIDAR_THRESH) {
		lidar_wait_time[lidar_num]++;
	} else {
		lidar_wait_time[lidar_num] = LIDAR_THRESH;
	}
}

void clr_lidar_wait_time(uint8_t lidar_num) {
	lidar_wait_time[lidar_num] = 0;
}

uint8_t judge_lidar_sensor(void) {

	uint8_t ret = 0;

	for (int i = Lidar1_Long; i < Lidar_Last; i++) {
		if (lidar_wait_time[i] < LIDAR_THRESH) {
			ret |= (BIT00 << i);
		}
	}
	return ret;
}

void scan_lidar(void){

	set_lidar_area();
	scan_lidar_failure();

#if (LIDAR_USE == LIDAR_USE_BACK) || (LIDAR_USE == LIDAR_USE_BOTH)
	// Lidar1
	if(HAL_GPIO_ReadPin(I_Lidar1Out1_GPIO_Port, I_Lidar1Out1_Pin) == GPIO_PIN_SET){
		clr_lidar_wait_time(Lidar1_Long);
	} else {
		add_lidar_wait_time(Lidar1_Long);
	}

	if(HAL_GPIO_ReadPin(I_Lidar1Out2_GPIO_Port, I_Lidar1Out2_Pin) == GPIO_PIN_SET){
		clr_lidar_wait_time(Lidar1_Medium);
	} else {
		add_lidar_wait_time(Lidar1_Medium);
	}

	if(HAL_GPIO_ReadPin(I_Lidar1Out3_GPIO_Port, I_Lidar1Out3_Pin) == GPIO_PIN_SET){
		clr_lidar_wait_time(Lidar1_Short);
	} else {
		add_lidar_wait_time(Lidar1_Short);
	}

	if(get_lidar1_fail_flag() != 0){
		clr_lidar_wait_time(Lidar1_Fail);
	} else {
		add_lidar_wait_time(Lidar1_Fail);
	}
#endif

#if (LIDAR_USE == LIDAR_USE_FRONT) || (LIDAR_USE == LIDAR_USE_BOTH)
	// Lidar2
	if(HAL_GPIO_ReadPin(I_Lidar2Out1_GPIO_Port, I_Lidar2Out1_Pin) == GPIO_PIN_SET){
		clr_lidar_wait_time(Lidar2_Long);
	} else {
		add_lidar_wait_time(Lidar2_Long);
	}

	if(HAL_GPIO_ReadPin(I_Lidar2Out2_GPIO_Port, I_Lidar2Out2_Pin) == GPIO_PIN_SET){
		clr_lidar_wait_time(Lidar2_Medium);
	} else {
		add_lidar_wait_time(Lidar2_Medium);
	}

	if(HAL_GPIO_ReadPin(I_Lidar2Out3_GPIO_Port, I_Lidar2Out3_Pin) == GPIO_PIN_SET){
		clr_lidar_wait_time(Lidar2_Short);
	} else {
		add_lidar_wait_time(Lidar2_Short);
	}

	if(get_lidar2_fail_flag() != 0){
		clr_lidar_wait_time(Lidar2_Fail);
	} else {
		add_lidar_wait_time(Lidar2_Fail);
	}
#endif

	sensor3 = judge_lidar_sensor();
}


void scan_lidar_failure(void) {

	static uint8_t lidar1_fail = 0;
	static uint8_t lidar2_fail = 0;

	lidar1_fail = lidar1_fail << BIT00;
	lidar2_fail = lidar2_fail << BIT00;

#if (LIDAR_USE == LIDAR_USE_BACK) || (LIDAR_USE == LIDAR_USE_BOTH)
	if (HAL_GPIO_ReadPin(I_Lidar1Trouble_GPIO_Port, I_Lidar1Trouble_Pin) == GPIO_PIN_SET) {
		lidar1_fail |= BIT00;
	}
#endif

#if (LIDAR_USE == LIDAR_USE_FRONT) || (LIDAR_USE == LIDAR_USE_BOTH)
	if (HAL_GPIO_ReadPin(I_Lidar2Trouble_GPIO_Port, I_Lidar2Trouble_Pin) == GPIO_PIN_SET) {
		lidar2_fail |= BIT00;
	}
#endif

	if (lidar1_fail == 0xFF) {
		set_lidar1_fail_flag();
		lidar_failure |= LIDAR1_FAILURE;
	} else if (lidar1_fail == 0x00) {
		lidar_failure &= ~LIDAR1_FAILURE;
	}
	if (lidar2_fail == 0xFF) {
		set_lidar2_fail_flag();
		lidar_failure |= LIDAR2_FAILURE;
	} else if (lidar2_fail == 0x00) {
		lidar_failure &= ~LIDAR2_FAILURE;
	}
}


void set_lidar1_fail_flag(void) {
	lidar1_fail_flag = 1;
}


void reset_lidar1_fail_flag(void) {
	lidar1_fail_flag = 0;
}


uint8_t get_lidar1_fail_flag(void) {
	return lidar1_fail_flag;
}


void set_lidar2_fail_flag(void) {
	lidar2_fail_flag = 1;
}


void reset_lidar2_fail_flag(void) {
	lidar2_fail_flag = 0;
}


uint8_t get_lidar2_fail_flag(void) {
	return lidar2_fail_flag;
}


uint8_t get_sensor3_state(void) {
	return sensor3;
}


uint8_t ck_lidar_failure(void) {
	return sensor3 & (LIDAR1_FAILURE + LIDAR2_FAILURE);
}


uint8_t ck_lidar_failsens(void) {
	return lidar_failure;
}
