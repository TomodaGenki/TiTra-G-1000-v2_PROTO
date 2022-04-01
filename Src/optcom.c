/*
 * optcom.c
 *
 *  Created on: 2021/06/22
 *      Author: Katte
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "optcom.h"
#include "Cntrl_Proc.h"
#include "conf.h"

/* External variables --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define OPTCOM_BIT01	0x01
#define	OPTCOM_BIT02	0x02
#define	OPTCOM_BIT03	0x04
#define	OPTCOM_BIT04	0x08
#define	OPTCOM_BITGO	0x10

/* Private variables ---------------------------------------------------------*/
static uint8_t rx_optdata_tmp[2] = {0,0};
static uint8_t rx_optdata = 0;
static uint8_t tx_optdata = 0;

uint8_t get_rx_optdata_bit01(void) {
	return rx_optdata & OPTCOM_BIT01;
}

uint8_t get_rx_optdata_bit02(void) {
	return rx_optdata & OPTCOM_BIT02;
}

uint8_t get_rx_optdata_bit03(void) {
	return rx_optdata & OPTCOM_BIT03;
}

uint8_t get_rx_optdata_bit04(void) {
	return rx_optdata & OPTCOM_BIT04;
}

uint8_t get_rx_optdata_bitgo(void) {
	return rx_optdata & OPTCOM_BITGO;
}

void set_tx_optdata_bit01(void) {
	tx_optdata |= OPTCOM_BIT01;
}

void reset_tx_optdata_bit01(void) {
	tx_optdata &= ~OPTCOM_BIT01;
}

void set_tx_optdata_bit02(void) {
	tx_optdata |= OPTCOM_BIT02;
}

void reset_tx_optdata_bit02(void) {
	tx_optdata &= ~OPTCOM_BIT02;
}

void set_tx_optdata_bit03(void) {
	tx_optdata |= OPTCOM_BIT03;
}

void reset_tx_optdata_bit03(void) {
	tx_optdata &= ~OPTCOM_BIT03;
}

void set_tx_optdata_bit04(void) {
	tx_optdata |= OPTCOM_BIT04;
}

void reset_tx_optdata_bit04(void) {
	tx_optdata &= ~OPTCOM_BIT04;
}

void scan_optdata(void) {

	rx_optdata_tmp[1] = rx_optdata_tmp[0];
	rx_optdata_tmp[0] = 0;
#if OPTCOM_USE
	if (HAL_GPIO_ReadPin(I_OptOut1_GPIO_Port, I_OptOut1_Pin) == GPIO_PIN_SET) {
		rx_optdata_tmp[0] |= OPTCOM_BIT01;
	}
	if (HAL_GPIO_ReadPin(I_OptOut2_GPIO_Port, I_OptOut2_Pin) == GPIO_PIN_SET) {
		rx_optdata_tmp[0] |= OPTCOM_BIT02;
	}
	if (HAL_GPIO_ReadPin(I_OptOut3_GPIO_Port, I_OptOut3_Pin) == GPIO_PIN_SET) {
		rx_optdata_tmp[0] |= OPTCOM_BIT03;
	}
	if (HAL_GPIO_ReadPin(I_OptOut4_GPIO_Port, I_OptOut4_Pin) == GPIO_PIN_SET) {
		rx_optdata_tmp[0] |= OPTCOM_BIT04;
	}
	if (HAL_GPIO_ReadPin(I_OptGo_GPIO_Port, I_OptGo_Pin) == GPIO_PIN_SET) {
		rx_optdata_tmp[0] |= OPTCOM_BITGO;
	}
	// chattering check
	if (((rx_optdata_tmp[0] & OPTCOM_BIT01) == OPTCOM_BIT01) && ((rx_optdata_tmp[1] & OPTCOM_BIT01) == OPTCOM_BIT01)) {
		rx_optdata |= OPTCOM_BIT01;
	} else if (((rx_optdata_tmp[0] & OPTCOM_BIT01) != OPTCOM_BIT01) && ((rx_optdata_tmp[1] & OPTCOM_BIT01) != OPTCOM_BIT01)) {
		rx_optdata &= ~OPTCOM_BIT01;
	}

	if (((rx_optdata_tmp[0] & OPTCOM_BIT02) == OPTCOM_BIT02) && ((rx_optdata_tmp[1] & OPTCOM_BIT02) == OPTCOM_BIT02)) {
		rx_optdata |= OPTCOM_BIT02;
	} else if (((rx_optdata_tmp[0] & OPTCOM_BIT02) != OPTCOM_BIT02) && ((rx_optdata_tmp[1] & OPTCOM_BIT02) != OPTCOM_BIT02)) {
		rx_optdata &= ~OPTCOM_BIT02;
	}

	if (((rx_optdata_tmp[0] & OPTCOM_BIT03) == OPTCOM_BIT03) && ((rx_optdata_tmp[1] & OPTCOM_BIT03) == OPTCOM_BIT03)) {
		rx_optdata |= OPTCOM_BIT03;
	} else if (((rx_optdata_tmp[0] & OPTCOM_BIT03) != OPTCOM_BIT03) && ((rx_optdata_tmp[1] & OPTCOM_BIT03) != OPTCOM_BIT03)) {
		rx_optdata &= ~OPTCOM_BIT03;
	}

	if (((rx_optdata_tmp[0] & OPTCOM_BIT04) == OPTCOM_BIT04) && ((rx_optdata_tmp[1] & OPTCOM_BIT04) == OPTCOM_BIT04)) {
		rx_optdata |= OPTCOM_BIT04;
	} else if (((rx_optdata_tmp[0] & OPTCOM_BIT04) != OPTCOM_BIT04) && ((rx_optdata_tmp[1] & OPTCOM_BIT04) != OPTCOM_BIT04)) {
		rx_optdata &= ~OPTCOM_BIT04;
	}

	if (((rx_optdata_tmp[0] & OPTCOM_BITGO) == OPTCOM_BITGO) && ((rx_optdata_tmp[1] & OPTCOM_BITGO) == OPTCOM_BITGO)) {
		rx_optdata |= OPTCOM_BITGO;
	} else if (((rx_optdata_tmp[0] & OPTCOM_BITGO) != OPTCOM_BITGO) && ((rx_optdata_tmp[1] & OPTCOM_BITGO) != OPTCOM_BITGO)) {
		rx_optdata &= ~OPTCOM_BITGO;
	}
#endif
}

void output_optdata(void) {
#if OPTCOM_USE
	if ((tx_optdata & OPTCOM_BIT01) == OPTCOM_BIT01) {
		HAL_GPIO_WritePin(O_OptIn1_GPIO_Port, O_OptIn1_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(O_OptIn1_GPIO_Port, O_OptIn1_Pin, GPIO_PIN_RESET);
	}

	if ((tx_optdata & OPTCOM_BIT02) == OPTCOM_BIT02) {
		HAL_GPIO_WritePin(O_OptIn2_GPIO_Port, O_OptIn2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(O_OptIn2_GPIO_Port, O_OptIn2_Pin, GPIO_PIN_RESET);
	}

	if ((tx_optdata & OPTCOM_BIT03) == OPTCOM_BIT03) {
		HAL_GPIO_WritePin(O_OptIn3_GPIO_Port, O_OptIn3_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(O_OptIn3_GPIO_Port, O_OptIn3_Pin, GPIO_PIN_RESET);
	}

	if ((tx_optdata & OPTCOM_BIT04) == OPTCOM_BIT04) {
		HAL_GPIO_WritePin(O_OptIn4_GPIO_Port, O_OptIn4_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(O_OptIn4_GPIO_Port, O_OptIn4_Pin, GPIO_PIN_RESET);
	}
#endif
}

void init_optdata(void) {
	tx_optdata = 0;
}
