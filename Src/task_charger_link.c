/*
 * task_charger_link.c
 *
 *  Edited : 2021/08/03
 *      Author: katte
 */

/* Includes ------------------------------*/
#include "main.h"
#include "stm32f4xx_hal_irda.h"
#include "Cntrl_Proc.h"
#include "task_charger_link.h"
#include "charge.h"

/* Private function ----------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern IRDA_HandleTypeDef hirda2;
/* Private define ------------------------------------------------------------*/
#define CHGR_PACKETSIZE	5

/* Private variables ---------------------------------------------------------*/
static uint8_t irda_rx_buff[CHGR_PACKETSIZE];
static uint8_t chgr_rxdata = 0;
static uint8_t chgr_txdata = 0;
static uint8_t irda_receive_flag = 0;

/*----------------------------------------------------------*/
/*	clear communication buffer								*/
/*----------------------------------------------------------*/
void clr_irda_comdata(void) {
	chgr_rxdata = 0;
	chgr_txdata = 0;
}

/*----------------------------------------------------------*/
/*	check charge request 									*/
/*----------------------------------------------------------*/
uint8_t ck_irda_comm(void) {
	return (chgr_rxdata & PT_COM_RES);
}

/*----------------------------------------------------------*/
/*	check charging state									*/
/*----------------------------------------------------------*/
uint8_t ck_irda_charging(void) {
	return (chgr_rxdata & PT_CHARGE_ING);
}

/*----------------------------------------------------------*/
/*	check error stop 										*/
/*----------------------------------------------------------*/
uint8_t ck_irda_error(void) {
	return (chgr_rxdata & PT_ERR_STOP);
}

/*----------------------------------------------------------*/
/*	set communication request with charger					*/
/*----------------------------------------------------------*/
void set_irda_comm_req(void) {
	chgr_txdata |= PT_COM_REQ;
}

/*----------------------------------------------------------*/
/*	reset communication request with charger				*/
/*----------------------------------------------------------*/
void reset_irda_comm_req(void) {
	chgr_txdata &= ~PT_COM_REQ;
}

/*----------------------------------------------------------*/
/*	set charging request									*/
/*----------------------------------------------------------*/
void set_irda_charge_req(void) {
	chgr_txdata |= PT_CHARGE_REQ;
}

/*----------------------------------------------------------*/
/*	reset charging request									*/
/*----------------------------------------------------------*/
void reset_irda_charge_req(void) {
	chgr_txdata &= ~PT_CHARGE_REQ;
}

/*----------------------------------------------------------*/
/*	set error request										*/
/*----------------------------------------------------------*/
void set_irda_error_req(void) {
	chgr_txdata |= PT_EMG_STOP;
}

/*----------------------------------------------------------*/
/*	reset error request										*/
/*----------------------------------------------------------*/
void reset_irda_error_req(void) {
	chgr_txdata &= ~PT_EMG_STOP;
}

/*----------------------------------------------------------*/
/*	IrDAの初期化												*/
/*----------------------------------------------------------*/
void chagerlink_init(void) {
	HAL_IRDA_Receive_DMA(&hirda2, irda_rx_buff, CHGR_PACKETSIZE);
}

/*----------------------------------------------------------*/
/*	IrDA receive flag control								*/
/*----------------------------------------------------------*/
void set_irda_receive_flag(void) {
	irda_receive_flag = 1;
}

void reset_irda_receive_flag(void) {
	irda_receive_flag = 0;
}

uint8_t get_irda_receive_flag(void) {
	return irda_receive_flag;
}

void reset_irda_process(void) {
	HAL_IRDA_Abort(&hirda2);
	HAL_IRDA_Receive_DMA(&hirda2, irda_rx_buff, CHGR_PACKETSIZE);
}

/*----------------------------------------------------------*/
/*	RX call back function									*/
/*----------------------------------------------------------*/
void charger_data_receive(void) {

	uint8_t chgr_rx_buff[CHGR_PACKETSIZE];

	if (get_irda_receive_flag() != 0) {
		reset_irda_receive_flag();
		for (int i = 0; i < CHGR_PACKETSIZE; i++) {
			chgr_rx_buff[i] = irda_rx_buff[i];
		}

		if ((chgr_rx_buff[0] == STXiR) && (chgr_rx_buff[1] == LENiR) && (chgr_rx_buff[4] == ETXiR)) {
			chgr_rxdata = chgr_rx_buff[3];
			set_utimer(CNT_IRDACOMM, CNT_IRDADOWN);
		} else {
			reset_irda_process();
		}
	}
	if (check_timer(CNT_IRDACOMM) == TIME_UP) {
		clr_irda_comdata();
	}
}

/*----------------------------------------------------------*/
/*	monitoring irda error									*/
/*----------------------------------------------------------*/
void monitor_irda_dma_error(void) {

	if ((__HAL_IRDA_GET_FLAG(&hirda2, IRDA_FLAG_ORE)) ||
		(__HAL_IRDA_GET_FLAG(&hirda2, IRDA_FLAG_NE)) ||
		(__HAL_IRDA_GET_FLAG(&hirda2, IRDA_FLAG_FE)) ||
		(__HAL_IRDA_GET_FLAG(&hirda2, IRDA_FLAG_PE))) {
		reset_irda_receive_flag();
		reset_irda_process();
	}
}

/*----------------------------------------------------------*/
/*	NVIC function for irda complete receive					*/
/*----------------------------------------------------------*/
void HAL_IRDA_RxCpltCallback(IRDA_HandleTypeDef *hirda) {

	if (hirda->Instance == hirda2.Instance) {
		set_irda_receive_flag();
	}
}

/*----------------------------------------------------------*/
/*　	送信処理													*/
/*----------------------------------------------------------*/
void transmit_charge_cmd(void) {

	static uint8_t chgr_tx_buff[CHGR_PACKETSIZE];
	HAL_StatusTypeDef ret = HAL_OK;

	if (get_charge_state_info() != Charge_Info_Idle) {
		// 充電シーケンスが開始された場合のみ、IrDA送信を行う
		chgr_tx_buff[0] = STXiR;										// header
		chgr_tx_buff[1] = LENiR;										// size
		chgr_tx_buff[2] = 0x12;											// command
		chgr_tx_buff[3] = chgr_txdata;									// data
		chgr_tx_buff[4] = ETXiR;										// footer

		ret = HAL_IRDA_Transmit_DMA(&hirda2, chgr_tx_buff, CHGR_PACKETSIZE);

		if (ret != HAL_OK) {
			reset_irda_process();
		}
	}
}
