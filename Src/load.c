/*
 * load.c
 *
 *  Created on: 2021/10/01
 *      Author: katte
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "syncturn.h"
#include "turn.h"
#include "common_func.h"
#include "Cntrl_Proc.h"
#include "nuc.h"
#include "load.h"
#include "stm32f4xx_hal_uart.h"
#include "conf.h"
/* Private function ----------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
#if USE_LOAD_MEASURE
extern UART_HandleTypeDef huart1;
#endif
extern UART_HandleTypeDef huart3;
/* Private define ------------------------------------------------------------*/
#define LOAD_LOG_NUM		400

#define W_RESISTER_NUM	0x02
#define R_RESISTER_NUM	0x02
#define W_BYTE_NUM		0x04

// Read要求送信用の定数定義
enum Read_Driver {
	R_Slave_Add,
	R_Func_Code,
	R_Resister_High,
	R_Resister_Low,
	R_Resister_Num_High,
	R_Resister_Num_Low,
	R_Crc_High,
	R_Crc_Low,
	Read_Buff_Size
};

// Read時の定数定義
enum Recive_Driver {
	Rx_Slave_Add,
	Rx_Func_Code,
	Rx_Data_Byte,
	Rx_Val_High,
	Rx_Val_Mdl_High,
	Rx_Val_Mdl_Low,
	Rx_Val_Low,
	Rx_Crc_High,
	Rx_Crc_Low,
	Receive_Buff_Size
};

/* Private variables ---------------------------------------------------------*/
static uint16_t load_log_point_lift = 1;
static uint16_t load_log_point_turn = 1;
static uint8_t load_log_data_lift[LOAD_LOG_NUM];
static uint8_t load_log_data_turn[LOAD_LOG_NUM];
static uint8_t rx_drive_data[Receive_Buff_Size];
static uint8_t driver_receive_flag = 0;
static uint8_t permit_read_driver = 1;
static uint8_t lift_load_log_flag = 0;
static uint8_t turn_load_log_flag = 0;

void set_driver_receive_flag(void) {
	driver_receive_flag = 1;
}

void reset_driver_receive_flag(void) {
	driver_receive_flag = 0;
}

uint8_t get_driver_receive_flag(void) {
	return driver_receive_flag;
}

void set_lift_load_log_flag(void) {
	lift_load_log_flag = 1;
}

void reset_lift_load_log_flag(void) {
	lift_load_log_flag = 0;
}

uint8_t get_lift_load_log_flag(void) {
	return lift_load_log_flag;
}

void set_turn_load_log_flag(void) {
	turn_load_log_flag = 1;
}

void reset_turn_load_log_flag(void) {
	turn_load_log_flag = 0;
}

uint8_t get_turn_load_log_flag(void) {
	return turn_load_log_flag;
}

void load_uart_trans(void) {
#if USE_LOAD_MEASURE
	HAL_GPIO_WritePin(O_USART1_TR_GPIO_Port, O_USART1_TR_Pin, GPIO_PIN_SET);
#endif
}

void load_uart_reveive(void) {
#if USE_LOAD_MEASURE
	HAL_GPIO_WritePin(O_USART1_TR_GPIO_Port, O_USART1_TR_Pin, GPIO_PIN_RESET);
#endif
}

void monitor_driver_dma_error(void) {
#if USE_LOAD_MEASURE
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) ||
		__HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE) ||
		__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE) ||
		__HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE)) {
		HAL_UART_Abort(&huart1);
		HAL_UART_Receive_DMA(&huart1, rx_drive_data, Receive_Buff_Size);
		reset_driver_receive_flag();
	}
#endif
}

void load_log_init(void) {

	// 先頭にダミーデータ格納(ログを読みやすくするため)
	load_log_data_lift[0] = 0xAA;
	load_log_data_turn[0] = 0xBB;
	load_log_point_lift = 1;
	load_log_point_turn = 1;

	for (int i = 1; i < LOAD_LOG_NUM; i++) {
		load_log_data_lift[i] = 0;
		load_log_data_turn[i] = 0;
	}
}

void load_init(void) {
	load_log_init();
	load_uart_trans();
#if USE_LOAD_MEASURE
	HAL_UART_Receive_DMA(&huart1, rx_drive_data, Receive_Buff_Size);
#endif
}


// 2つのレジスタ(上位・下位)に対して読み込みを行うためのインターフェース
// slave_add : 読み込みを行うスレーブアドレス(1:リフトモーター、2:ターンモーター)
// resister : 読み込みの起点となるレジスタアドレス
void read_mtrdriver_data(uint8_t slave_add, uint16_t resister) {
#if USE_LOAD_MEASURE
	static uint8_t tx_buff[Read_Buff_Size];
	union LongByte tx_cnv;
	uint16_t crc;

	// set trans data
	tx_buff[R_Slave_Add] = slave_add;				// set slave address
	tx_buff[R_Func_Code] = READ_MTR_VAL;			// set function code (0x03 : multiple read)

	tx_cnv.w_val[0] = resister;
	tx_buff[R_Resister_High] = tx_cnv.b_val[1];		// set resister address high byte
	tx_buff[R_Resister_Low] = tx_cnv.b_val[0];		// set resister address low byte

	tx_cnv.w_val[0] = R_RESISTER_NUM;
	tx_buff[R_Resister_Num_High] = tx_cnv.b_val[1];	// set number of resister for write high byte
	tx_buff[R_Resister_Num_Low] = tx_cnv.b_val[0];	// set number of resister for write low byte

	crc = crc16_calc(tx_buff, Read_Buff_Size);

	tx_cnv.w_val[0] = crc;
	tx_buff[R_Crc_High] = tx_cnv.b_val[0];			// set error check high byte
	tx_buff[R_Crc_Low] = tx_cnv.b_val[1];			// set error check low byte

	HAL_UART_Transmit_DMA(&huart1, tx_buff, Read_Buff_Size);
#endif
}

// ログ出力用関数
// 指定コマンド受領によりログを出力する
void ck_load_log_data(void) {

	if (get_lift_load_log_flag() != 0) {
		reset_lift_load_log_flag();
		HAL_UART_Transmit_DMA(&huart3, load_log_data_lift, sizeof(load_log_data_lift));
	} else if (get_turn_load_log_flag() != 0) {
		reset_turn_load_log_flag();
		HAL_UART_Transmit_DMA(&huart3, load_log_data_turn, sizeof(load_log_data_turn));
	}
}

// 負荷率測定のメイン関数
// コールタイミング:20msec
void load_measure(void) {

	if ((Is_SyncTurnning() != 0) && (permit_read_driver == 1)) {
		permit_read_driver = 0;
		read_mtrdriver_data(SLAVE_ADD2, LOAD_FACTOR);
	} else if (((get_sensor2_state() & LIFT_COMPLTE) != LIFT_COMPLTE) && (permit_read_driver == 1)) {
		permit_read_driver = 0;
		read_mtrdriver_data(SLAVE_ADD1, LOAD_FACTOR);
	} else if (get_turn_status() != ST_TURN_VEL_RUN) {
		permit_read_driver = 0;
		read_mtrdriver_data(SLAVE_ADD2, LOAD_FACTOR);
	}

	ck_load_log_data();
}

// モータードライバからの返信を確認する
// UART1の受信完了割り込みにより driver_receive_flag がONとなり処理が走る
// main.c の 2msecタイマーからコール
void ck_motor_receive_data(void) {

	union LongByte rx_cnv;
	uint16_t rx_crc;
	uint16_t crc;
	uint8_t load_tmp;
	uint8_t slave_add;
	rx_cnv.l_val = 0;

	if (get_driver_receive_flag() != 0) {
		reset_driver_receive_flag();
		permit_read_driver = 1;
		rx_cnv.b_val[0] = rx_drive_data[Rx_Crc_High];
		rx_cnv.b_val[1] = rx_drive_data[Rx_Crc_Low];
		rx_crc = rx_cnv.w_val[0];

		crc = crc16_calc(rx_drive_data, sizeof(rx_drive_data));

		if (crc == rx_crc) {
			slave_add = rx_drive_data[Rx_Slave_Add];
			rx_cnv.b_val[3] = rx_drive_data[Rx_Val_High];
			rx_cnv.b_val[2] = rx_drive_data[Rx_Val_Mdl_High];
			rx_cnv.b_val[1] = rx_drive_data[Rx_Val_Mdl_Low];
			rx_cnv.b_val[0] = rx_drive_data[Rx_Val_Low];

			// 負荷率の情報は0 ~ 200なので、容量削減のため1Byteに圧縮する
			load_tmp = (uint8_t)rx_cnv.l_val;

			if (slave_add == SLAVE_ADD1) {
				// Liftからの受信
				load_log_data_lift[load_log_point_lift] = load_tmp;
				if (load_log_point_lift < (LOAD_LOG_NUM-1)) {
					load_log_point_lift++;
				}
			} else if (slave_add == SLAVE_ADD2) {
				// Turnからの受信
				load_log_data_turn[load_log_point_turn] = load_tmp;
				if (load_log_point_turn < (LOAD_LOG_NUM-1)) {
					load_log_point_turn++;
				}
			}
		}
	}
}

