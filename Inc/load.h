/*
 * load.h
 *
 *  Created on: 2021/10/01
 *      Author: katte
 */

#ifndef LOAD_H_
#define LOAD_H_

/* function ----------------------------------------------------------*/
void set_driver_receive_flag(void);
void reset_driver_receive_flag(void);
uint8_t get_driver_receive_flag(void);
void load_uart_trans(void);
void load_uart_reveive(void);
void set_lift_load_log_flag(void);
void reset_lift_load_log_flag(void);
uint8_t get_lift_load_log_flag(void);
void set_turn_load_log_flag(void);
void reset_turn_load_log_flag(void);
uint8_t get_turn_load_log_flag(void);

void load_measure(void);
void ck_motor_receive_data(void);
void load_init(void);
void monitor_driver_dma_error(void);
/* define ------------------------------------------------------------*/
#define READ_MTR_VAL			0x03
#define WRITE_MTR_VAL			0x06
#define WRITE_MULTI_MTR_VALS	0x10
#define READ_WRITE_MTR_VAL		0x17

#define LOAD_FACTOR		0x0108

// define slave address
#define BROAD_CAST	0x00
#define SLAVE_ADD1	0x01	// Lift Motor Driver
#define SLAVE_ADD2	0x02	// Turn Motor Driver

#endif /* LOAD_H_ */
