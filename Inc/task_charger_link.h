/*
 * task_charger_link.h
 *
 *  Created on: 2020/12/07
 *      Author: katte
 */

#ifndef TASK_CHARGER_LINK_H_
#define TASK_CHARGER_LINK_H_

/* function ----------------------------------------------------------*/
void chagerlink_init(void);
void charger_data_receive(void);
void transmit_charge_cmd(void);
void monitor_irda_dma_error(void);

void clr_irda_comdata(void);
void set_irda_comm_req(void);
void reset_irda_comm_req(void);
void set_irda_charge_req(void);
void reset_irda_charge_req(void);
void set_irda_error_req(void);
void reset_irda_error_req(void);

uint8_t ck_irda_comm(void);
uint8_t ck_irda_charging(void);
uint8_t ck_irda_error(void);

/* define ------------------------------------------------------------*/
#define STXiR	0x55
#define LENiR	0x02
#define ETXiR	0x03

#define PT_COM_REQ				   0x01
#define PT_CHARGE_REQ			   0x02
#define PT_EMG_STOP				   0x08

#define PT_COM_RES				   0x01  // bit0:responce check
#define PT_CHARGE_ING			   0x02  // bit1:charging
#define PT_ERR_STOP				   0x08  // bit3:error

#endif /* TASK_CHARGER_LINK_H_ */
