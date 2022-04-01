/*
 * can.h
 *
 *  Created on: 2021/11/10
 *      Author: Katte
 */

#ifndef CAN_H_
#define CAN_H_

/* function ----------------------------------------------------------*/
void can_filter_setting(void);
HAL_StatusTypeDef can1_transmit(void);
uint8_t can1_enque(uint16_t id, uint8_t *pdata);
uint8_t can1_deque(uint16_t *p_id, uint8_t *pdata);

/* define ------------------------------------------------------------*/


#endif /* CAN */
