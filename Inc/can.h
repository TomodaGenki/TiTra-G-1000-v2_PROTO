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
HAL_StatusTypeDef can1_transmit(uint16_t id, uint8_t *pdata);

/* define ------------------------------------------------------------*/


#endif /* CAN */
