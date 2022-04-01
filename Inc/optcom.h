/*
 * optcom.h
 *
 *  Created on: 2021/06/22
 *      Author: Katte
 */

#ifndef OPTCOM_H_
#define OPTCOM_H_

/* function ----------------------------------------------------------*/
void scan_optdata(void);
void output_optdata(void);
void init_optdata(void);
uint8_t get_rx_optdata_bit01(void);
uint8_t get_rx_optdata_bit02(void);
uint8_t get_rx_optdata_bit03(void);
uint8_t get_rx_optdata_bit04(void);
uint8_t get_rx_optdata_bitgo(void);
void set_tx_optdata_bit01(void);
void reset_tx_optdata_bit01(void);
void set_tx_optdata_bit02(void);
void reset_tx_optdata_bit02(void);
void set_tx_optdata_bit03(void);
void reset_tx_optdata_bit03(void);
void set_tx_optdata_bit04(void);
void reset_tx_optdata_bit04(void);
/* define ------------------------------------------------------------*/

#endif /* OPTCOM */
