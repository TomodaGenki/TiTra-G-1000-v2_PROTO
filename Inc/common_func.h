/*
 * common_func.h
 *
 *  Created on: 2021/09/09
 *      Author: tomoda
 */

#ifndef COMMON_FUNC_H_
#define COMMON_FUNC_H_

double mysin(double x);
double LPF_1order(double input, double input_old, double output_old, double tau, double delta_t);
uint16_t crc16_calc(uint8_t *buff, uint8_t sizeOfArray);

// 4Byte‚Ì‹¤—p‘Ì
// ’ÊM“™‚Å2,4Byte‚ğ1Byte‚É•ªŠ„‚·‚é“™‚É—˜—p
union LongByte {
	uint32_t l_val;
	uint16_t w_val[2];
	uint8_t b_val[4];
};

#endif /* COMMON_FUNC_H_ */
