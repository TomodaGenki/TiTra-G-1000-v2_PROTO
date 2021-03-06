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

// 4Byteの共用体
// 通信等で2,4Byteを1Byteに分割する時等に利用
union LongByte {
	uint32_t l_val;
	uint16_t w_val[2];
	uint8_t b_val[4];
};

#endif /* COMMON_FUNC_H_ */
