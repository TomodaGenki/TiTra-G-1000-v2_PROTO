/*
 * wheel_test.h;
 *
 *  Created on: 2021/05/07
 *      Author: takumi
 */

#ifndef WHEEL_TEST_H_
#define WHEEL_TEST_H_


void wheel_test_main(void);
short get_motor_ref_r(void);
short get_motor_ref_l(void);
void set_wheel_test_dump_req(void);
void reset_wheel_test_dump_req(void);
uint8_t get_wheel_test_dump_req(void);
void wheel_test_log_dump(void);
void set_wheel_test_dump_comp(void);
void reset_wheel_test_dump_comp(void);
uint8_t get_wheel_test_dump_comp(void);


#endif /* WHEEL_TEST_H_ */
