/*
 * lidar.h
 *
 *  Created on: 2020/12/07
 *      Author: katte
 */

#ifndef LIDAR_H_
#define LIDAR_H_

/* function ----------------------------------------------------------*/
void scan_lidar(void);
void init_lidar_wait_time(void);
uint8_t get_sensor3_state(void);
uint8_t ck_lidar_failure(void);
void scan_lidar_failure(void);
void set_lidar1_fail_flag(void);
void reset_lidar1_fail_flag(void);
uint8_t get_lidar1_fail_flag(void);
void set_lidar2_fail_flag(void);
void reset_lidar2_fail_flag(void);
uint8_t get_lidar2_fail_flag(void);
uint8_t ck_lidar_failsens(void);

/* define ------------------------------------------------------------*/
#define LIDAR1_LONG			0x01
#define LIDAR1_MEDIUM		0x02
#define LIDAR1_SHORT		0x04
#define LIDAR1_FAILURE		0x08
#define LIDAR2_LONG			0x10
#define LIDAR2_MEDIUM		0x20
#define LIDAR2_SHORT		0x40
#define LIDAR2_FAILURE		0x80

#endif /* LIDAR_H_ */
