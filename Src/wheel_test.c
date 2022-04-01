/*
 * wheel_test.c
 *
 *  Created on: 2021/05/07
 *      Author: takumi
 */


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stdint.h"
#include "wheel_test.h"
#include "main.h"
#include "nuc.h"
#include "wheel.h"
#include "Cntrl_Proc.h"
#include "conf.h"

#define TEST_TIME_INT 10	// 制御周期 ms
#define TEST_READY	1		// ステータス：READY
#define TEST_ING	2		// ステータス：ING
#define TEST_COMP	3		// ステータス：COMP
#define MAX_PHASE	13		// フェーズ最大値
#define LOGSIZE_TEST 2600	// ログ点数
#define TIM_CYCLE	24.39024	//タイマーカウントアップ周期(secを10^6倍)


#if WHEEL_TEST
typedef struct test_log{
	uint16_t	idx_;
	uint32_t	timer_[LOGSIZE_TEST];
	uint8_t		phase_[LOGSIZE_TEST];
	double		right_ref_[LOGSIZE_TEST];
	double		left_ref_[LOGSIZE_TEST];
	short		right_out_[LOGSIZE_TEST];
	short		left_out_[LOGSIZE_TEST];
	double		right_spd_[LOGSIZE_TEST];
	double		left_spd_[LOGSIZE_TEST];
	short		right_enc_[LOGSIZE_TEST];
	short		left_enc_[LOGSIZE_TEST];
}TEST_LOG;

TEST_LOG logdata;
#endif

extern uint32_t	freerun;
static double dist_per_pulse = WHEEL_DIAMETER * PI / PULS_WHEEL;
uint8_t flg_wheel_test_req = 0;
uint8_t test_status = TEST_READY;
uint8_t flg_test_start_edge;
uint16_t timer = 0;
uint16_t start_time = 0;
uint16_t phase_timer = 0;
uint16_t phase_start_time = 0;
uint8_t	phase = 0;					// 制御フェーズ

//					   	        0     1      2     3      4     5    6    7    8    9    10   11   12   13
//uint16_t phase_time[] = 	{1000, 1333,  2000, 1333,  1000,    0,   0,   0,   0,   0,   0,   0,   0,   0};
//double	s_ref_phase_r[] =   {0.00,  0.0,   0.8,  0.8,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//double	e_ref_phase_r[] =   {0.00,  0.8,   0.8,  0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//double	s_ref_phase_l[] =   {0.00,  0.0,   0.8,  0.8,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//double	e_ref_phase_l[] =   {0.00,  0.8,   0.8,  0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

uint16_t phase_time[] = 	{1000,  2000,  2000,  2000,  1000,    0,   0,   0,   0,   0,   0,   0,   0,   0};
double	s_ref_phase_r[] =   {0.00,   0.0,   0.8,   0.8,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double	e_ref_phase_r[] =   {0.00,   0.8,   0.8,   0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double	s_ref_phase_l[] =   {0.00,   0.0,   0.8,   0.8,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double	e_ref_phase_l[] =   {0.00,   0.8,   0.8,   0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

double	right_ref = 0;			// タイヤ周速指示[m/s]
double	left_ref = 0;
short	right_out = 0;			// モーター制御に渡す指示値(-30000～30000)
short	left_out = 0;
double	right_spd = 0;			// 実角速度[m/s]
double	left_spd = 0;
int32_t	right_enc = 0;			// エンコーダパルスカウンタ
int32_t	left_enc = 0;
uint8_t wheel_test_dump_req = 0;	// ログ出力の要求フラグ
uint8_t wheel_test_dump_comp = 0;	// ログ出力の完了フラグ


void update_timer(){
/*
 * タイマー更新
 */
	timer = freerun - start_time;	// [ms]
}


void set_motor_ref_r(double speed){
	// 速度指示[m/s]を-30000~30000に変換してセット
	right_out = (short)(speed * (double)MAX_NUC_REF / MAX_NUC_SPEED_WHEEL);
}


void set_motor_ref_l(double speed){
	// 速度指示[m/s]を-30000~30000に変換してセット
	left_out = (short)(speed * (double)MAX_NUC_REF / MAX_NUC_SPEED_WHEEL);
}


short get_motor_ref_r(void){
	return right_out;
}


short get_motor_ref_l(void){
	return left_out;
}


void set_test_start_flg(){
	flg_test_start_edge = 1;
}


void reset_test_start_flg(){
	flg_test_start_edge = 0;
}


uint8_t get_test_start_flg(){
	return flg_test_start_edge;
}


void set_wheel_test_dump_req(){
	wheel_test_dump_req = 1;
}


void reset_wheel_test_dump_req(){
	wheel_test_dump_req = 0;
}


uint8_t get_wheel_test_dump_req(){
	return wheel_test_dump_req;
}


void set_wheel_test_dump_comp(){
	wheel_test_dump_comp = 1;
}


void reset_wheel_test_dump_comp(){
	wheel_test_dump_comp = 0;
}


uint8_t get_wheel_test_dump_comp(){
	return wheel_test_dump_comp;
}


void phase_update(){
	uint8_t phase_old = phase;
	uint8_t i = 0;
	uint32_t phase_time_sum = 0;
	// 現在のフェーズを判定
	for (i = 0; i <= MAX_PHASE; i++){
		phase_time_sum += phase_time[i];
		if(phase_time_sum >= timer){
			phase = i;
			break;
		}
	}

	//フェーズ開始時からの時間を計算
	if (phase > phase_old){
		phase_start_time = freerun;
		phase_timer = 0;
	}
	phase_timer = freerun - phase_start_time;
}

void motor_spd_decision(){
/*
 * モーターの指示速度を決定
 * 最大値は30000
 */
	phase_update();

	right_ref = s_ref_phase_r[phase] + (e_ref_phase_r[phase] - s_ref_phase_r[phase]) / (double)phase_time[phase] * (double)phase_timer;
	left_ref = s_ref_phase_l[phase] + (e_ref_phase_l[phase] - s_ref_phase_l[phase]) / (double)phase_time[phase] * (double)phase_timer;
}
void wheel_enc_count(){
/*
 * エンコーダパルスをカウントする
 */
	static uint32_t l_wheel_encoder = 0;
	static uint32_t r_wheel_encoder = 0;
	static uint32_t l_wheel_encoder_old = 0;
	static uint32_t r_wheel_encoder_old = 0;

	if(get_test_start_flg() == 1 ){	// TEST_INGに移行した1JOB目のみ実行
		l_wheel_encoder = 0;
		r_wheel_encoder = 0;
		l_wheel_encoder_old = get_l_wheel_encoder();
		r_wheel_encoder_old = get_r_wheel_encoder();
		left_enc = 0;
		right_enc = 0;
	}
	else{
		l_wheel_encoder = get_l_wheel_encoder();
		r_wheel_encoder = get_r_wheel_encoder();
		left_enc += (l_wheel_encoder - l_wheel_encoder_old);
		right_enc += (r_wheel_encoder_old - r_wheel_encoder);
		l_wheel_encoder_old = l_wheel_encoder;
		r_wheel_encoder_old = r_wheel_encoder;
	}
}

void wheel_spd_measure(){
/*
 * エンコーダから速度を取得する
 */
	double cnt_left = (double)get_l_wheel_enc_cnt();
	if ( cnt_left != 0){
		left_spd = dist_per_pulse * 1.0e6 / (TIM_CYCLE * (double)cnt_left);	// m/s
	}

	double cnt_right = (double)get_r_wheel_enc_cnt();
	if ( cnt_right != 0){
		right_spd = dist_per_pulse * 1.0e6 / (TIM_CYCLE * (double)cnt_right);	// m/s
	}
}


void test_log_update(){
/*
 * ログを更新する
 */
#if WHEEL_TEST
	if(logdata.idx_ < LOGSIZE_TEST){
		logdata.timer_[logdata.idx_] = timer;
		logdata.phase_[logdata.idx_] = phase;
		logdata.right_ref_[logdata.idx_] = right_ref;
		logdata.left_ref_[logdata.idx_] = left_ref;
		logdata.right_out_[logdata.idx_] = right_out;
		logdata.left_out_[logdata.idx_] = left_out;
		logdata.right_spd_[logdata.idx_] = right_spd;
		logdata.left_spd_[logdata.idx_] = left_spd;
		logdata.right_enc_[logdata.idx_] = right_enc;
		logdata.left_enc_[logdata.idx_] = left_enc;
		logdata.idx_ ++;
	}
#endif
}


void init_motor_test(){
/*
 * 状態変数初期化
 */
	timer = 0;
	start_time = freerun;
	phase = 0;
	phase_timer = 0;
	phase_start_time = freerun;
	right_ref = 0;
	left_ref = 0;
	right_out = 0;
	left_out = 0;
	right_spd = 0;
	left_spd = 0;
	right_enc = 0;
	left_enc = 0;
}


void init_test_log(){
/*
 * ログ変数初期化
 */
#if WHEEL_TEST
	logdata.idx_ = 0;
	for(int i = 0; i < LOGSIZE_TEST; i++){
		logdata.timer_[i] = 0;
		logdata.phase_[i] = 0;
		logdata.right_ref_[i] = 0;
		logdata.left_ref_[i] = 0;
		logdata.right_out_[i] = 0;
		logdata.left_out_[i] = 0;
		logdata.right_spd_[i] = 0;
		logdata.left_spd_[i] = 0;
		logdata.right_enc_[i] = 0;
		logdata.left_enc_[i] = 0;
	}
#endif
}

void wheel_test_log_dump() {
/*
 * ログの出力 mainからコールされる
 */
#if WHEEL_TEST
	extern void uart2_transmitte(char *p);

	char buf[600];

	memset(buf, 0x00, sizeof(buf));
	sprintf(buf, "\r\ntimer, phase, right_ref, left_ref, right_out, left_out, right_spd, left_spd, right_enc, left_enc\r\n");
	uart2_transmitte(buf);

	memset(buf, 0x00, sizeof(buf));
	sprintf(buf, "[msec], [-], [m/s], [m/s], [-], [-], [m/s], [m/s], [-], [-]\r\n");
	uart2_transmitte(buf);

	for(int i = 0; i < LOGSIZE_TEST; i++) {
		if (logdata.timer_[i] == 0 && i != 0){
			break;
		}
		memset(buf, 0x00, sizeof(buf));
		//             1   2   3   4   5   6   7,  8   9  10
		sprintf(buf, "%d, %d, %f, %f, %d, %d, %f, %f, %d, %d\r\n",
				logdata.timer_[i],		//1
				logdata.phase_[i],		//2
				logdata.right_ref_[i],	//3
				logdata.left_ref_[i],	//4
				logdata.right_out_[i],	//5
				logdata.left_out_[i],	//6
				logdata.right_spd_[i],	//7
				logdata.left_spd_[i],	//8
				logdata.right_enc_[i],	//9
				logdata.left_enc_[i]	//10
				);
		uart2_transmitte(buf);
	}
#endif
	set_wheel_test_dump_comp();
}


void wheel_test_status_ctrl(){
/*
 * テスト時のステータス制御
 */
	switch (test_status){
		case TEST_READY:
			if (flg_wheel_test_req == 1){
				test_status = TEST_ING;
				set_test_start_flg();
			}
			break;

		case TEST_ING:
			reset_test_start_flg();
			if (phase >= MAX_PHASE || flg_wheel_test_req != 1){
				test_status = TEST_COMP;
			}
			break;

		case TEST_COMP:
			if (flg_wheel_test_req != 1){
				test_status = TEST_READY;
			}
			break;
	}
}


void wheel_test_main(void){
/*
 * テストのメイン関数
 * 10ms周期でmainからコールされる
 */
	if (get_RxCommand() == 0x30){
		// Require Wheel Motor test
		flg_wheel_test_req = 1;
	}
	else{
		flg_wheel_test_req = 0;
	}

	wheel_test_status_ctrl();

	switch (test_status){
	case TEST_READY:
		set_motor_ref_r(0);
		set_motor_ref_l(0);
		break;

	case TEST_ING:
		if(get_test_start_flg() == 1 ){	// TEST_INGに移行した1JOB目のみ実行
			init_motor_test();
			init_test_log();
		}
		update_timer();
		motor_spd_decision();
		set_motor_ref_r(right_ref);
		set_motor_ref_l(left_ref);
		wheel_enc_count();
		wheel_spd_measure();
		test_log_update();
		break;

	case TEST_COMP:
		set_motor_ref_r(0);
		set_motor_ref_l(0);
		break;
	}
}
