/*
 * turn.h
 *
 *  Created on: 2019/05/31
 *      Author: ayabe
 */

#ifndef TURN_H_
#define TURN_H_

void t_set_brake(void);
void t_release_brake(void);
void set_rotation_cw(void);
void set_rotation_ccw(void);
void turn_cntrl(void);
void turn_init(void);
void turn_relay_on(void);
void turn_relay_off(void);
void turn_alarm_reset(void);
void turn_alarm_recover(void);
void turn_motor_stop(void);
void turn_sensor_check(void);
void scan_turnmotor_alarm(void);
void turn_set_speed(uint16_t dac);
uint8_t ck_turn_motor_alarm(void);
void set_turn_flg_send_comp(void);
void reset_turn_flg_send_comp(void);
uint8_t get_turn_flg_send_comp(void);
uint8_t get_turn_status(void);
void turn_log_dump(void);
void set_turn_dump_req(void);
void reset_turn_dump_req(void);
uint8_t get_turn_dump_req(void);
void set_turn_dump_comp(void);
void reset_turn_dump_comp(void);
uint8_t get_turn_dump_comp(void);

#define TURN_ANG0_SENSOR	0x01
#define TURN_ANG90_SENSOR	0x02
#define TURN_ANG180_SENSOR	0x04
#define TURN_ANG270_SENSOR	0x08

//NUC通知用ステータス
#define ST_TURN_VEL_RUN		0x00	// 速度制御モード
#define ST_TURN_ANG_RUN		0x01	// 角度制御モード実行中
#define ST_TURN_ANG_COMP	0x02	// 角度制御モード正常終了
#define ST_TURN_ANG_ERRCOMP	0x03	// 角度制御モード異常終了
//NUCからの指示コマンド
#define TURN_VEL_MODE		0x00	// 速度指示モード
#define TURN_ANG_MODE		0x12	// 角度指示モード
#define TURN_NO_REQ			0x00	// 角度指示なし
#define TURN_0				0x01	// 0度の位置へ回転
#define TURN_90				0x02	// 90度の位置へ回転
#define TURN_180			0x03	// 180度の位置へ回転
#define TURN_270			0x04	// 270度の位置へ回転
#define TURN_LOW			0x00	// 低速で回転
#define TURN_MID			0x01	// 中速で回転
#define TURN_HIGH			0x02	// 高速で回転
//回転量のパラメータ
typedef struct turn_ctrl_pram{
	double		reach_angle;	// 1.回転量(deg)
	double		v_max;			// 2.最高速(rad/s)
	double		t_acc;			// 3.加速時間(sec)
	double		t_decel;		// 4.減速時間(sec)
	double		brake_angle;	// 5.ブレーキ開始角(deg)
	double		approach_vel;	// 6.アプローチ速度(rad/s)
}TURN_CTRL_PRAM;

//状態量
typedef struct turn_variable{
	uint16_t	time;
	uint8_t		flg_cruise_end;
	double		cruise_end_time;
	double		trgt_angle;
	double		trgt_vel;
	double		trgt_acc;
	double		act_angle;
	double		act_vel;
	double		act_vel_old;
	double		act_vel_lpf;
	double		err_angle;
	double		err_vel;
	double		err_vel_old;
	double		err_vel_lpf;
	double		ref_vel;
	double		ref_vel_ff;
	double		ref_vel_fb;
	double		d_term;
	double		init_deff;		// 開始時の角度
	double		sens_angle;		// ターゲットのフォトセンサが反応した時の回転角
	double		result_angle;	// フォトセンサ反応後の角度(ターン結果の出力用)
	uint8_t		recover_cnt;	// リカバリー動作実施回数
}TRUN_VARIABLE;

#endif /* TURN_H_ */
