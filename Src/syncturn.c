/*

 * syncturn.c
 *
 *  Created on: 2020/11/27
 *      Author: ROBO TAKUMI
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "Cntrl_Proc.h"
#include "syncturn.h"
#include "nuc.h"
#include "stm32f4xx_it.h"
#include "wheel.h"
#include "turn.h"
#include "common_func.h"
#include "conf.h"

// 一般設定
#define TimeIntSync_ms	10			//制御周期(msec)
#define TimeIntSync_sec	(double)TimeIntSync_ms / 1000.0	//制御周期(sec)
#define TIM_50MSEC	5
#define LOGSIZE  800 				// ログ点数
#define	SYNC_ERROR_ANGLE 50 		// シンクロターン完了時の許容誤差(deg/10)
// 動輪
#define WHEEL_RATIO		4.9			// ホイールベース/動輪直径
// ターンテーブル
#define WHAT_TURNTBL_V(av)		2.344 * (av)	// ターンテーブル角速度→指示電圧 変換式
#define SYNC_TURNTBLIMPVOLT_DAC(volt)	(int32_t)((double)(4095 / 5.0) * (volt)) // 指示電圧→DAC指示値への変換式
#define TIM_CYCLE	24.39024		// タイマーカウントアップ周期(secを10^6倍)
#define TIM_CYCLE_TTBL	1.2195122	// タイマーカウントアップ周期_ターンテーブル用(secを10^6倍)
// ターン種別によらない共通キャリブレーション
#if SYNC_RESULT_OUT == 1
#define WAIT_TIME 1000				// ブレーキ後待機時間 ms(テスト用)
#else
#define WAIT_TIME 250				// ブレーキ後待機時間 ms
#endif
#define STOP_PALSE_SYNC	4			// ブレーキを許可するエンコーダ更新数
#define FB_OFF_ANGLE 0				// フィードバック縮退開始する角度 deg/10
#define FB_RELEASE_GRAD 0.0005 		// フィードバック縮退勾配 rad/s/10ms
#define APPROACH_VEL	0.05		// ターン終了間際の微調整速度 rad/s
#define BODY_VEL_FIR 0.05			// 車体角速度にかけるLPF時定数(sec)
#define TTBL_VEL_FIR 0.05			// ターンテーブル角速度にかけるLPF時定数(sec)
#define ERR_ANG_FIR 0.05			// 角度の偏差にかけるLPF時定数(sec)
#define ERR_VEL_FIR 0.05			// 角速度の偏差にかけるLPF時定数(sec)
// カルマンフィルタ用キャリブレーション
#define VER_COEF 0.1				// 0に近いほどP&Fの情報を重視　0より大きく1より小さい値を設定
#define ALPHA_KL 0.0976				// P&Fセンサ角の遅れ係数(rad/rad/s)
// ターン種別ごとのキャリブレーション定数
static SYNC_CTRL_TBL *pTurnCalib;
//---LOW SPEED--- 90TURN
static SYNC_CTRL_TBL turn90_low_calib  = {
		(PI * 0.5 - 6.0 * PI / 180.0),	//1.ターン量 (rad)
		3.2,				//2.ターン時間(sec)
		5,					//3.ブレーキ開始車体角 (deg/10)
		4.0 * PI / 180.0	//4.テーブルセンサー反応後、テーブルを進める距離 rad
};
//---MID SPEED--- 90TURN
static SYNC_CTRL_TBL turn90_mid_calib  = {
		(PI * 0.5 - 6.0 * PI / 180.0),	//1.ターン量 (rad)
		3.2,				//2.ターン時間(sec)
		5,					//3.ブレーキ開始車体角 (deg/10)
		4.0 * PI / 180.0	//4.テーブルセンサー反応後、テーブルを進める距離 rad
};
//---HIGH SPEED--- 90TURN
static SYNC_CTRL_TBL turn90_high_calib  = {
		(PI * 0.5 - 6.0 * PI / 180.0),	//1.ターン量 (rad)
		3.2,				//2.ターン時間(sec)
		5,					//3.ブレーキ開始車体角 (deg/10)
		4.0 * PI / 180.0	//4.テーブルセンサー反応後、テーブルを進める距離 rad
};
//---LOW SPEED--- 180TURN
static SYNC_CTRL_TBL turn180_low_calib = {
		(PI - 9.0 * PI / 180.0),	//1.ターン量 (rad)
		5.0,				//2.ターン時間(sec)
		3,					//3.ブレーキ開始車体角 (deg/10)
		4.0 * PI / 180.0	//4.テーブルセンサー反応後、テーブルを進める距離 rad
};
//---MID SPEED--- 180TURN
static SYNC_CTRL_TBL turn180_mid_calib = {
		(PI - 9.0 * PI / 180.0),	//1.ターン量 (rad)
		5.0,				//2.ターン時間(sec)
		3,					//3.ブレーキ開始車体角 (deg/10)
		4.0 * PI / 180.0	//4.テーブルセンサー反応後、テーブルを進める距離 rad
};
//---HIGH SPEED--- 180TURN
static SYNC_CTRL_TBL turn180_high_calib = {
		(PI - 9.0 * PI / 180.0),	//1.ターン量 (rad)
		5.0,				//2.ターン時間(sec)
		3,					//3.ブレーキ開始車体角 (deg/10)
		4.0 * PI / 180.0	//4.テーブルセンサー反応後、テーブルを進める距離 rad
};

//フィードバックパラメータ
static SYNC_FB_PRAM *pFB_Calib;
static SYNC_FB_PRAM fb_pram = {
		0.3,	//1.車体FB 比例ゲインKp
		0.3,	//2.車体FB 積分時間Ti
		0.0,	//3.車体FB 微分時間Td
		0.5,	//4.車体FB 微分係数
		0.3,	//5.ターンテーブルFB 比例ゲインKp
		100.0,	//6.ターンテーブルFB 積分時間Ti
		0.2,	//7.ターンテーブルFB 微分時間Td
		0.2		//8.ターンテーブルFB 微分係数
};

//逆モデルパラメータ
static SYNC_INV_MDL *pInv_Model_Calib;
static SYNC_INV_MDL inv_model_param_90 = {
		0.9366,	//1.車体gain rise
		6.3421,	//2.車体omega rise
		0.1891,	//3.車体zeta rise
		0.9366,	//4.車体gain fall
		6.3421,	//5.車体omega fall
		0.1891,	//6.車体zeta fall
		0.9016,	//7.ターンテーブルgain rise
		5.9773,	//8.ターンテーブルomega rise
		0.5131,	//9.ターンテーブルzeta rise
		0.9016, //10.ターンテーブルgain fall
		6.004,	//11.ターンテーブルomega fall
		0.6411	//12.ターンテーブルzeta fall
};

static SYNC_INV_MDL inv_model_param_180 = {
		0.9396,	//1.車体gain rise
		6.6875,	//2.車体omega rise
		0.2038,	//3.車体zeta rise
		0.9396,	//4.車体gain fall
		6.6875,	//5.車体omega fall
		0.2038,	//6.車体zeta fall
		0.9021,	//7.ターンテーブルgain rise
		5.9787,	//8.ターンテーブルomega rise
		0.5152,	//9.ターンテーブルzeta rise
		0.9021, //10.ターンテーブルgain fall
		6.0048,	//11.ターンテーブルomega fall
		0.64	//12.ターンテーブルzeta fall
};


//ステータス
static uint8_t	g_syncturn_status = SYNC_TURN_READY;		//シンクロターンステータス
//フラグ
static uint8_t 	g_flg_send_comp = 0;			//NUCへシンクロターン完了フラグを送信済で1
static uint8_t	g_flg_approach	= 0;			//目標軌跡がアプローチに入ったら1
static uint8_t 	g_flg_brk_angle_body = 0;		//車体ブレーキ開始角を超えたら1
static uint8_t 	g_flg_brk_angle_ttbl = 0;		//ターンテーブルブレーキ開始角を超えたら1
static uint8_t	g_flg_syncturn_start = 0;		//シンクロターン開始時に1
static uint8_t	g_flg_illg_command = 0;			//NUCから受信したコマンドが不正な値の時に1

static uint8_t	g_flg_ttbl_sens_aprch = 0;		//ターンテーブルがセンサーに到達で1
static uint8_t	g_flg_brake_permission = 0;		//ブレーキをかけて良いスピードなら1
static uint8_t	g_flg_body_stp_jdg = 0;			//パルス停止判定(車輪)
static uint8_t	g_flg_ttbl_stp_jdg = 0;			//パルス停止判定(ターンテーブル)
static uint8_t	g_flg_sync_dump_req = 0;		//ログ出力要求ありで1
static uint8_t	g_flg_sync_dump_comp = 0;		//ログ出力終了で1
//NUCからの指示
static uint8_t	g_command = 0;		//シンクロターン実施指示
static uint8_t	g_sync_spd = 0;		//シンクロターン速度指示
static uint8_t	g_sync_dir = 0;		//シンクロターン回転方向指示
//タイマ等
static uint32_t	g_time = 0;				//開始からの時間[ms]
static uint16_t	g_time_from_brk = 0; 	//ブレーキ後の経過時間
//P&Fセンサ
static uint16_t	g_pf_angle = 0;			//P&Fセンサ角
static uint16_t g_pf_angle_old = 0;		//P&Fセンサ角の前回値
static uint16_t	g_pf_start_angle = 0;	//ターン開始時のP&Fセンサ角 [deg/10]
static int16_t	g_diff_rad = 0;			//初期角のずれ rad/1000
static int16_t	g_pf_angle_dist  = 0;	//ターン開始時を0としたP&Fセンサ角 [deg/10]
static double	g_pf_angle_dist_ln = 0;	//P&Fセンサ未更新時の補間処理後[rad]
static int8_t	g_pf_nup_count	= 0;	//P&Fセンサ未更新回数のカウンタ
static int8_t	g_pf_count = 0;			//P&Fセンサが3600と0の境界をまたいだ回数
static int16_t	g_target_dist = 0;		//ターン開始時を基準としたターゲット角 [deg/10]
//シンクロターン用エンコーダカウンタ
static int16_t	g_wheel_enccnt_left = 0;
static int16_t	g_wheel_enccnt_right = 0;
static int16_t	g_turntbl_enccnt = 0;
static uint16_t g_wheel_encoder_left_old = 0;
static uint16_t g_wheel_encoder_right_old = 0;
static uint16_t g_ttbl_encoder_old = 0;
//目標軌跡
static double	g_trgt_angle = 0;			//目標角度[rad]
static double	g_trgt_vel = 0;				//目標角速度[rad/s]
static double	g_trgt_acc = 0;				//目標角加速度[rad/s^2]
static double	g_trgt_jerk = 0;			//目標躍度[rad/s^3]
//車体角速度の指示値
static double	g_ref_body_vel = 0;			//最終指示値[rad/s]
static double	g_ref_body_vel_ff = 0;		//フィードフォワード項[rad/s]
static double	g_d_term_body = 0;
static double	g_d_term_body_old = 0;
//ターンテーブル角速度の指示値
static double	g_ref_ttbl_vel = 0;			//最終指示値[rad/s]
static double	g_ref_ttbl_vel_ff = 0;		//フィードフォワード項[rad/s]
static double	g_d_term_ttbl = 0;
static double	g_d_term_ttbl_old = 0;
//エンコーダから取得した状態量
static double	g_act_body_angle = 0;		//車体角[rad]
static double	g_act_body_vel   = 0;		//車体角速度[rad/s]
static double	g_act_body_vel_lpf   = 0;	//車体角速度(LPF後)[rad/s]
static double	g_act_body_vel_lpf_old = 0;	//車体角速度(LPF後)前回値[rad/s]
static double	g_act_ttbl_angle = 0;		//ターンテーブル角[rad]
static double	g_act_ttbl_vel   = 0;		//ターンテーブル角速度[rad/s]
static double	g_act_ttbl_vel_lpf   = 0;	//ターンテーブル角速度(LPF後)[rad/s]
static double	g_act_ttbl_vel_lpf_old = 0;	//ターンテーブル角速度(LPF後)前回値[rad/s]
static double	g_act_err_angle  = 0;		//シンクロずれ角[rad]
static double	g_act_err_angle_old  = 0;	//シンクロずれ角[rad]前回値
static double	g_act_err_angle_lpf  = 0;	//シンクロずれ角[rad]LPF後
static double	g_act_err_vel		= 0;	//シンクロずれ角速度[rad/s]
static double	g_act_err_vel_old 	= 0;	//シンクロずれ角速度[rad/s]
static double	g_act_err_vel_lpf	= 0;	//シンクロずれ角速度(LPF後)[rad/s]
static double	g_ttbl_enc_sens		= 0;	//フォトセンサが反応した時の角度[rad]
//カルマンフィルタ
static double	g_p_body_angle		= 0;	//状態量推定誤差の分散
static double	g_est_body_angle	= 0;	//カルマンフィルタにより推定した車体角[rad]
//目標軌跡と車体・ターンテーブルの偏差
static double	g_act_body_err_angle	= 0;
static double	g_act_body_err_vel		= 0;
static double	g_act_body_err_vel_old	= 0;
static double	g_act_body_err_vel_lpf	= 0;
static double	g_act_body_err_acc		= 0;
static double	g_act_ttbl_err_angle	= 0;
static double	g_act_ttbl_err_vel		= 0;
static double	g_act_ttbl_err_vel_old	= 0;
static double	g_act_ttbl_err_vel_lpf	= 0;
static double	g_act_ttbl_err_acc		= 0;
//回転方向
static int8_t	g_ttbl_dir = 0;				//ターンテーブル回転方向：右回りを正
static int8_t	g_body_dir = 0;				//車体回転方向：右回りを正
//ターゲット
static uint8_t	g_ttbl_sens_target_pos = 0;	//ターンテーブルのターゲットセンサー位置
static uint16_t	g_target_pf_angle = 0;		//車体角度ターゲット
//検査用ログ変数
static int16_t	g_result_max_err = 0;		//シンクロ誤差の最大値
static int16_t	g_result_ttbl_angle	= 0;	//ターン前後でターンテーブルが動いた角度


#if SYNC_TURN_LOG_DUMP
// 開発用ログ変数
uint16_t	g_log_idx = 0;
uint32_t	g_time_log[LOGSIZE] = {0};
uint16_t	g_pf_angle_log[LOGSIZE] = {0};
double		g_pf_angle_dist_ln_log[LOGSIZE] = {0};
uint8_t		g_flg_brk_angle_body_log[LOGSIZE] = {0};
uint8_t		g_flg_brk_angle_ttbl_log[LOGSIZE] = {0};
double		g_trgt_angle_log[LOGSIZE] = {0};
double		g_trgt_vel_log[LOGSIZE] = {0};
double		g_ref_body_vel_log[LOGSIZE] = {0};
double		g_ref_ttbl_vel_log[LOGSIZE] = {0};
double		g_act_body_angle_log[LOGSIZE] = {0};
double		g_act_body_vel_lpf_log[LOGSIZE] = {0};
double		g_act_ttbl_angle_log[LOGSIZE] = {0};
double		g_act_ttbl_vel_lpf_log[LOGSIZE] = {0};
double		g_act_err_angle_log[LOGSIZE] = {0};
double		g_act_err_vel_lpf_log[LOGSIZE] = {0};
double		g_est_body_angle_log[LOGSIZE] = {0};
#endif


/*
 * sycturn_direction
 * 回転方向を設定する。
 * int32_t dir：　回転方向
 */
static void sycturn_direction(uint8_t dir, int32_t diffrad) {

	switch(dir) {
	case L90T:		//左９０度ターン
		//左タイヤ逆回転、右タイヤ正回転
		g_body_dir = -1;
//		wheel_set_dir(-1, 1, 0, 0);

		//ターンテーブルは右周り
		g_ttbl_dir = 1;
		set_rotation_cw();
		break;

	case R90T:		//右９０度ターン
		//左タイヤ正回転、右タイヤ逆回転
		g_body_dir = 1;
//		wheel_set_dir(1, -1, 0, 0);
		
		//ターンテーブルは左周り
		g_ttbl_dir = -1;
		set_rotation_ccw();
		break;

	case RL180T:
		if (diffrad < 0) {
			//左ターン
			//左タイヤ逆回転、右タイヤ正回転
			g_body_dir = -1;
//			wheel_set_dir(-1, 1, 0, 0);
			//ターンテーブルは右周り
			g_ttbl_dir = 1;
			set_rotation_cw();
		}
		else {
			//右ターン
			//左タイヤ正回転、右タイヤ逆回転
			g_body_dir = 1;
//			wheel_set_dir(1, -1, 0, 0);
			//ターンテーブルは左周り
			g_ttbl_dir = -1;
			set_rotation_ccw();
		}
		break;

	case STOP:
		//停止
//		wheel_set_dir(0, 0, 0, 0);
		turn_motor_stop();
	}
}

/*
 * conv_wheel_ref
 * 走行用モーターの角速度指示をNUC指示値に変換する
 */
static int16_t conv_wheel_ref(double speed){
	int16_t ref = 0;
	ref = (short)((double)WHEEL_DIAMETER * 0.5 * speed * (double)MAX_NUC_REF / (double)MAX_NUC_SPEED_WHEEL);
	return ref;
}


/*
 * conv_motervolt2dac
 * モータの種別に応じて入力電圧値をDAC指示値に変換する
 * int kind:	モータ種別
 * double volt:	入力電圧
 * 戻り値：DAC指示値
 */
static int32_t conv_motervolt2dac(int kinds, double volt) {

	int32_t	retDAC;

	if (kinds == SYNC_WHEEL) {
		//TAG1000はwheel.cを介して走行用モーターに指示を出すのでsyncturn.cで電圧変換はしない
	}
	else {
		retDAC = SYNC_TURNTBLIMPVOLT_DAC(volt);
	}

	return retDAC;
}


/*
 * turntbl_motor_ctrl
 * ターンテーブル制御
 */
static void turntbl_motor_ctrl(int32_t dacval) {
	turn_set_speed((uint16_t)dacval);
}


/*
 * calc_turn_locus
 * 目標軌跡の計算
 */
static void calc_turn_locus() {
	// 初期角に応じてターン量を増減
	double reach_angle_adj = 0;
	if (g_sync_dir == R90T || g_sync_dir == L90T){
		// 90度ターン
		reach_angle_adj = (double)g_diff_rad * 0.001;
	}
	else{
		// 180度ターン
		if(g_body_dir == 1){
			// 右ターン
			reach_angle_adj = -1.0 * (double)g_diff_rad * 0.001;
		}
		else{
			// 左ターン
			reach_angle_adj = (double)g_diff_rad * 0.001;
		}
	}


	// ターン量とターン時間から正弦波のパラメータを算出
	double amp = (pTurnCalib->reach_angle + reach_angle_adj)/ pTurnCalib->turn_time - APPROACH_VEL/4.0;
	double amp_adj = APPROACH_VEL / 2.0;
	double omega = 2 * PI / pTurnCalib->turn_time;

	if ((omega * g_time * 0.001) < PI){
		// 加速区間
		g_trgt_vel =  amp * mysin(omega * g_time * 0.001 - PI * 0.5) + amp;
		g_trgt_acc = amp * omega * mysin(omega * g_time * 0.001);
		g_trgt_jerk = -1.0 * amp * omega * omega * mysin(omega * g_time * 0.001 - PI * 0.5);
		g_flg_approach = 0;
	}
	else if ((omega * g_time * 0.001) < (2.0 * PI)){
		// 減速区間
		g_trgt_vel =  amp * mysin(omega * g_time * 0.001 - PI * 0.5) + amp + amp_adj * mysin(omega * g_time * 0.001 + PI * 0.5) + amp_adj;
		g_trgt_acc = amp * omega * mysin(omega * g_time * 0.001) - amp_adj * omega * mysin(omega * g_time * 0.001);
		g_trgt_jerk = -1.0 * amp * omega * omega * mysin(omega * g_time * 0.001 - PI * 0.5) - 1.0 * amp_adj * omega * omega * mysin(omega * g_time * 0.001 + PI * 0.5);
		g_flg_approach = 0;
	}
	else{
		// 微調整区間
		g_trgt_vel = APPROACH_VEL;
		g_trgt_acc = 0.0;
		g_trgt_jerk = 0.0;
		g_flg_approach = 1;
	}

	// ターゲット角
	g_trgt_angle += g_trgt_vel * 0.01;
}


/*
 * calc_d_term
 * 改良形双一次変換を用いた不完全微分によりPIDのD成分を算出する
 */
static double calc_d_term(double d_term_old, double err, double err_old, double Td, double eta){
	//d_term_old:d項の前回値
	//err:偏差（速度）の今回値
	//err_old:偏差(速度)の前回値
	//Td:微分時間(=Dゲイン/Pゲイン)
	//eta:微分係数(0.1~0.125推奨)

	double delta_t = 0.01;	//制御周期[sec]
	double d_term = d_term_old + (2 * Td * (err - err_old)) / (delta_t + 2 * eta * Td)  - (2 * delta_t * d_term_old) / (delta_t + 2 * eta * Td);

	return d_term;
}


/*
 * calc_FF_body
 * 車体の逆モデルを解いて角速度指示値を計算する
 */
static double calc_FF_body(){

	static double ret = 0;
	double gain_body = 0;
	double omega_body = 0;
	double zeta_body = 0;

	if (g_trgt_acc >= 0){
		// ターゲット角加速度が正
		gain_body = pInv_Model_Calib->inv_mdl_body_gain_rise;
		omega_body = pInv_Model_Calib->inv_mdl_body_omega_rise;
		zeta_body = pInv_Model_Calib->inv_mdl_body_zeta_rise;
	}
	else{
		// ターゲット角加速度が負
		gain_body = pInv_Model_Calib->inv_mdl_body_gain_fall;
		omega_body = pInv_Model_Calib->inv_mdl_body_omega_fall;
		zeta_body = pInv_Model_Calib->inv_mdl_body_zeta_fall;
	}

	if (g_flg_approach == 0){
		ret = 1.0 / (gain_body * omega_body * omega_body) * (g_trgt_jerk + 2.0 * zeta_body * omega_body * g_trgt_acc + omega_body * omega_body * g_trgt_vel);
	}
	else{;}	// 速度指示値の段付き防止のため、アプローチ時はFF項を更新しない

	return ret;
}


/*
 * calc_FF_ttbl
 * ターンテーブルの逆モデルを解いて角速度指示値を計算する
 */
static double calc_FF_ttbl(){

	static double ret = 0;
	double gain_ttbl = 0.0;
	double omega_ttbl = 0;
	double zeta_ttbl = 0;

	if (g_trgt_acc >= 0){
		// ターゲット角加速度が正
		gain_ttbl = pInv_Model_Calib->inv_mdl_ttbl_gain_rise;
		omega_ttbl = pInv_Model_Calib->inv_mdl_ttbl_omega_rise;
		zeta_ttbl = pInv_Model_Calib->inv_mdl_ttbl_zeta_rise;
	}
	else{
		// ターゲット角加速度が負
		gain_ttbl = pInv_Model_Calib->inv_mdl_ttbl_gain_fall;
		omega_ttbl = pInv_Model_Calib->inv_mdl_ttbl_omega_fall;
		zeta_ttbl = pInv_Model_Calib->inv_mdl_ttbl_zeta_fall;
	}

	if (g_flg_approach == 0){
		ret = 1.0 / (gain_ttbl * omega_ttbl * omega_ttbl) * (g_trgt_jerk + 2.0 * zeta_ttbl * omega_ttbl * g_trgt_acc + omega_ttbl * omega_ttbl * g_trgt_vel);
	}
	else{;}	// 速度指示値の段付き防止のため、アプローチ時はFF項を更新しない

	return ret;
}


/*
 * syncturn_result
 * シンクロターンの結果を判定
 */
static uint8_t syncturn_result() {

	uint16_t diff;
	uint8_t ret = 0;
	uint8_t ttbl_pos = get_turn_sensor_pos();

	if ((g_target_pf_angle == 3600) || (g_target_pf_angle == 0)) {
		if (g_pf_angle < 180) {
			diff = abs(g_pf_angle - 0);
		} else {
			diff = abs(3600 - g_pf_angle);
		}
	} else {
		diff = abs(g_pf_angle - g_target_pf_angle);
	}

	// NUCへ通知するためのシンクロターン結果判定
	// return 1:　規定内の角度でターン完了, return 0： 規定外の角度でターン完了
	if (((g_ttbl_sens_target_pos & ttbl_pos) != 0) && (diff < SYNC_ERROR_ANGLE)) {
		ret = 1;
	}

	// ターン開始前と終了後でターンテーブルがグローバル座標系に対して何度動いたか
	double angle_tmp = (double)g_pf_angle_dist / 10.0 - g_act_ttbl_angle * 180.0 / PI;
	g_result_ttbl_angle = (int16_t)(angle_tmp * 10.0);
	return ret;
}


/*
 * sync_motor_ctrl
 * シンクロターンモータ制御
 */
static void sync_motor_ctrl() {
	///////////////////
	//走行用モーターの制御//
	///////////////////
	double Kp_body = pFB_Calib->Kp_body;	// 比例ゲイン
	double Ti_body = pFB_Calib->Ti_body;	// 積分時間
	double Td_body = pFB_Calib->Td_body;	// 微分時間
	double eta_body = pFB_Calib->eta_body;	// 微分係数

	if(g_flg_brk_angle_body < 1){
		//FF項計算
		g_ref_body_vel_ff = calc_FF_body();
		//FB項計算
		g_d_term_body = calc_d_term(g_d_term_body, g_act_err_vel, g_act_err_vel_old, Td_body, eta_body);
		double ref_body_vel_fb = g_act_err_vel_lpf * Kp_body + Kp_body * g_d_term_body + Kp_body / Ti_body * g_act_err_angle;
		//FF項とFB項を足す
		g_ref_body_vel = g_ref_body_vel_ff + ref_body_vel_fb;

		//車体角速度指示をタイヤ角速度指示に変換
		double wheel_angle_v = WHEEL_RATIO * g_ref_body_vel;
		//タイヤ角速度指示値をNUC指示値に変換
		int16_t wheel_ref = conv_wheel_ref(wheel_angle_v);
		if (wheel_ref < 0){
			wheel_ref = 0;
		}
		else{;}
		//モーター制御
//		wheel_set_freerun(0);
//		wheel_set_brake(0);
//		wheel_set_dc_lock(0);
		//wheel_set_speed(wheel_ref, wheel_ref);
	}
	else{						//ブレーキ区間
//		wheel_set_freerun(0);
//		wheel_set_dc_lock(0);
		//wheel_set_speed(0, 0);
		if(g_flg_brake_permission > 0){
			// 速度がしきい値以下になってからブレーキをかける
//			wheel_set_brake(1);
		}
		else{
//			wheel_set_brake(0);
		}
	}

	///////////////////////
	//ターンテーブルモーターの制御//
	///////////////////////
	double Kp_ttbl = pFB_Calib->Kp_ttbl;	// 比例ゲイン
	double Ti_ttbl = pFB_Calib->Ti_ttbl;	// 積分時間
	double Td_ttbl = pFB_Calib->Td_ttbl;	// 微分時間
	double eta_ttbl = pFB_Calib->eta_ttbl;	// 微分係数
	if(g_flg_brk_angle_ttbl < 1 ){
		//FF項計算
		g_ref_ttbl_vel_ff = calc_FF_ttbl();
		//FB項計算
		g_d_term_ttbl = calc_d_term(g_d_term_ttbl, g_act_err_vel, g_act_err_vel_old, Td_ttbl, eta_ttbl);
		double ref_ttbl_vel_fb = g_act_err_vel * Kp_ttbl + Kp_ttbl * g_d_term_ttbl + Kp_ttbl / Ti_ttbl * g_act_err_angle;
		//FF項とFB項を足す
		g_ref_ttbl_vel = g_ref_ttbl_vel_ff - ref_ttbl_vel_fb;
		//角速度から電圧に変換
		volatile double turntable_input_volt = WHAT_TURNTBL_V(g_ref_ttbl_vel);
		if (turntable_input_volt < 0){
			turntable_input_volt = 0;
		}
		else{;}
		//電圧をDAC指示値に変換
		int32_t turntbl_motor_dac = conv_motervolt2dac(TURN_TABLE, turntable_input_volt);
		turntbl_motor_ctrl(turntbl_motor_dac);
	}
	else{				//ブレーキ区間
		turntbl_motor_ctrl(0);
		turn_motor_stop();
		t_set_brake();
	}
}


/*
 * calc_angle
 * エンコーダパルス数から現在の角度を取得する
 */
static void calc_angle(){
	double body_angle_per_pulse = 2 * PI / (WHEEL_RATIO * PULS_WHEEL);	// エンコーダ1パルス当たりの車体回転角(rad)
	int32_t wheel_angle_ave_l = (abs(g_wheel_enccnt_left) + abs(g_wheel_enccnt_right) + 1) / 2;
	g_act_body_angle = (double)wheel_angle_ave_l * body_angle_per_pulse;
	g_act_ttbl_angle = (2.0 * PI * (double)g_turntbl_enccnt) / TURNTBL_ENC *(double)g_ttbl_dir;
}


/*
 * calc_err_angle
 * 目標軌道に対する車体角とターンテーブル角の差を計算
 */
static void calc_err_angle(){
	// ターンテーブルと車体角の差を計算
	g_act_err_angle_old = g_act_err_angle;
	g_act_err_angle = g_act_ttbl_angle - g_est_body_angle;
	g_act_err_vel_old = g_act_err_vel;
	g_act_err_vel = g_act_ttbl_vel - g_act_body_vel; // [rad/s]
	// ローパスフィルタ処理
	g_act_err_angle_lpf = LPF_1order(g_act_err_angle, g_act_err_angle_old, g_act_err_angle_lpf, ERR_ANG_FIR, TimeIntSync_sec); // [rad/s]
	g_act_err_vel_lpf = LPF_1order(g_act_err_vel, g_act_err_vel_old, g_act_err_vel_lpf, ERR_VEL_FIR, TimeIntSync_sec); // [rad/s]

	// 目標軌道に対する差を計算
	g_act_body_err_angle	= g_trgt_angle - g_act_body_angle;
	g_act_body_err_vel_old	= g_act_body_err_vel;
	g_act_body_err_vel		= g_trgt_vel   - g_act_body_vel;
	g_act_body_err_vel_lpf	= LPF_1order(g_act_body_err_vel, g_act_body_err_vel_old, g_act_body_err_vel_lpf, BODY_VEL_FIR, TimeIntSync_sec);

	g_act_ttbl_err_angle	= g_trgt_angle - g_act_ttbl_angle;
	g_act_ttbl_err_vel_old	= g_act_ttbl_err_vel;
	g_act_ttbl_err_vel		= g_trgt_vel   - g_act_ttbl_vel;
	g_act_ttbl_err_vel_lpf	= LPF_1order(g_act_ttbl_err_vel, g_act_ttbl_err_vel_old, g_act_ttbl_err_vel_lpf, TTBL_VEL_FIR, TimeIntSync_sec);
}


/*
 * calc_pf_dist
 * 初期状態を基準としたP&Fセンサの変位角を求める
 */
static void calc_pf_dist(){
	if ((g_pf_angle != g_pf_angle_old) && (get_sync_start_flg() != 1)){
		// 0度を跨いだ回数をカウント
		if((g_pf_angle_old > 2700) && (g_pf_angle < 900)){
			g_pf_count ++;
		}
		else if((g_pf_angle_old < 900) && (g_pf_angle > 2700)){
			g_pf_count --;
		}

		// 初期角を基準としたP&Fセンサの変位角を算出
		if (g_pf_count == 0){
			g_pf_angle_dist = (int16_t)g_pf_angle - (int16_t)g_pf_start_angle;
		}
		else if (g_pf_count > 0){
			g_pf_angle_dist = (3600 - (int16_t)g_pf_start_angle) + (int16_t)g_pf_angle + 3600 * (g_pf_count - 1);
		}
		else{
			g_pf_angle_dist = -1 * (int16_t)g_pf_start_angle - (3600 - (int16_t)g_pf_angle) - 3600 * (g_pf_count + 1);
		}

		// ターン方向を正とする
		g_pf_angle_dist *= (int16_t)g_body_dir;
	}
	g_pf_angle_old = g_pf_angle;
}


/*
 * calc_pf_liner
 * P&Fセンサ未更新時の補間処理
 */
static void calc_pf_liner(){
	static int16_t	g_pf_angle_dist_old = 0;
	if((g_pf_angle_dist != g_pf_angle_dist_old) || ((g_pf_angle_dist == 0) && (g_pf_angle_dist_old == 0))){
		g_pf_nup_count = 0;
		g_pf_angle_dist_ln = (double)g_pf_angle_dist * 0.1 * M_PI / 180.0;
	}
	else{
		g_pf_nup_count ++;
		// 未更新中は車輪のエンコーダ値から推定した車体角速度を使って線形補間する
		g_pf_angle_dist_ln = g_pf_angle_dist_ln + g_act_body_vel * 0.01;
	}
	g_pf_angle_dist_old = g_pf_angle_dist;
}


/*
 * calc_body_angle_kalman
 * カルマンフィルタにより車体角を推定する
 */
static void calc_body_angle_kalman(){
	double dt = 0.01;				// 時間刻み(sec)
	double ver_v = 1.0 - VER_COEF;	// システムノイズの分散
	double ver_w = VER_COEF;		// 観測ノイズの分散
	double est_body_angle_pre = g_est_body_angle + dt * g_act_body_vel;
	double p_body_angle_pre = g_p_body_angle + dt * dt  * ver_v;
	double kalman_gain = p_body_angle_pre / (p_body_angle_pre + ver_w);
	g_p_body_angle = (1.0 - kalman_gain) * p_body_angle_pre;
	g_est_body_angle = est_body_angle_pre + kalman_gain * (g_pf_angle_dist_ln - est_body_angle_pre + ALPHA_KL * g_act_body_vel);
}


/*
 * judge_ttbl_finish
 * ターンテーブルが目標角(=ブレーキ開始角)に到達したか判定する
 */
static void judge_ttbl_finish(uint8_t ttbl_pos) {

	double now_angle = g_act_ttbl_angle;
	double sens_diff;

	if (((g_ttbl_sens_target_pos & ttbl_pos) != 0) && (g_flg_ttbl_sens_aprch == 0)) {
		g_flg_ttbl_sens_aprch = 1;
		g_ttbl_enc_sens = now_angle;
	}

	if (g_flg_ttbl_sens_aprch == 1) {
		sens_diff = fabs(now_angle) - fabs(g_ttbl_enc_sens);
		if (sens_diff > pTurnCalib->brake_ttbl_angle) {
			g_flg_brk_angle_ttbl = 1;
		}
	}
}

/*
 * judge_body_finish
 * 車体が目標角(=ブレーキ開始角)に到達したか判定する
 */
static void judge_body_finish(void) {

	switch(g_sync_dir){
	case R90T:
	case L90T:
		g_target_dist = 900 + ((double)g_diff_rad * 0.001 * 180.0 / PI * 10);
		break;
	case RL180T:
		g_target_dist = 1800 - abs((double)g_diff_rad * 0.001 * 180.0 / PI * 10);
		break;
	}

	if(g_pf_angle_dist > (g_target_dist - pTurnCalib->brake_body_angle)){
		g_flg_brk_angle_body = 1;
	}
}

/*
 * brake_timer
 * ブレーキ開始後の経過時間をカウント
 */
static void brake_timer(){
	if(g_flg_brk_angle_body == 1 && g_flg_brk_angle_ttbl == 1
	   && g_flg_brake_permission == 1){
		g_time_from_brk += TimeIntSync_ms;
	}
	else{
		g_time_from_brk = 0;
	}
}


/*
 * log_update
 * ログ用変数に現在の状態量を保存する
 */
static void log_update(){
	// 検査用にターン中のシンクロ誤差の最大値を保存
	int16_t g_act_err_angle_temp = (int16_t)(g_act_err_angle * 180.0 / M_PI * 10);
	if(abs(g_act_err_angle_temp) > g_result_max_err){
		g_result_max_err = g_act_err_angle_temp;	// 角度(deg)の10倍
	}

	// ターン中の状態量を保存(開発用)
#if SYNC_TURN_LOG_DUMP
	g_time_log[g_log_idx] = g_time;

	g_pf_angle_log[g_log_idx] = g_pf_angle;
	g_pf_angle_dist_ln_log[g_log_idx] = g_pf_angle_dist_ln;
	g_flg_brk_angle_body_log[g_log_idx] = g_flg_brk_angle_body;
	g_flg_brk_angle_ttbl_log[g_log_idx] = g_flg_brk_angle_ttbl;

	g_trgt_angle_log[g_log_idx] = g_trgt_angle;
	g_trgt_vel_log[g_log_idx] = g_trgt_vel;

	g_ref_body_vel_log[g_log_idx] = g_ref_body_vel;
	g_ref_ttbl_vel_log[g_log_idx] = g_ref_ttbl_vel;

	g_act_body_angle_log[g_log_idx] = g_act_body_angle;
	g_act_body_vel_lpf_log[g_log_idx] = g_act_body_vel_lpf;
	g_act_ttbl_angle_log[g_log_idx] = g_act_ttbl_angle;
	g_act_ttbl_vel_lpf_log[g_log_idx] = g_act_ttbl_vel_lpf;

	g_act_err_angle_log[g_log_idx] = g_act_err_angle;
	g_act_err_vel_lpf_log[g_log_idx] = g_act_err_vel_lpf;
	g_est_body_angle_log[g_log_idx] = g_est_body_angle;

	if (g_log_idx < (LOGSIZE - 1)){
		g_log_idx++;
	}
#endif
}

/*
 * log_reset
 * ログ変数のリセット
 */
static void log_reset(){
	g_result_max_err = 0;
#if SYNC_TURN_LOG_DUMP
	g_log_idx = 0;
	for(int i = 0; i < LOGSIZE; i++){
		g_time_log[i] = 0;
		g_pf_angle_log[i] = 0;
		g_pf_angle_dist_ln_log[i] = 0;
		g_flg_brk_angle_body_log[i] = 0;
		g_flg_brk_angle_ttbl_log[i] = 0;

		g_trgt_angle_log[i] = 0;
		g_trgt_vel_log[i] = 0;

		g_ref_body_vel_log[i] = 0;
		g_ref_ttbl_vel_log[i] = 0;

		g_act_body_angle_log[i] = 0;
		g_act_body_vel_lpf_log[i] = 0;
		g_act_ttbl_angle_log[i] = 0;
		g_act_ttbl_vel_lpf_log[i] = 0;

		g_act_err_angle_log[i] = 0;
		g_act_err_vel_lpf_log[i] = 0;

		g_est_body_angle_log[i] = 0;

	}
#endif
}

/*
 * get_diff_from_stdangele
 * 初期P＆Fセンサーの角度が、0(360), 90, 180, 270度の
 * いずれからずれている角度を求める。
 */
static int32_t get_diff_from_stdangele(uint8_t dir, uint16_t pf_angle) {

	double min_angle_d;
	double rad_d;
	int32_t rad;
	int16_t diff;
	int16_t min_angle = 900;
	int16_t angle[5] = {0, 900, 1800, 2700, 3600};
	uint8_t i;
	uint8_t buf_save;

	for (i = 0; i < 5; i ++) {
		diff = pf_angle - angle[i];
		if (abs(min_angle) > abs(diff)) {
			min_angle = diff;
			buf_save = i;
		}
	}
	min_angle_d = (double)min_angle / 10.0;

	//ラジアンに変換後、1000倍
	rad_d = (min_angle_d * M_PI) / 180.0;
	rad = rad_d * 1000;

	//回転方向によって正負を変える
	//回転方向と現在のP&Fセンサーの角度からターゲット角度を求める
	switch (dir) {
	case L90T:	//左90度回転の場合はそのまま
		if (buf_save == 0) {
			buf_save = 4;
		}
		g_target_pf_angle = angle[buf_save - 1];
		break;
	case R90T:	//右90度回転の場合は正負反転
		rad *= -1.0;
		if (buf_save == 4) {
			buf_save = 0;
		}
		g_target_pf_angle = angle[buf_save + 1];
		break;
	case RL180T:
		if (buf_save < 2) {
			g_target_pf_angle = angle[buf_save + 2];
		} else if ((buf_save > 2)) {
			g_target_pf_angle = angle[buf_save - 2];
		} else {
			// 180度付近にいる時は、どちらに回転するかによってターゲットを変える
			if (rad < 0) {
				g_target_pf_angle = angle[0];
			} else {
				g_target_pf_angle = angle[4];
			}
		}
		break;
	}

	return rad;
}

/*
 * syncturn_init
 * 内部変数のリセット
 */
static void syncturn_init() {

	reset_sync_flg_send_comp();
	g_flg_brk_angle_body = 0;
	g_flg_brk_angle_ttbl = 0;
	g_flg_ttbl_sens_aprch = 0;
	g_flg_brake_permission = 0;

	g_time = 0;
	g_diff_rad = 0;
	g_time_from_brk = 0;

	g_wheel_encoder_left_old = get_l_wheel_encoder();
	g_wheel_encoder_right_old = get_r_wheel_encoder();
	g_wheel_enccnt_left = 0;
	g_wheel_enccnt_right = 0;
	g_turntbl_enccnt = 0;
	g_ttbl_encoder_old = get_turn_encoder();

	g_pf_angle_old = 0;
	g_pf_angle_dist = 0;
	g_pf_angle_dist_ln = 0;
	g_pf_nup_count = 0;
	g_pf_count = 0;
	g_target_dist = 0;

	g_trgt_angle = 0;
	g_trgt_vel = 0;
	g_trgt_acc = 0;
	g_trgt_jerk = 0;

	g_ref_body_vel = 0;
	g_ref_body_vel_ff = 0;
	g_d_term_body = 0;
	g_d_term_body_old = 0;

	g_ref_ttbl_vel = 0;
	g_ref_ttbl_vel_ff = 0;
	g_d_term_ttbl = 0;
	g_d_term_ttbl_old = 0;

	g_act_body_angle = 0;
	g_act_body_vel   = 0;
	g_act_body_vel_lpf   = 0;
	g_act_body_vel_lpf_old = 0;

	g_act_ttbl_angle = 0;
	g_act_ttbl_vel   = 0;
	g_act_ttbl_vel_lpf   = 0;
	g_act_ttbl_vel_lpf_old = 0;

	g_act_err_angle		= 0;
	g_act_err_angle_old = 0;
	g_act_err_angle_lpf = 0;
	g_act_err_vel		= 0;
	g_act_err_vel_old	= 0;
	g_act_err_vel_lpf 	= 0;
	g_ttbl_enc_sens		= 0;

	g_p_body_angle		= 0;
	g_est_body_angle	= 0;

	g_act_body_err_angle	= 0;
	g_act_body_err_vel		= 0;
	g_act_body_err_vel_old	= 0;
	g_act_body_err_vel_lpf	= 0;
	g_act_body_err_acc		= 0;

	g_act_ttbl_err_angle	= 0;
	g_act_ttbl_err_vel		= 0;
	g_act_ttbl_err_vel_old	= 0;
	g_act_ttbl_err_vel_lpf	= 0;
	g_act_ttbl_err_acc		= 0;

	g_body_dir = 0;
	g_ttbl_dir = 0;

}
/*
 * syncturn_pram_set
 * キャリブレーションのセット
 */
static void syncturn_pram_set(uint8_t spd, uint8_t dir) {
	//キャリブレーション値をセット
	if (dir == R90T || dir == L90T ){
		switch(spd){
		case LOW:
			pTurnCalib = &turn90_low_calib;
			break;
		case MID:
			pTurnCalib = &turn90_mid_calib;
			break;
		case HIGH:
			pTurnCalib = &turn90_high_calib;
			break;
		}
		pInv_Model_Calib = &inv_model_param_90;
		pFB_Calib = &fb_pram;
	}
	else if(dir == RL180T){
		switch(spd){
		case LOW:
			pTurnCalib = &turn180_low_calib;
			break;
		case MID:
			pTurnCalib = &turn180_mid_calib;
			break;
		case HIGH:
			pTurnCalib = &turn180_high_calib;
			break;
		}
		pInv_Model_Calib = &inv_model_param_180;
		pFB_Calib = &fb_pram;
	}
}


/*
 * calc_wheel_encoder
 * 車輪エンコーダパルス更新数を取得
 */
static void calc_wheel_encoder(void) {

	uint16_t diff_l;
	uint16_t diff_r;
	uint16_t l_wheel_encoder = get_l_wheel_encoder();
	uint16_t r_wheel_encoder = get_r_wheel_encoder();
	static uint8_t cnt_nochange = 0;

	if ( l_wheel_encoder == g_wheel_encoder_left_old){
		cnt_nochange ++;
	}
	else{
		cnt_nochange = 0;
	}

	if(cnt_nochange > 5){
		// パルスが60ms更新なければ停止したとみなす
		g_flg_body_stp_jdg = 1;
	}
	else{
		g_flg_body_stp_jdg = 0;
	}

	diff_l = l_wheel_encoder - g_wheel_encoder_left_old;
	diff_r = r_wheel_encoder - g_wheel_encoder_right_old;

	g_wheel_enccnt_left += (int16_t)diff_l;
	g_wheel_enccnt_right += (int16_t)diff_r;

	g_wheel_encoder_left_old = l_wheel_encoder;
	g_wheel_encoder_right_old = r_wheel_encoder;
}


/*
 * ck_wheel_moving
 * 動輪ブレーキをかけて良い速度か確認する
 */
static void ck_wheel_moving(void){
	static uint16_t encCtrW1_diff = 0;
	static uint16_t encCtrW2_diff = 0;
	static uint16_t encCtrW1_old = 0;
	static uint16_t encCtrW2_old = 0;
	static uint8_t timer_count = 0;
	uint16_t encCtrW1 = get_r_wheel_encoder();
	uint16_t encCtrW2 = get_l_wheel_encoder();

	timer_count++;
	if (timer_count >= TIM_50MSEC) {
		timer_count = 0;

		encCtrW1_diff = abs(encCtrW1 - encCtrW1_old);
		encCtrW2_diff = abs(encCtrW2 - encCtrW2_old);

		encCtrW1_old = encCtrW1;
		encCtrW2_old = encCtrW2;
	}

	if((encCtrW1_diff < STOP_PALSE_SYNC) && (encCtrW2_diff < STOP_PALSE_SYNC)){
		g_flg_brake_permission = 1;
	}
	else{
		g_flg_brake_permission = 0;
	}
}


/*
 * calc_wheel_enc_spd
 * 車輪エンコーダのパルス更新間隔から車体角速度を算出
 */
static void calc_wheel_enc_spd(void){
	double body_angle_per_pulse = 2 * PI / (WHEEL_RATIO * PULS_WHEEL);	// エンコーダ1パルス当たりの車体回転角(rad)
	double ave_cnt = ((double)get_l_wheel_enc_cnt() + (double)get_r_wheel_enc_cnt()) * 0.5;

	double act_body_vel_old = g_act_body_vel;
	if ( ave_cnt != 0){
		g_act_body_vel = body_angle_per_pulse * 1.0e6 / (TIM_CYCLE * ave_cnt);
	}
	if(g_flg_body_stp_jdg > 0){
		g_act_body_vel = 0;
	}

	// ローパスフィルタをかける
	g_act_body_vel_lpf_old = g_act_body_vel_lpf;
	g_act_body_vel_lpf = LPF_1order(g_act_body_vel, act_body_vel_old, g_act_body_vel_lpf, BODY_VEL_FIR, TimeIntSync_sec);
}


/*
 * calc_turn_encoder
 * ターンテーブルエンコーダのパルス更新数を取得
 */
static void calc_turn_encoder(void) {

	uint16_t diff_t;
	uint16_t turn_enc = get_turn_encoder();
	static uint8_t cnt_nochange = 0;

	if ( turn_enc == g_ttbl_encoder_old){
		cnt_nochange ++;
	}
	else{
		cnt_nochange = 0;
	}

	if(cnt_nochange > 5){
		// パルスが60ms更新なければ停止したとみなす
		g_flg_ttbl_stp_jdg = 1;
	}
	else{
		g_flg_ttbl_stp_jdg = 0;
	}

	diff_t = turn_enc - g_ttbl_encoder_old;
	g_turntbl_enccnt += (int16_t)diff_t;
	g_ttbl_encoder_old = turn_enc;
}


/*
 * calc_ttbl_enc_spd
 * ターンテーブルエンコーダのパルス更新間隔からターンテーブル角速度を算出
 */
static void calc_ttbl_enc_spd(void){

	double ttbl_angle_per_pulse = 2 * PI / TURNTBL_ENC;	//ターンテーブルエンコーダ1パルス当たりの回転角
	double turn_cnt = (double)get_turn_enc_cnt();
	double act_ttbl_vel_old = g_act_ttbl_vel;
	if ( turn_cnt != 0){
		g_act_ttbl_vel = ttbl_angle_per_pulse * 1.0e6 / (TIM_CYCLE_TTBL * turn_cnt);
	}
	if(g_flg_ttbl_stp_jdg > 0){
		g_act_ttbl_vel = 0;
	}

	// ローパスフィルタをかける
	g_act_ttbl_vel_lpf_old = g_act_ttbl_vel_lpf;
	g_act_ttbl_vel_lpf = LPF_1order(g_act_ttbl_vel, act_ttbl_vel_old, g_act_ttbl_vel_lpf, TTBL_VEL_FIR, TimeIntSync_sec);
}

/*
 * set_ttbl_target_pos
 * ターゲットとするターンテーブルフォトセンサ位置を計算
 */
uint8_t set_ttbl_target_pos(uint8_t start_pos, uint8_t dir) {

	uint8_t ret_pos = 0;

	switch(start_pos) {
	case TRUN_POSI_ANG0:
		if (dir == R90T) {
			ret_pos = TRUN_POSI_ANG270;
		} else if (dir == L90T) {
			ret_pos = TRUN_POSI_ANG90;
		} else if (dir == RL180T) {
			ret_pos = TRUN_POSI_ANG180;
		} else {
			ret_pos = start_pos;
		}
		break;

	case TRUN_POSI_ANG90:
		if (dir == R90T) {
			ret_pos = TRUN_POSI_ANG0;
		} else if (dir == L90T) {
			ret_pos = TRUN_POSI_ANG180;
		} else if (dir == RL180T) {
			ret_pos = TRUN_POSI_ANG270;
		} else {
			ret_pos = start_pos;
		}
		break;

	case TRUN_POSI_ANG180:
		if (dir == R90T) {
			ret_pos = TRUN_POSI_ANG90;
		} else if (dir == L90T) {
			ret_pos = TRUN_POSI_ANG270;
		} else if (dir == RL180T) {
			ret_pos = TRUN_POSI_ANG0;
		} else {
			ret_pos = start_pos;
		}

		break;

	case TRUN_POSI_ANG270:
		if (dir == R90T) {
			ret_pos = TRUN_POSI_ANG180;
		} else if (dir == L90T) {
			ret_pos = TRUN_POSI_ANG0;
		} else if (dir == RL180T) {
			ret_pos = TRUN_POSI_ANG90;
		} else {
			ret_pos = start_pos;
		}
		break;

	default:
		//どのセンサーも反応していない場合、最初に反応したセンサーで止める
		ret_pos = TURN_SENS_ALL;
		break;
	}
	return ret_pos;
}

/*
 * Is_SyncTurnning
 * シンクロターンの状態を返す
 * 戻り値：	TRUE シンクロターン中
 * 			FALSE　シンクロターン中ではない
 */
uint8_t Is_SyncTurnning() {

	uint8_t ret = FALSE;

	if (g_syncturn_status == SYNC_TURN_ING) {
		ret = TRUE;
	}
	return ret;
}

/*
 * get_syncturn_status
 * シンクロターンのステータスを返す
 */
uint8_t get_syncturn_status() {
	return g_syncturn_status;
}

/*
 * syncturn_stop
 * シンクロターンを停止する
 */
void syncturn_stop() {

//	wheel_set_freerun(0);
//	wheel_set_brake(1);
//	wheel_set_dc_lock(0);
	//wheel_set_speed(0, 0);
	turn_motor_stop();
	t_set_brake();
	sycturn_direction(STOP, 0);
}



/*
 * set_sync_nuc_command
 * NUCからのコマンドをセット
 * シンクロターン中はNUCからのメッセージ受信の度にコールされる
 */
void set_sync_nuc_command(uint8_t cmd, uint8_t sync_spd, uint8_t sync_dir, uint16_t pf_angle) {

	uint8_t cmd_mon = 0;

	//不正コマンドをガード
	if((cmd < NO_REQ_SYNC_TURN) || (cmd > REQ_SYNC_TURN)){
		cmd = NO_REQ_SYNC_TURN;
		cmd_mon++;
	}
	if((sync_spd < LOW) || (sync_spd > HIGH)){
		sync_spd = LOW;
		cmd_mon++;
	}
	if((sync_dir < R90T) || (sync_dir > RL180T)){
		sync_dir = R90T;
		cmd_mon++;
	}
	if((pf_angle < 0) || (pf_angle > 3600)){
		pf_angle = 0;
		cmd_mon++;
	}

	if (cmd_mon != 0) {
		set_illg_command_flg();
	} else {
		reset_illg_command_flg();
	}

	//コマンドセット
	g_command = cmd;
	g_sync_spd = sync_spd;
	g_sync_dir = sync_dir;
	g_pf_angle = pf_angle;
}

/*
 * set_sync_flg_send_comp
 * シンクロターン終了ステータス送信済フラグをセット
 */
void set_sync_flg_send_comp(){
	g_flg_send_comp = 1;
}

void reset_sync_flg_send_comp(){
	g_flg_send_comp = 0;
}

uint8_t get_sync_flg_send_comp(){
	return g_flg_send_comp;
}

/*
 * set_sync_start_flag
 * シンクロターン開始フラグをセット
 */
void set_sync_start_flg(){
	g_flg_syncturn_start = 1;
}

void reset_sync_start_flg(){
	g_flg_syncturn_start = 0;
}

uint8_t get_sync_start_flg(){
	return g_flg_syncturn_start;
}

/*
 * set_illg_command_flg
 * 不正コマンド受信フラグをセット
 */
void set_illg_command_flg(){
	g_flg_illg_command = 1;
}

void reset_illg_command_flg(){
	g_flg_illg_command = 0;
}

uint8_t get_illg_command_flg(){
	return g_flg_illg_command;
}

void set_sync_dump_req(){
	g_flg_sync_dump_req = 1;
}

void reset_sync_dump_req(){
	g_flg_sync_dump_req = 0;
}

uint8_t get_sync_dump_req(){
	return g_flg_sync_dump_req;
}

void set_sync_dump_comp(){
	g_flg_sync_dump_comp = 1;
}

void reset_sync_dump_comp(){
	g_flg_sync_dump_comp = 0;
}

uint8_t get_sync_dump_comp(){
	return g_flg_sync_dump_comp;
}

/*
 * syncturn_log_dump
 * シンクロターン結果のダンプを出力する。
 */
void syncturn_log_dump() {
	extern void uart2_transmitte(char *p);

	// 検査用にターン結果を出力
	char buf1[40];
	memset(buf1, 0x00, sizeof(buf1));
	sprintf(buf1, "Table Angle: %d [deg/10]\r\n", g_result_ttbl_angle);
	uart2_transmitte(buf1);

	memset(buf1, 0x00, sizeof(buf1));
	sprintf(buf1, "Synchro Error: %d [deg/10]\r\n", g_result_max_err);
	uart2_transmitte(buf1);

	// ターン中の状態量ログを出力(開発用)
#if SYNC_TURN_LOG_DUMP
	char buf2[600];

	memset(buf2, 0x00, sizeof(buf2));
	sprintf(buf2, "g_time,g_pf_angle_dist_ln_log,"
			"g_trgt_angle,g_trgt_vel,"
			"g_ref_body_vel,g_ref_ttbl_vel,"
			"g_act_body_angle,g_act_body_vel_lpf,"
			"g_act_ttbl_angle,g_act_ttbl_vel_lpf,"
			"g_act_err_angle, g_act_err_vel_lpf,"
			"g_est_body_angle,"
			"g_flg_brk_angle_body,g_flg_brk_angle_ttbl"
			"\r\n");
	uart2_transmitte(buf2);
	for(int i = 0; i < LOGSIZE; i++) {
		memset(buf2, 0x00, sizeof(buf2));
		//             1   2   3   4   5   6   7   8   9   10  11  12  13  14  15
		sprintf(buf2, "%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %d\r\n",
				g_time_log[i],					// 1
				g_pf_angle_dist_ln_log[i],			// 2
				g_trgt_angle_log[i],			// 3
				g_trgt_vel_log[i],				// 4
				g_ref_body_vel_log[i],			// 5
				g_ref_ttbl_vel_log[i],			// 6
				g_act_body_angle_log[i],		// 7
				g_act_body_vel_lpf_log[i],		// 8
				g_act_ttbl_angle_log[i],		// 9
				g_act_ttbl_vel_lpf_log[i],		// 10
				g_act_err_angle_log[i],			// 11
				g_act_err_vel_lpf_log[i],		// 12
				g_est_body_angle_log[i],		// 13
				g_flg_brk_angle_body_log[i],	// 14
				g_flg_brk_angle_ttbl_log[i]		// 15
				);
		uart2_transmitte(buf2);
		if (i != 0 && g_time_log[i+1] == 0){
			break;
		}
	}
#endif
	set_sync_dump_comp();
}


/*
 * sync_status_ctrl
 * シンクロターンステータス制御
 */
static uint8_t sync_status_ctrl(uint8_t syncturn_status_z){

	uint8_t ret = syncturn_status_z;

	switch(syncturn_status_z){

	case SYNC_TURN_READY:
		if(g_command == REQ_SYNC_TURN){
			if ((ck_emerg_stop() == NOT_EMERGENCY) && (get_lift_pos() != LIFT_DOWN_STATE) && (get_illg_command_flg() == 0)){
				ret = SYNC_TURN_ING;
				set_sync_start_flg();
			}
			else{
				ret = SYNC_TURN_ILLG;
			}
		}
		break;

	case SYNC_TURN_ING:
		if((ck_emerg_stop() != NOT_EMERGENCY) || (get_illg_command_flg() != 0) || (g_command == NO_REQ_SYNC_TURN)){
			ret = SYNC_TURN_ILLG;
		}
		else if(g_time_from_brk >= WAIT_TIME) {
			if (syncturn_result() == 1){
			ret = SYNC_TURN_COMP;
			}
			else if (syncturn_result() != 1){
				ret = SYNC_TURN_ERR_COMP;
			}
		}
		break;

	case SYNC_TURN_COMP:
		if((g_command == NO_REQ_SYNC_TURN) && (get_sync_flg_send_comp() == 1)){
			ret = SYNC_TURN_READY;
		}
		break;

	case SYNC_TURN_ERR_COMP:
		if((g_command == NO_REQ_SYNC_TURN) && (get_sync_flg_send_comp() == 1)){
			ret = SYNC_TURN_READY;
		}
		break;

	case SYNC_TURN_ILLG:
		if((g_command == NO_REQ_SYNC_TURN) && (get_sync_flg_send_comp() == 1)){
			ret = SYNC_TURN_READY;
		}
		break;

	default: //ここは通らないがワーニング回避のために記述
		ret = SYNC_TURN_READY;
		break;
	}
	return ret;
}


/*
 * syncturn_ctrl
 * シンクロターンのメイン関数
 */
void syncturn_ctrl() {

	uint8_t ttbl_sens_pos = get_turn_sensor_pos();

	// シンクロターンステータス更新
	g_syncturn_status = sync_status_ctrl(g_syncturn_status);

	switch(g_syncturn_status){
	case SYNC_TURN_READY:
		syncturn_init();
		break;

	case SYNC_TURN_ING:
		if (get_sync_start_flg() == 1){	//　INGに移行した1job目のみ実行
			syncturn_pram_set(g_sync_spd, g_sync_dir);
			g_pf_start_angle = (short)g_pf_angle;
			g_diff_rad = get_diff_from_stdangele(g_sync_dir, g_pf_angle);
			sycturn_direction(g_sync_dir, g_diff_rad);
			g_ttbl_sens_target_pos = set_ttbl_target_pos(ttbl_sens_pos, g_sync_dir);
			log_reset();
			// ブレーキ解除
//			wheel_set_brake(0);
//			wheel_set_dc_lock(0);
			t_release_brake();
			// モータードライバの負荷慣性設定
//			wheel_set_inertia(INERTIA1);
		}
		calc_wheel_encoder();
		ck_wheel_moving();
		calc_wheel_enc_spd();
		calc_turn_encoder();
		calc_ttbl_enc_spd();
		calc_angle();
		calc_pf_dist();
		calc_pf_liner();
		calc_body_angle_kalman();
		calc_turn_locus();
		calc_err_angle();
		judge_ttbl_finish(ttbl_sens_pos);
		judge_body_finish();
		brake_timer();
		sync_motor_ctrl();
		log_update();
		reset_sync_start_flg();
		g_time += TimeIntSync_ms;
		break;

	case SYNC_TURN_COMP:
		syncturn_stop();
		break;

	case SYNC_TURN_ERR_COMP:
		syncturn_stop();
		break;

	case SYNC_TURN_ILLG:
		syncturn_stop();
		break;
	}

}
