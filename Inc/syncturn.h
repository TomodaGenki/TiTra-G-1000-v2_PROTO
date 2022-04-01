/*
 * syncturn.h
 *
 *  Created on: 2020/11/27
 *      Author: ROBO TAKUMI
 */

#ifndef SYNCTURN_H_
#define SYNCTURN_H_

#define Bool int

//シンクロターンステータス
#define	SYNC_TURN_READY	0x00		//シンクロターン待機中
#define	SYNC_TURN_ING	0x01		//シンクロターン中
#define SYNC_TURN_COMP	0x02		//シンクロターン完了
#define SYNC_TURN_ERR_COMP	0x03	//シンクロターン規定外完了
#define SYNC_TURN_ILLG	0x07		//シンクロターン異常終了
//NUC要求　シンクロターンコマンド
#define NO_REQ_SYNC_TURN 0x00	//シンクロターン非要求
#define REQ_SYNC_TURN	 0x11 	//シンクロターン実施要求

//NUC要求　回転方向種別
typedef enum  {
		R90T = 0x01,
		L90T,
		RL180T,
		STOP
} KIND_OF_TURN_DIR;

//NUC要求　回転速度種別
typedef enum  {
		LOW = 0x01,
		MID,
		HIGH
} KIND_OF_TURN_SPD;

//デバイス種別
typedef enum motor_kind {
	WHEEL_RIGHT,
	WHEEL_LEFT,
	TURN_TABLE,
	SYNC_WHEEL
} MOTOR_KIND;

//キャリブレーション定数
typedef struct sync_ctrl_tbl {
	double		reach_angle;		//1.ターン量(rad)
	double		turn_time;			//2.ターン時間(sec)
	int16_t		brake_body_angle;	//3.ブレーキ開始車体角 (angle/10)
	double		brake_ttbl_angle;	//4.テーブルセンサー反応後、テーブルを進める距離 rad
} SYNC_CTRL_TBL;

//フィードバックパラメータ
typedef struct sync_fb_pram{
	double		Kp_body;				//1.
	double		Ti_body;				//2.
	double		Td_body;				//3.
	double		eta_body;				//4.
	double		Kp_ttbl;				//5.
	double		Ti_ttbl;				//6.
	double		Td_ttbl;				//7.
	double		eta_ttbl;				//8.
}SYNC_FB_PRAM;

//逆モデルパラメータ
typedef struct sync_inv_mdl{
	double		inv_mdl_body_gain_rise;		//1.
	double		inv_mdl_body_omega_rise;	//2.
	double		inv_mdl_body_zeta_rise;		//3.
	double		inv_mdl_body_gain_fall;		//4.
	double		inv_mdl_body_omega_fall;	//5.
	double		inv_mdl_body_zeta_fall;		//6.
	double		inv_mdl_ttbl_gain_rise;		//7.
	double		inv_mdl_ttbl_omega_rise;	//8.
	double		inv_mdl_ttbl_zeta_rise;		//9.
	double		inv_mdl_ttbl_gain_fall;		//10.
	double		inv_mdl_ttbl_omega_fall;	//11.
	double		inv_mdl_ttbl_zeta_fall;		//12.
}SYNC_INV_MDL;


//モデルパラメータ
typedef struct sync_ttbl_mdl{
	double		mdl_ttbl_gain_90;	//1.90度ターン時のgain
	double		mdl_ttbl_omega_90;	//2.90度ターン時のomega
	double		mdl_ttbl_zeta_90;	//3.90度ターン時のzeta
	double		mdl_ttbl_gain_180;	//4.180度ターン時のgain
	double		mdl_ttbl_omega_180;	//5.180度ターン時のomega
	double		mdl_ttbl_zeta_180;	//6.180度ターン時のzeta
}SYNC_TTBL_MDL;


void syncturn_ctrl(void);
uint8_t Is_SyncTurnning(void);
void set_sync_nuc_command(uint8_t, uint8_t, uint8_t, uint16_t);
uint8_t get_syncturn_status(void);
void set_sync_flg_send_comp(void);
void reset_sync_flg_send_comp(void);
uint8_t get_sync_flg_send_comp(void);
void set_sync_start_flg(void);
void reset_sync_start_flg(void);
uint8_t get_sync_start_flg(void);
void set_illg_command_flg(void);
void reset_illg_command_flg(void);
uint8_t get_illg_command_flg(void);
void syncturn_log_dump(void);
void set_sync_dump_req(void);
void reset_sync_dump_req(void);
uint8_t get_sync_dump_req(void);
void set_sync_dump_comp(void);
void reset_sync_dump_comp(void);
uint8_t get_sync_dump_comp(void);

#endif /* SYNCTURN_H_ */
