/*
 * conf.h
 *
 *  Created on: 2019/09/12
 *      Author: yuki
 */

#ifndef CONF_H_
#define CONF_H_

// ----firmware version ----------------------
#define ver_length 8
// farmware_version	"SPC.004.000.000.000"
// 仕向け先、メジャーバージョン、マイナーバージョン、バッチバージョンの順に定義
// バージョン情報 000 ~ 999 を2byteの整数値として送信する
#define	DEST_HIGH	0x00
#define	DEST_LOW	0x04
#define MAJOR_HIGH	0x00
#define MAJOR_LOW	0x00
#define MINOR_HIGH	0x00
#define MINOR_LOW	0x00
#define BATCH_HIGH	0x00
#define BATCH_LOW	0x00


// 各種設定切り替え用スイッチ

// 近接センサー設定用スイッチ
// 0 : 近接センサー不使用
// 1 : 前方近接センサーのみ使用
// 2 : 後方近接センサーのみ使用
// 3 : 両方の近接センサーを使用
#define LIDAR_USE 1

// バッテリー選択スイッチ
// 0 : NECバッテリー
// 1 : 村田バッテリー
#define BATTERY_TYPE	0

// ブレーキ解除時、黄色点灯させるかどうかを選択するスイッチ
// 0 : 無効　　1 : 有効
#define USE_BRAKE_RELEASE	1

// 異常発生時、各モーターの電源を落とすかどうかを選択するスイッチ
// 0 : 無効　　1 : 有効
#define EMERGE_POWER_CUT	0

// 報知機の使用選択スイッチ
// 0 : 無効　　1 : 有効
#define SOUND_USE	1

// 光伝送装置の使用選択スイッチ
// 0 : 無効　　 1 : 有効
#define OPTCOM_USE 0

// 負荷率測定の選択スイッチ
// 0 : 無効　　 1 : 有効
#define USE_LOAD_MEASURE	0

// シンクロターンログ機能の選択スイッチ
// 0 : 無効　　 1 : 有効
#define SYNC_TURN_LOG_DUMP	0

// ターンテーブルログ機能の選択スイッチ
// 0 : 無効　　 1 : 有効
#define TURN_LOG_DUMP	0

// 走行用モーターログ機能の選択スイッチ
// 0 : 無効　　 1 : 有効
#define WHEEL_LOG_DUMP	1

// 走行用モーターテスト機能の選択スイッチ
// 0 : 無効　　 1 : 有効
#define WHEEL_TEST		0

// シンクロターン検査結果出力の可否スイッチ
// 0 : 無効　　 1 : 有効
#define SYNC_RESULT_OUT	0

// 円周率
#define PI	3.1415926f

// 走行用モーターのギヤ比選択
#define WHEEL_GEAR_RATIO	30

// 動輪の直径[m]
#define WHEEL_DIAMETER	0.155

// 車輪一回転当たりのエンコーダパルス数
#define PULS_WHEEL		900

// NUCからの速度指令最大値
#define MAX_NUC_REF		26500

// 速度指令最大時の車速[m/s]
#define MAX_NUC_SPEED_WHEEL 0.8

//ターンテーブル1回転した時のパルス数
#define	TURNTBL_ENC		12666.7

#endif /* CONF_H_ */
