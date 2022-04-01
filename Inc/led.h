/*
 * led.h
 *
 *  Created on: 2019/11/27
 *      Author: Takumi
 */
#ifndef LED_H_
#define LED_H_
// ----------- extern ------------

//--- timer ---

// --- define ---
#define	LED_TURN_OFF		0x00
// --- BLINK ----
#define LED_BLINK_SLOW		0x01
#define LED_BLINK_MIDIUM	0x02
#define LED_BLINK_QUICK		0x03
#define	LED_BLINK_REQ		(LED_BLINK_SLOW + LED_BLINK_MIDIUM)

// --- BLINK PERIOD ---
#define LED_BLINK_PERIOD_SLOW	200 // 2.0sec
#define LED_BLINK_PERIOD_MIDIUM	100	// 1.0sec
#define LED_BLINK_PERIOD_QUICK	50	// 0.5sec


// --- BRIGHTNESS ---
#define	LED_WINKER_RIGHT	0x04
#define	LED_WINKER_LEFT		0x08
#define	LED_WINKER_REQ		(LED_WINKER_RIGHT + LED_WINKER_LEFT)

// --- COLOR ------
#define LED_COLOR_BLUE		0x10
#define LED_COLOR_GREEN		0x20
#define LED_COLOR_RED		0x40
#define LED_COLOR_PUPLE		(LED_COLOR_RED+LED_COLOR_BLUE)
#define	LED_COLOR_YELLOW	(LED_COLOR_RED+LED_COLOR_GREEN)
#define	LED_COLOR_LIGHTBLUE	(LED_COLOR_BLUE+LED_COLOR_GREEN)
#define	LED_COLOR_WHITE		(LED_COLOR_RED+LED_COLOR_BLUE+LED_COLOR_GREEN)
#define LED_TABLE_FLASH		0x80

enum Led_Cntrl {
	LedOff,
	LedOn
};

// ---
void led_proc(void);
// --- debug ---
#endif
