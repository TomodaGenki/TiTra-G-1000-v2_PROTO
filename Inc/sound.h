/*
 * sound.h
 *
 *  Created on: 2020/05/14
 *      Author: katte
 */

#ifndef SOUND_H_
#define SOUND_H_

/* function ----------------------------------------------------------*/
void sound_cntrol(void);
/* define ------------------------------------------------------------*/
#define NO_SOUND	0x00
#define SOUND_CH1	0x10
#define SOUND_CH2	0x20
#define SOUND_CH3	0x40
#define SOUND_CH4	0x80
#define	SOUND_CH_ALL	(SOUND_CH1 + SOUND_CH2 + SOUND_CH3 + SOUND_CH4)

#endif /* SOUND_H_ */
