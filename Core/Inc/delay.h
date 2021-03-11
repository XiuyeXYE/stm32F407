/*
 * delay.h
 *
 *  Created on: 2021年3月11日
 *      Author: Dell
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include"sys.h"

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);



#endif /* INC_DELAY_H_ */
