/*
 * led.h
 *
 *  Created on: 2021年3月11日
 *      Author: Dell
 */

#ifndef INC_LED_H_
#define INC_LED_H_

//LED端口定义
#define LED0 PFout(9)	// DS0
#define LED1 PFout(10)	// DS1
void LED_Init(void);//初始化


#endif /* INC_LED_H_ */
