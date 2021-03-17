/*
 * delay.h
 *
 *  Created on: Mar 17, 2021
 *      Author: Dell
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_
#include "main.h"

void delay_init(uint8_t sysclkMHZ);

void delay_ms(uint16_t n);

void delay_us(uint32_t n);


#endif /* INC_DELAY_H_ */
