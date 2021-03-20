/*
 * delay.c
 *
 *  Created on: Mar 17, 2021
 *      Author: Dell
 */

#include "delay.h"

static uint8_t fac_us = 0;
static uint8_t fac_ms = 0;

void delay_init(uint8_t sysclkMHZ) {
//	SysTick->CTRL &= ~(1 << 2);
//	fac_us = sysclkMHZ / 16;
//	fac_ms = (uint16_t) fac_us * 1000;

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); //SysTick频率为HCLK
	fac_us = sysclkMHZ;						//不论是否使用OS,fac_us都需要使用
}

void delay_us(uint32_t n) {
//	uint32_t temp;
//	SysTick->LOAD = n * fac_us; 				//时间加载
//	SysTick->VAL = 0x00;        				//清空计数器
//	SysTick->CTRL = 0x01;      				//开始倒数
//	do {
//		temp = SysTick->CTRL;
//	} while ((temp & 0x01) && !(temp & (1 << 16)));	//等待时间到达
//	SysTick->CTRL = 0x00;      	 			//关闭计数器
//	SysTick->VAL = 0X00;

	uint32_t ticks;
	uint32_t told, tnow, tcnt = 0;
	uint32_t reload = SysTick->LOAD;				//LOAD的值
	ticks = n * fac_us; 						//需要的节拍数
	told = SysTick->VAL;        				//刚进入时的计数器值
	while (1) {
		tnow = SysTick->VAL;
		if (tnow != told) {
			if (tnow < told)
				tcnt += told - tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else
				tcnt += reload - tnow + told;
			told = tnow;
			if (tcnt >= ticks)
				break;			//时间超过/等于要延迟的时间,则退出.
		}
	};

}

void delay_xms(uint16_t n) {
	uint32_t temp;
	SysTick->LOAD = (uint32_t) n * fac_ms;			//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL = 0x00;           			//清空计数器
	SysTick->CTRL = 0x01;          			//开始倒数
	do {
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp & (1 << 16)));	//等待时间到达
	SysTick->CTRL = 0x00;       				//关闭计数器
	SysTick->VAL = 0X00;     		  		//清空计数器
}

void delay_ms(uint16_t n) {
//	uint8_t repeat = n / 540;						//这里用540,是考虑到某些客户可能超频使用,
//	//比如超频到248M的时候,delay_xms最大只能延时541ms左右了
//	uint16_t remain = n % 540;
//	while (repeat) {
//		delay_xms(540);
//		repeat--;
//	}
//	if (remain)
//		delay_xms(remain);

	for(uint32_t i=0;i<n;i++) delay_us(1000);
}

