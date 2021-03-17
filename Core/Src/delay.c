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
	SysTick->CTRL &= ~(1 << 2);
	fac_us = sysclkMHZ / 8;
	fac_ms = (uint16_t) fac_us * 1000;
}



void delay_us(uint32_t n) {
	uint32_t temp;
	SysTick->LOAD = n * fac_us; 				//时间加载
	SysTick->VAL = 0x00;        				//清空计数器
	SysTick->CTRL = 0x01;      				//开始倒数
	do {
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp & (1 << 16)));	//等待时间到达
	SysTick->CTRL = 0x00;      	 			//关闭计数器
	SysTick->VAL = 0X00;
}



void delay_xms(uint16_t n)
{
	uint32_t temp;
	SysTick->LOAD=(uint32_t)n*fac_ms;			//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           			//清空计数器
	SysTick->CTRL=0x01 ;          			//开始倒数
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达
	SysTick->CTRL=0x00;       				//关闭计数器
	SysTick->VAL =0X00;     		  		//清空计数器
}

void delay_ms(uint16_t n)
{
	uint8_t repeat=n/540;						//这里用540,是考虑到某些客户可能超频使用,
											//比如超频到248M的时候,delay_xms最大只能延时541ms左右了
	uint16_t remain=n%540;
	while(repeat)
	{
		delay_xms(540);
		repeat--;
	}
	if(remain)delay_xms(remain);
}

