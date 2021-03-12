/*
 * beep.c
 *
 *  Created on: 2021年3月12日
 *      Author: Dell
 */

#include"beep.h"

//初始化PF8为输出口
//BEEP IO初始化
void BEEP_Init(void) {

	GPIO_InitTypeDef2 GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); //使能GPIOF时钟


	//初始化蜂鸣器对应引脚GPIOF8
	  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_8;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉

	GPIO_Init2(GPIOF2, &GPIO_InitStructure); //初始化GPIO



	GPIO_ResetBits2(GPIOF2, GPIO_PIN_8);  //蜂鸣器对应引脚GPIOF8拉低，
}

