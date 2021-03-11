/*
 * led.cpp
 *
 *  Created on: 2021年3月11日
 *      Author: Dell
 */

#include"sys.h"
#include "led.h"



//初始化PF9和PF10为输出口.并使能这两个口的时钟
//LED IO初始化
void LED_Init(void) {
	RCC->AHB1ENR |= 1 << 5; //使能PORTF时钟
//	GPIO_Set(GPIOF,PIN9|PIN10,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF9,PF10设置
//	LED0 = 1; //LED0关闭
//	LED1 = 1; //LED1关闭

//	GPIOF->MODER &= ~(3 << 2 * 9);
//	GPIOF->MODER |= (1 << (2 * 9));

//	GPIOF->OSPEEDR &= ~
	//PF9
	//优先级 &(取址) 与 -> 一样的，但是结合性是自右向左
	Reg_Pos_Bits_Reset_0(&GPIOF->MODER, 2 * 9, 2);
	Reg_Pos_Bits_Set(&GPIOF->MODER, 2 * 9, 1);

	Reg_Pos_Bits_Reset_0(&GPIOF->OSPEEDR, 2 * 9, 2);
	Reg_Pos_Bits_Set(&GPIOF->OSPEEDR, 2 * 9, 2);

	Reg_Pos_Bits_Reset_0(&GPIOF->PUPDR, 2 * 9, 2);
	Reg_Pos_Bits_Set(&GPIOF->PUPDR, 2 * 9, 1);

	Reg_Pos_Bits_Reset_0(&GPIOF->OTYPER, 9, 1);
	Reg_Pos_Bits_Set(&GPIOF->OTYPER, 9, 0);

	Reg_Pos_Bits_Set(&GPIOF->ODR, 9, 1);

//	Reg_Pos_Bits_Set(&GPIOF->ODRA,9,1);

	//PF10

	Reg_Pos_Bits_Reset_0(&GPIOF->MODER, 2 * 10, 2);
	Reg_Pos_Bits_Set(&GPIOF->MODER, 2 * 10, 1);

	Reg_Pos_Bits_Reset_0(&GPIOF->OSPEEDR, 2 * 10, 2);
	Reg_Pos_Bits_Set(&GPIOF->OSPEEDR, 2 * 10, 2);

	Reg_Pos_Bits_Reset_0(&GPIOF->PUPDR, 2 * 10, 2);
	Reg_Pos_Bits_Set(&GPIOF->PUPDR, 2 * 10, 1);

	Reg_Pos_Bits_Reset_0(&GPIOF->OTYPER, 10, 1);
	Reg_Pos_Bits_Set(&GPIOF->OTYPER, 10, 0);

	Reg_Pos_Bits_Set(&GPIOF->ODR, 10, 1);

	//	Reg_Pos_Bits_Set(&GPIOF->ODRA,10,1);

}

