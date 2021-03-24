/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

void USART1_IRQHandler() {
	if (USART1->SR & (1 << 5)) //接收到数据
			{
		uint32_t res = USART1->DR;
		if ((USART_RX_STA & 0x8000) == 0) //接收未完成
				{
			if (USART_RX_STA & 0x4000) //接收到了0x0d
					{
				if (res != 0x0a)
					USART_RX_STA = 0; //接收错误,重新开始
				else
					USART_RX_STA |= 0x8000;	//接收完成了
			} else //还没收到0X0D
			{
				if (res == 0x0d)
					USART_RX_STA |= 0x4000;
				else {
					USART_RX_BUF[USART_RX_STA & 0X3FFF] = res;
					USART_RX_STA++;
					if (USART_RX_STA > (USART_REC_LEN - 1))
						USART_RX_STA = 0; //接收数据错误,重新开始接收
				}
			}
		}
	}
}

/* USER CODE END 1 */

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void) {

//LED 和 蜂鸣器 初始化
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_USART1_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : PF8 PF9 PF10 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	// USART1串口初始化

	//GPIOA EN
	RCC->AHB1ENR |= 1;
	//USART1 EN
	RCC->APB2ENR |= 1 << 4;

//	GPIO_InitTypeDef GPIO_USART1_InitStruct = { 0 };

	//引脚9和10 模式为复用模式 10B
	GPIOA->MODER |= 2 << 18 | 2 << 20;
	GPIOA->OSPEEDR |= 2 << 18 | 2 << 20;
	GPIOA->OTYPER |= 1 << 9 | 1 << 10;
	GPIOA->PUPDR |= 1 << 18 | 1 << 20;

	//内存是小端模式？
	//pin9 和pin10 在高位上！
	//复用功能有什么用？
	//复用功能AF7 是USART1 串口！
	GPIOA->AFR[1] |= 7 << 4 | 7 << 8;

	uint32_t bound = 115200;
	uint32_t pclk2 = 84;

	float temp;
	uint16_t mantissa;
	uint16_t fraction;
	temp = (float) (pclk2 * 1000000) / (bound * 16);	//得到USARTDIV@OVER8=0
	mantissa = temp;				 //得到整数部分
	fraction = (temp - mantissa) * 16; //得到小数部分@OVER8=0
	mantissa <<= 4;
	mantissa += fraction;

	//波特率设置
	USART1->BRR = mantissa; 	//波特率设置
	USART1->CR1 &= ~(1 << 15); 	//设置OVER8=0
	USART1->CR1 |= 1 << 3;  	//串口发送使能
#if EN_USART1_RX		  	//如果使能了接收
		//使能接收中断
	USART1->CR1|=1<<2;  	//串口接收使能
	USART1->CR1|=1<<5;    	//接收缓冲区非空中断使能
	MY_NVIC_Init(3,3,USART1_IRQn,2);//组2，最低优先级
#endif
	USART1->CR1 |= 1 << 13;  	//串口使能

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
