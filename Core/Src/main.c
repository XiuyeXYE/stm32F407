/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "delay.h"
#include<stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef unsigned char u8;
typedef unsigned int u32;

void ext_NVIC_CONFIG(u8 gpiox, u8 bitx, u8 trim) {
	u8 extoffset = (bitx % 4) * 4;
	RCC->APB2ENR |= 1 << 14;
	SYSCFG->EXTICR[bitx / 4] &= ~(0x000F << extoffset);
	SYSCFG->EXTICR[bitx / 4] |= gpiox << extoffset;
	EXTI->EMR |= 1 << bitx;
	if (trim & 0x01) {
		EXTI->FTSR |= 1 << bitx;
	}
	if (trim & 0x02) {
		EXTI->RTSR |= 1 << bitx;
	}

}

#define GPIO_A 				0
#define GPIO_B 				1
#define GPIO_C				2
#define GPIO_D 				3
#define GPIO_E 				4
#define GPIO_F 				5
#define GPIO_G 				6
#define GPIO_H 				7
#define GPIO_I 				8

#define FTIR   				1  		//下降沿触发
#define RTIR   				2  		//上升沿触发

//设置NVIC分组
//NVIC_Group:NVIC分组 0~4 总共5组
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group) {
	u32 temp, temp1;
	temp1 = (~NVIC_Group) & 0x07; //取后三位
	temp1 <<= 8;
	temp = SCB->AIRCR;  //读取先前的设置
	temp &= 0X0000F8FF; //清空先前分组
	temp |= 0X05FA0000; //写入钥匙
	temp |= temp1;
	SCB->AIRCR = temp;  //设置分组
}

//设置NVIC
//NVIC_PreemptionPriority:抢占优先级
//NVIC_SubPriority       :响应优先级
//NVIC_Channel           :中断编号
//NVIC_Group             :中断分组 0~4
//注意优先级不能超过设定的组的范围!否则会有意想不到的错误
//组划分:
//组0:0位抢占优先级,4位响应优先级
//组1:1位抢占优先级,3位响应优先级
//组2:2位抢占优先级,2位响应优先级
//组3:3位抢占优先级,1位响应优先级
//组4:4位抢占优先级,0位响应优先级
//NVIC_SubPriority和NVIC_PreemptionPriority的原则是,数值越小,越优先
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority,
		u8 NVIC_Channel, u8 NVIC_Group) {
	u32 temp;
	MY_NVIC_PriorityGroupConfig(NVIC_Group);  //设置分组
	temp = NVIC_PreemptionPriority << (4 - NVIC_Group);
	temp |= NVIC_SubPriority & (0x0f >> NVIC_Group);
	temp &= 0xf;								//取低四位
	NVIC->ISER[NVIC_Channel / 32] |= 1 << NVIC_Channel % 32;//使能中断位(要清除的话,设置ICER对应位为1即可)
	NVIC->IP[NVIC_Channel] |= temp << 4;				//设置响应优先级和抢断优先级
}

//外部中断0服务程序
void EXTI0_IRQHandler(void) {
	delay_ms(10);	//消抖
	printf("EXTI0_IRQHandler");
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
					GPIO_PIN_RESET);
	EXTI->PR = 1 << 0;  //清除LINE0上的中断标志位
}
//外部中断2服务程序
void EXTI2_IRQHandler(void) {
	delay_ms(10);	//消抖
	printf("EXTI2_IRQHandler");
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
					GPIO_PIN_RESET);
	EXTI->PR = 1 << 2;  //清除LINE2上的中断标志位
}
//外部中断3服务程序
void EXTI3_IRQHandler(void) {
	delay_ms(10);	//消抖
	printf("EXTI3_IRQHandler");
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
					GPIO_PIN_RESET);
	EXTI->PR = 1 << 3;  //清除LINE3上的中断标志位
}
//外部中断4服务程序
void EXTI4_IRQHandler(void) {
	delay_ms(10);	//消抖
	printf("EXTI4_IRQHandler");
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
					GPIO_PIN_RESET);
	EXTI->PR = 1 << 4;  //清除LINE4上的中断标志位
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	/* USER CODE BEGIN 2 */


	RCC->AHB1ENR |= 1 << 0;
	RCC->AHB1ENR |= 1 << 4;

	//init key
	GPIO_InitTypeDef GPIOE_InitStruct = { 0 };
	GPIOE_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
	GPIOE_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIOE_InitStruct.Pull = GPIO_PULLUP;
	GPIOE_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOE, &GPIOE_InitStruct);

	GPIO_InitTypeDef GPIOA_InitStruct = { 0 };
	GPIOA_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
	GPIOA_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIOA_InitStruct.Pull = GPIO_PULLDOWN;
	GPIOA_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOE, &GPIOA_InitStruct);

	ext_NVIC_CONFIG(GPIO_E, 2, FTIR);
	ext_NVIC_CONFIG(GPIO_E, 3, FTIR);
	ext_NVIC_CONFIG(GPIO_E, 4, FTIR);
	ext_NVIC_CONFIG(GPIO_A, 0, RTIR);

	MY_NVIC_Init(3, 2, EXTI2_IRQn, 2);		//抢占3，子优先级2，组2
	MY_NVIC_Init(2, 2, EXTI3_IRQn, 2);		//抢占2，子优先级2，组2
	MY_NVIC_Init(1, 2, EXTI4_IRQn, 2);		//抢占1，子优先级2，组2
	MY_NVIC_Init(0, 2, EXTI0_IRQn, 2);		//抢占0，子优先级2，组2


//	beep_init();
	delay_init(168);

	/* USER CODE END 2 */
	uint32_t len = 0;
	uint32_t times = 0;
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (USART_RX_STA & 0x8000) {
			len = USART_RX_STA & 0x3fff; //得到此次接收到的数据长度
			printf("\r\n您发送的消息为:\r\n");
			for (uint32_t t = 0; t < len; t++) {
				USART1->DR = USART_RX_BUF[t];
				while ((USART1->SR & 0X40) == 0)
					; //等待发送结束
			}
			printf("\r\n\r\n"); //插入换行
			USART_RX_STA = 0;
		} else {
			times++;
			if (times % 5000 == 0) {
				printf("\r\nALIENTEK 探索者STM32F407开发板 串口实验\r\n");
				printf("正点原子@ALIENTEK\r\n\r\n\r\n");
			}
			if (times % 200 == 0)
				printf("请输入数据,以回车键结束\r\n");
//			if (times % 30 == 0){
//				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9 | GPIO_PIN_10,
//												GPIO_PIN_SET);
//			}else{
//				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9 | GPIO_PIN_10,
//												GPIO_PIN_RESET);
//			}

//			delay_ms(10);
		}

		/* USER CODE END WHILE */
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
//				GPIO_PIN_RESET);
//		delay_ms(500);
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
//				GPIO_PIN_SET);
//		delay_ms(500);
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
