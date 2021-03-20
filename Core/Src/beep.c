/*
 * beep.c
 *
 *  Created on: Mar 17, 2021
 *      Author: Dell
 */

#include "beep.h"

void beep_init() {

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOF_CLK_ENABLE();

	/*Configure GPIO pin Output Level */


	/*Configure GPIO pins : PF8  */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);

}

