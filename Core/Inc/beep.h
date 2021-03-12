/*
 * beep.h
 *
 *  Created on: 2021年3月12日
 *      Author: Dell
 */

#ifndef INC_BEEP_H_
#define INC_BEEP_H_

#include"sys.h"

#define BEEP PFout(8)	// 蜂鸣器控制IO

void BEEP_Init(void); //初始化

#endif /* INC_BEEP_H_ */
