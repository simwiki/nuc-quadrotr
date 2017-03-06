#ifndef __PWM_H
#define __PWM_H
#include"sys.h"

//arr:自动重装载值（TIM2,TIM5是32位的 ！！！ ）
//psc：时钟预分频数

void Tim1_PWM_Init(u16 arr,u16 psc);

#endif