#include "pwm.h"

/***需在主函数将TIM4CH4_CAPTURE_STA置零，才可以开启 第二次捕***/
//TIM1 PWM部分初始化 PE9--CH1,PE11--CH2,PE13--CH3,PE14--CH4
//arr：自动重装值
//psc：时钟预分频数
void Tim1_PWM_Init(u16 arr,u16 psc)
{
	RCC->APB2ENR|=1<<0;		//TIM1时钟使能  
	RCC->AHB1ENR|=1<<4;		//使能PORTE时钟	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;		//复用功能
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;		//上拉输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; 
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	TIM1->ARR=arr;
	TIM1->PSC=psc;
	TIM1->CCMR1|=6<<4;		//CH1 PWM1模式
	TIM1->CCMR1|=6<<12;		//CH2 PWM1模式
	TIM1->CCMR2|=6<<4;		//CH3 PWM1模式
	TIM1->CCMR2|=6<<12;		//CH4 PWM1模式
	
	TIM1->CCMR1|=1<<3;		//CH1 预装载使能 
	TIM1->CCMR1|=1<<11;		//CH2 预装载使能
	TIM1->CCMR2|=1<<3;		//CH3 预装载使能
	TIM1->CCMR2|=1<<11;		//CH4 预装载使能
	
	TIM1->CCER|=1<<0;		//OC1 输出使能
	TIM1->CCER|=1<<4;		//OC2 输出使能
	TIM1->CCER|=1<<8;		//OC3 输出使能
	TIM1->CCER|=1<<12;		//OC4 输出使能
	
	TIM1->CCER|=1<<1;		//OC1 低电平有效 
	TIM1->CCER|=1<<5;		//OC2 低电平有效 
	TIM1->CCER|=1<<9;		//OC3 低电平有效 
	TIM1->CCER|=1<<13;		//OC4 低电平有效 
	
	TIM1->CR1|=1<<7;    	//ARPE使能 //自动重载预装载使能
	TIM1->BDTR|=1<<15;  	//MOE位，主输出使能位
	
	TIM1->CR1|=1<<0;    	//使能定时器1
	
}

