#include "csb.h"
#include "usart.h"
#include "delay.h"
u8  TIM4CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM4CH1_CAPTURE_VAL;	//输入捕获值

//RX-PD14  捕获Echo  		TIM4_CH3
//TX-PD15  控制端trigger 	TIM4_CH4

void Tim4_CH3_Capture_Init(u16 arr,u16 psc)
{
	RCC->AHB1ENR|=1<<3;					//使能PORTD
	RCC->APB1ENR|=1<<2; 				//使能TIM4时钟
	GPIO_Set(GPIOD,1<<15,1,0,3,2);      //PD15,普通输出，下拉,100Mhz,推挽输出
	GPIO_Set(GPIOD,1<<14,0,0,3,2);		//PD14,输入， 下拉
	GPIO_AF_Set(GPIOD,14,2);            //PD14,AF2
	
	TIM4->ARR=arr;
	TIM4->PSC=psc;
	TIM4->CCMR2|=1<<0;     //CC3S=01 	选择输入端 IC3映射到TI3上
	TIM4->CCMR2|=0<<4;    //IC3F=0000 配置输入滤波器 不滤波
	TIM4->CCMR2|=0<<2;    //IC3PS=00 	配置输入分频,不分频 
	TIM4->CCER|=0<<9;     //CC3P=0	上升沿捕获
	TIM4->CCER|=1<<8;     //CC3E=1 	允许捕获计数器的值到捕获寄存器(CCR3)中
	TIM4->EGR|=1<<3;	  //软件控制产生更新事件，使写入PSC的值立即生效
	TIM4->DIER|=1<<3;	  //允许捕获3中断
	TIM4->DIER|=1<<0;	  //允许更新中断
	TIM4->CR1|=1<<0;	  //使能定时器4
	
	MY_NVIC_Init(2,0,TIM4_IRQn,2);//抢占2，子优先级0，组2

}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM4CH3_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM4CH3_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
//定时器4中断服务程序	 
void TIM4_IRQHandler(void)
{
	u16 tsr=TIM4->SR;
	if((TIM4CH3_CAPTURE_STA&0x80)==0)	//还未成功捕获
	{
		if(tsr&0x01)//溢出
		{
			if(TIM4CH3_CAPTURE_STA&0x40)//已经捕获到高电平了
			{
				if((TIM4CH3_CAPTURE_STA&0x3F)==0x3F)//高电平太长了
				{
					TIM4CH3_CAPTURE_STA|=0x80;//标记成功捕获了一次
					TIM4CH3_CAPTURE_VAL=0xffff;
				}else TIM4CH3_CAPTURE_STA++;
			}
		}
		if(tsr&0x10)//捕获4发生捕获事件
		{
			if(TIM4CH3_CAPTURE_STA&0x40) //捕获到一个下降沿 		
			{
				TIM4CH3_CAPTURE_STA|=0x80;  //标记成功捕获到一次高电平脉宽
				TIM4CH3_CAPTURE_VAL=TIM4->CCR3;//获取当前的捕获值.
				TIM4->CCER&=~(1<<13);			//CC4P=0 设置为上升沿捕获
			}else								//还未开始,第一次捕获上升沿
				{
					TIM4CH3_CAPTURE_STA=0;		//清空
					TIM4CH3_CAPTURE_VAL=0;
					TIM4CH3_CAPTURE_STA|=0x40;	//标记捕获到了上升沿
					TIM4->CR1&=~(1<<0);			//失能定时器3
					TIM4->CNT=0;				//计数器清空
					TIM4->CCER|=1<<9;			//CC3P=1 设置为下降沿捕获
					TIM4->CR1|=0X01;			//使能定时器3
				}
		}
	}
	TIM4->SR=0;	//清除中断标志
}

/*
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
	{
		USART_SendData(USART2,USART_ReceiveData(USART2));
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	}
	
}
*/
//串口6来显示所测高度等数据
void Csb_Test()
{	
		u32 temp=0;
		float length;//length声明为浮点数型(实型)
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 while(1)
	 {
		
		GPIO_SetBits(GPIOD,GPIO_Pin_15);//把PB7(TRIG 发射)管脚置高	
		delay_us(20);//拉高超过10us,发射超声波
		GPIO_ResetBits(GPIOD,GPIO_Pin_14);
		//等待ECHO PD14脚高电平		
		if(TIM4CH1_CAPTURE_STA&0X80)//成功捕获到了一次高电平
		{
			temp=TIM4CH1_CAPTURE_STA&0X3F;
			temp*=65536;					//溢出时间总和
			temp+=TIM4CH1_CAPTURE_VAL;		//得到总的高电平时间
			printf("echo高电平时间为:%d us\r\n",temp);	//打印总的高点平时间
			
			length=temp*0.0172;
			printf("测试距离=%f cm\r\n",length);
			
 			TIM4CH1_CAPTURE_STA=0;			//开启下一次捕获
			 
 		}
		   
		delay_ms(20);
		
	} 
}