#include "USART6.h"

extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
//打印串口DR中的数据
void Usart6_SendBuffer(float * rx_buf,uint8_t len)
{
	u8 t;
	u16 times=0;
	if(USART_RX_STA&0x80)
		{
			len=USART_RX_STA&0x3fff;
			printf("\r\n您发送的消息为\r\n");
			for(t=0;t<len;t++)
			{
				USART6->DR=USART_RX_BUF[t];
				while((USART6->SR&0X40)==0);//等待发送结束
			}
			printf("\r\n\r\n");
			USART_RX_STA=0;
		}else
		{
			times++;
			if(times%200==0)
				printf("请输入数据，以回车结束");
		}
}

//串口6发送一个字节，c:要发送的字符
void Usart6_Send_Char(u8 c)
{
	while(USART6->SR == 0); //等待上一次发送完毕
	USART6->DR = c;
}

//传送数据到上位机
//fun:功能字.0XA0~0XAF; data:数据缓存区，最多28个字节
//len:data区有效数据个数
void Usart6_Send_To_Shangweiji(u8 fun,u8 * data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len > 28) return;	//最多28字节数据
	send_buf[len+3] = 0;	//校验数置零
	send_buf[0] = 0X88;		//帧头
	send_buf[1] = fun;		//功能字
	send_buf[2] = len;		//数据长度
	for(i = 0;i < len;i++) send_buf[3+i] = data[i];					//复制数据
	for(i = 0;i < len + 3;i++) send_buf[len+3] += send_buf[i];		//计算校验和
	for(i = 0;i < len + 4;i++) Usart6_Send_Char(send_buf[i]);		//发送数据到串口6
}

