#include "USART6.h"

extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
//打印串口DR中的数据
void usart6_SendBuffer(float * rx_buf,uint8_t len)
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
