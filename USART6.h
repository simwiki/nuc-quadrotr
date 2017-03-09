#ifndef _USART6_H
#define _USART6_H
#include "usart.h"
#include "stm32f4xx.h"

void Usart6_SendBuffer(float * rx_buf, uint8_t len);
void Usart6_Send_Char(u8 c);
void Usart6_Send_To_Shangweiji(u8 fun,u8 * data,u8 len);

#endif
