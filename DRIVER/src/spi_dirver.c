/*************************************************************************************
** @file spi_driver.c
** @author codeKlidd_ming
** @version fly_dream-V1.0
** @date 
** @decription 
**              
**
**
**************************************************************************************
++ @attention  
++            
++            
**************************************************************************************/

/*include----------------------------------------------------------------------------*/
#include "spi_driver.h"

/*source code------------------------------------------------------------------------*/

 /**
 * @brief  Select spi and initialize the SPI_Driver struct
 * @param[in] SPI_TypeDef* SPIx
 *           
 * @param[out]       
 **/
 
 void SPI_Struct_Init(SPI_Driver* SPI_driv,SPI_TypeDef* SPIx)
 {	 
	 SPI_driv->SPI = SPIx;
	 if (SPIx == SPI1)
	 {
		 SPI_driv->GPIO = GPIOA;
		 SPI_driv->SPI_SCK_Pin = GPIO_Pin_5;
		 SPI_driv->SPI_MISO_Pin = GPIO_Pin_6;
		 SPI_driv->SPI_MOSI_Pin = GPIO_Pin_7;
		 SPI_driv->SPI_RCC_Peripherals = RCC_APB2Periph_SPI1;
		 SPI_driv->GPIO_RCC_Peripherals = RCC_AHB1Periph_GPIOA;
		 SPI_driv->Pin_Source.CLK = GPIO_PinSource5;
		 SPI_driv->Pin_Source.MISO = GPIO_PinSource6;
		 SPI_driv->Pin_Source.MOSI = GPIO_PinSource7;		
     SPI_driv->AF_function_Selct = GPIO_AF_SPI1; 		
     SPI_driv->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		 
		 SPI_driv->SPI_PeriphalCLK = RCC_APB2PeriphClockCmd;
	 }
	 else if (SPIx == SPI2)
	 {
		 SPI_driv->GPIO = GPIOB;
		 SPI_driv->SPI_SCK_Pin = GPIO_Pin_13;
		 SPI_driv->SPI_MISO_Pin = GPIO_Pin_14;
		 SPI_driv->SPI_MOSI_Pin = GPIO_Pin_15;
		 SPI_driv->SPI_RCC_Peripherals = RCC_APB2Periph_SPI2;
		 SPI_driv->GPIO_RCC_Peripherals = RCC_AHB1Periph_GPIOB;
		 SPI_driv->Pin_Source.CLK = GPIO_PinSource13;
		 SPI_driv->Pin_Source.MISO = GPIO_PinSource14;
		 SPI_driv->Pin_Source.MOSI = GPIO_PinSource15;		
     SPI_driv->AF_function_Selct = GPIO_AF_SPI2; 		
     SPI_driv->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	
		 SPI_driv->SPI_PeriphalCLK = RCC_APB1PeriphClockCmd;		 
	 }
	 else IF (SPIx == SPI3)
	 {
		 SPI_driv->GPIO = GPIOB;
		 SPI_driv->SPI_SCK_Pin = GPIO_Pin_3;
		 SPI_driv->SPI_MISO_Pin = GPIO_Pin_4;
		 SPI_driv->SPI_MOSI_Pin = GPIO_Pin_5;
		 SPI_driv->SPI_RCC_Peripherals = RCC_APB2Periph_SPI3;
		 SPI_driv->GPIO_RCC_Peripherals = RCC_AHB1Periph_GPIOB;
		 SPI_driv->Pin_Source.CLK = GPIO_PinSource3;
		 SPI_driv->Pin_Source.MISO = GPIO_PinSource4;
		 SPI_driv->Pin_Source.MOSI = GPIO_PinSource5;		
     SPI_driv->AF_function_Selct = GPIO_AF_SPI3; 		
     SPI_driv->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	
		 SPI_driv->SPI_PeriphalCLK = RCC_APB1PeriphClockCmd;		 
	 }
 }
 
  /**
 * @brief   Configer gpio and initialize spi configeration,then set the speed transmission
 * @param[in] SPI_Driver* SPI_driv
 *           
 * @param[out]       
 **/
void SPI_Config(SPI_Driver* SPI_driv)
{
	  GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
	
	  RCC_AHB1PeriphClockCmd(SPI_driv->GPIO_RCC_Peripherals,ENABLE);
	
	 /*use function pointer to distinguish spi periphal clock cmd*/
	  (*(SPI_driv->SPI_PeriphalCLK))(SPI_driv->SPI_RCC_Peripherals,ENABLE);
	
	 /*gpio initialize and ports configer */
    GPIO_InitStructure.GPIO_Pin = SPI_driv->SPI_SCK_Pin|SPI_driv->SPI_MISO_Pin| \
	                                  SPI_driv->SPI_MOSI_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SPI_driv->GPIO, &GPIO_InitStructure);
		  
   /*spi initialize*/
    (*(SPI_driv->SPI_PeriphalCLK))(SPI_driv->SPI_RCC_Peripherals,ENABLE);  /*reset spi clock*/
	  (*(SPI_driv->SPI_PeriphalCLK))(SPI_driv->SPI_RCC_Peripherals,DISENABLE); /*stop reset spi clock*/

  	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   
	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	
	  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		          
	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	
  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		
  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	
	  SPI_InitStructure.SPI_CRCPolynomial = 7;	
	  SPI_Init(SPI_driv->SPI, &SPI_InitStructure);  
	
	 /*set spi speed*/
		Clear_SPI_BaudRate;
	  Set_SPI_BaudRate(SPI_driv->SPI_BaudRatePrescaler);
	
	  SPI_Cmd(SPI_driv->SPI,ENABLE);
	 /*start tansmission*/
		SPI1_ReadWriteByte(0xff);
}