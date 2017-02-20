/*************************************************************************************
** @file spi_driver.h
** @author codeKlidd_ming
** @version fly_dream-V1.0
** @date 
** @decription 
**              
**              
**              
**              
**              
**************************************************************************************
++ @attention  
++            
++              
**************************************************************************************/
#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

/*include--------------------------------------------------------------*/
#include "stm32f4xx.h" 
#include "stm32f4xx_rcc.h"

/*define---------------------------------------------------------------*/

/*************************************************************************
 *@brief set SPI Baud speed Rate
 *************************************************************************/
 
#define Clear_SPI_BaudRate   SPI1->CR1 &= 0XFFC7
#define Set_SPI_BaudRate(BaudRate)     SPI1->CR1 |= BaudRate 

/************************************************************************
 *@brief declear the nrf24l01 used gpio pin
 *************************************************************************/
 
typedef struct
{
	uint8_t CLK;
	uint8_t MISO;
	uint8_t MOSI;
}GPIO_Pin_Source;

typedef enum
{
	spi1,
	spi2,
	spi3,
	spi4
}usedspi;

usedspi mUSE_SPI;

/********************************************************************************
 *
 ********************************************************************************/
#if mUSE_SPI == spi1

#define mSPIx                         SPI1
#define mSPI_GPIO                     GPIOA
#define mSPI_SCK_Pin                  GPIO_Pin_5
#define mSPI_MISO_Pin                 GPIO_Pin_6
#define mSPI_MOSI_Pin                 GPIO_Pin_7
#define mSPI_RCC_Peripherals          RCC_APB2Periph_SPI1
#define mGPIO_RCC_Peripherals         RCC_AHB1Periph_GPIOA
#define mPin_Source_CLK               GPIO_PinSource5
#define mPin_Source_MISO              GPIO_PinSource6
#define mPin_Source_MOSI              GPIO_PinSource7
#define mAF_function_Selct            GPIO_AF_SPI1
#define mSPI_BaudRatePrescaler        SPI_BaudRatePrescaler_8

#elif mUSE_SPI == spi2

#define mSPIx                         SPI2
#define mSPI_GPIO                     GPIOB
#define mSPI_SCK_Pin                  GPIO_Pin_13
#define mSPI_MISO_Pin                 GPIO_Pin_14
#define mSPI_MOSI_Pin                 GPIO_Pin_15
#define mSPI_RCC_Peripherals          RCC_APB2Periph_SPI2
#define mGPIO_RCC_Peripherals         RCC_AHB1Periph_GPIOB
#define mPin_Source_CLK               GPIO_PinSource13
#define mPin_Source_MISO              GPIO_PinSource14
#define mPin_Source_MOSI              GPIO_PinSource15
#define mAF_function_Selct            GPIO_AF_SPI2
#define mSPI_BaudRatePrescaler        SPI_BaudRatePrescaler_8

#elif mUSE_SPI == spi3

#define mSPIx                         SPI3
#define mSPI_GPIO                     GPIOB
#define mSPI_SCK_Pin                  GPIO_Pin_3
#define mSPI_MISO_Pin                 GPIO_Pin_4
#define mSPI_MOSI_Pin                 GPIO_Pin_5
#define mSPI_RCC_Peripherals          RCC_APB2Periph_SPI3
#define mGPIO_RCC_Peripherals         RCC_AHB1Periph_GPIOB
#define mPin_Source_CLK               GPIO_PinSource3
#define mPin_Source_MISO              GPIO_PinSource4
#define mPin_Source_MOSI              GPIO_PinSource5
#define mAF_function_Selct            GPIO_AF_SPI3
#define mSPI_BaudRatePrescaler        SPI_BaudRatePrescaler_8

#endif

/***************************************************************************
 *@brief functions declaration
 ***************************************************************************/

uint8_t mSPI_ReadWriteByte(uint8_t TxData);
void mSPI_Config(void);
void mSPI_Init(void);

#endif
