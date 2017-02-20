/*************************************************************************************
** @file d_mpu9150.h
** @author codeKiller_ming
** @version fly_dram-V1.0
** @date 2016-11-14
** @description this file contains all the functions protypes for MPU9150 firmware
**              library.
*************************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _I2C_DRIVER_H
#define _I2C_DRIVER_H

/*include---------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "inv_mpu.h"
#include "in_mpu_dmp_motion.h"

/*define----------------------------------------------------------------------*/
#define m_I2C   I2C1
#define mI2C_Speed 200000
#define mI2C_Address 0x49             /*slaver address*/

/***
**@defgroup defination of I2C GPIO,the Pins can be changed acording to demand 
**@{
***/

#if defined USE_IO_SIMULATE_I2C
#if m_I2C == I2C1
#define SCL_Pin  GPIO_Pin_7
#define SDA_Pin  GPIO_Pin_6 
#define SDA_H    GPIOB->BSRRH = SDA_Pin
#define SDA_L    GPIOB->BSRRL = SDA_Pin
#define SCL_H    GPIOB->BSRRH = SCL_Pin
#define SCL_L    GPIOB->BSRRL = SCL_Pin
#define SDA_Read GPIOB->IDR & SDA_Pin 

#elif m_I2C == I2C2
#define SCL_Pin  GPIO_Pin_10
#define SDA_Pin  GPIO_Pin_11 
#define SDA_H    GPIOB->BSRRH = SDA_Pin
#define SDA_L    GPIOB->BSRRL = SDA_Pin
#define SCL_H    GPIOB->BSRRH = SCL_Pin
#define SCL_L    GPIOB->BSRRL = SCL_Pin
#define SDA_Read GPIOB->IDR & SDA_Pin 

#elif m_I2C == I2C3
#define SDA_Pin  GPIO_Pin_9
#define SCL_Pin  GPIO_Pin_8
#define SDA_H    GPIOC->BSRRH = SDA_Pin
#define SDA_L    GPIOC->BSRRL = SDA_Pin
#define SCL_H    GPIOA->BSRRH = SCL_Pin
#define SCL_L    GPIOA->BSRRL = SCL_Pin
#endif
#endif 

/***
**@}
***/

#endif    /*_I2C_DRIVER_H*/