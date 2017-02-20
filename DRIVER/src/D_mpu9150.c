/*************************************************************************************
** @file D_mpu9150
** @author codeKlidd_ming
** @version fly_dream-V1.0
** @date 2016-11-14
** @decription this file fiamware functions to manage the following functionalities
**              of the mpu9150:
**             $ mpu9150 and IIC configeration;
**             $ aquire accel,gyr and angle form quat 
**************************************************************************************
++ @attention  


**************************************************************************************/

/*include----------------------------------------------------------------------------*/
#include "D_mpu9150.h"

/***
*      @defgroup mpu9150_drive
*      @{
*      @brief Get 9-axis angles through manituding the DMP,and aquire the accel 
*             and gyro by the way. <- ->!
*
***/
/***       
*       @defgroup I2C_USED
*       @{
*       @brief In this section,configer we used I2C,and definate the STM32_i2c_write
*               and STM32_i2c_read which are used in inv_mpu.c  <- ->!
*
****/
 

/**
 * @brief configer i2c gpio
 * @param[in] I2Cx where x can be 1 2 3 to select
 * @retval 0is successful
 **/
 static int8_T mI2C_GPIO_Configeration(I2C_TypeDef* I2Cx)
 { 
	 GPIO_InitTypeDef GPIO_InitStructure; 
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB,ENABLE);
	
	 if(I2Cx==I2C1){
		 RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE)
		 /*configer SDA SCL GPIO*/
		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //GPIO Alternate function Mode
		 GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;   //open drain output
		 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHZ;
		 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		 GPIO_Init(GPIOB,&GPIO_InitStructure);		 
	 }
	 else if(I2Cx == I2C2){
		 RCC_APB1Peri  phClockCmd(RCC_APB1Periph_I2C2,ENABLE)
		 /*configer SDA SCL GPIO*/
		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11; //I2C_SCL------SDA
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //GPIO Alternate function Mode
		 GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;   //open drain output
		 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHZ;
		 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		 GPIO_Init(GPIOB,&GPIO_InitStructure);		 
	 }
	 else if(I2Cx == I2C3){
		 RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE)
		 /*configer SDA SCL GPIO*/
		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //GPIO Alternate function Mode
		 GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;   //open drain output
		 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHZ;
		 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  // PuPd_UP
		 GPIO_Init(GPIOC|GPIOA,&GPIO_InitStructure);				 
	 }
		 return 0
 }
 /*use st standard i2c library */
 #ifdef USE_I2C_STANDARD_LIBRARY
 /**
 * @brief configer i2c 
 * @param[in] I2CX where x can be (1,2,3) to select
 * @retval 0is successful
 **/
 int8_t mI2C_Configeration(I2C_TypeDef* I2Cx)
 {
	 I2C_InitTypeDef I2C_InitStructure;
	 /*I2Cx GPIO configeration*/
	 mI2C_GPIO_Configeration(I2Cx);
	 /*I2Cx Default Init*/
	 I2C_DeInit(I2Cx);
	 /*I2C Strcture Paramters Init*/
	 I2C_InitStructure.ACK = I2C_ACK_Enable;
	 I2C_InitStructure.AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	 I2C_InitStructrue.ClockSpeed = mI2C_Speed;
	 I2C_InitStructrue.DutyCycle = Duty_Cycle_2  //which is less understanding
	 I2C_InitStructure.Mode = I2C_Mode_I2C;
	 I2C_InitStructure.OwnAddress1 = mI2C_Address;   //select our own Address
	 /*I2C Configeration finished*/
	 
	 /*Enable I2C & Init I2C*/
	 I2C_Init(I2Cx,&I2C_InitStructure);
	 I2C_Cmd(I2Cx,ENABLE);
	 
	 return 0;
 }
 
  /**
 * @brief I2C send Bytes function using ST standard library 
 * @param[in] 1)I2Cx where can be (I2C1 I2C2 I2C3)to select.2)Slave Address.
 *            3)Reg Address.4)Data(Send Buff) and Data length(len).
 * @param[out] NONE
 * @retval 0is successful,-1isI2C Bus is busy,-2is Start Failed
 *         -3is Master doesn't get ACK from slaver, -4is Send Regsiter address failed
 *         -5is Send Data failed 
 **/
 int8_t mI2C_SendByte(I2C_Typedef* I2Cx,uint8_t Slave_addr,uint8_t Reg_addr,uint8_t len,uint8_t *Send_Buff)
 {
	 int16_t Wait_Time = 0x1ff;
	 while((!I2C_GetFlagStatus(I2Cx,I2C_FLAGBUSY)) && Wait_Time--);
		 if(-1 == Wait_Time){
			 return -1;                   /*Time spill out,I2C Bus is busy*/
		 }
	 /*I2C generate start signal*/
	 I2C_GenerateSTART(I2Cx,ENABLE);
	 uint8_t Wait_Time = 0xff;
	 while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT) && Wait_Time--);
		 if(-1 == Wait_Time){
			 return -2;          /*Time spill out,Start Failed*/
		 }
	 /*Send Address*/
	 I2C_Send7biteAddress(I2Cx,Slave_addr,I2C_Direction_Transmitter);
	 uint8_t Wait_Time = 0xff;
	 while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && Wait_Time--);
		 if(-1 == Wait_Time){
			 I2C_GenerateSTOP(I2Cx,ENABLE);   /*Stop I2C Bus*/
			 return -3;          /*Time spill out,Master doesn't get ACK from slaver*/
		 }
	 /*Connection is successful and Send Regsiter Address*/
	 I2C_SendData(I2Cx,Reg_addr);
	 uint8_t Wait_Time = 0xff;
	 while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED) && Wait_Time--);
		 if(-1 == Wait_Time){
			 I2C_GenerateSTOP(I2Cx,ENABLE);    /*Stop I2C Bus*/
			 return -4;          /*Time spill out,Send Regsiter address failed*/
		 	 }
	 /*if Connection between master and slaver is successful,then send Data Buff*/
	 while(len--){
		 I2C_SendData(I2Cx,*(Send_Buff++));
		 uint8_t Wait_Time = 0xff;
		 while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED) && Wait_Time--);
			 if(-1 == Wait_Time){
				 I2C_GenerateSTOP(I2Cx,ENABLE);
				 return -5;      /*Time spill out,Send Data failed*/
			 }
	 }
	 /*Send Data Finish Stop I2C Bus*/
	 I2C_GenerateSTOP(I2Cx,ENABLE);
	 return 0;  /*I2C Send Data is successful*/
 }
 
 /**
 * @brief I2C Receive Bytes function using ST standard library
 * @param[in] 1)I2Cx where can be (I2C1 I2C2 I2C3)to select.2)Slave Address.
 *            3)Reg Address.4)Data(Receive Buff) and Data length(len).
 * @param[out] NONE
 * @retval 0is successful,-1isI2C Bus is busy,-2is Start Failed
 *         -3is Master doesn't get ACK from slaver, -4is Send Regsiter address failed
 *         -5is Receive Data failed 
 **/
 int8_t mI2C_ReceiveByte(I2C_Typedef* I2Cx,uint8_t Slave_Addr,uint8_t Reg_Addr,uint8 len,uint8_t *Recv_Buff)
 {
	 /*Check I2C whether is busy*/
	 uint16_t Wait_Time = 0x1ff;
	 while(!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY) && Wait_Time--);
		 if(-1 == Wait_Time){
			 return -1;   /*Time spill out ,I2C Bus is busy*/
		 }
	 /*Generate a start signal*/
	 I2C_GenerateSTART(I2Cx,ENABLE);
	 uint8_t Wait_Iime = 0xff;
	 while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT) && Wait_Time--);
		 if(-1 == Wait_Time){
			 return -2;    /*Time spill out ,Start failed*/
		 }
	 /*Send Slaver address when send regsiter address*/
	 I2C_Send7biteAddress(I2Cx,Slave_Addr,I2C_Direction_Transmitter);
	 uint8_t Wait_Time = 0xff;
	 while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && Wait_Time--);
		 if(-1 == Wait_Time){
			 return -3;  /*Time spill out Slaver address send failed,doesn's receive ack*/
		 }
	 /*Send Data Regsiter address*/
	 I2C_SendData(I2Cx,Reg_Addr);
	 uint8_t Wait_Time = 0xff;
	 while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED) && Wait_Time--);
		 if(-1 == Wait_Time){
			 return -4;     /*Wait Time spill out Send Regsiter address failed*/
		 }
	 /*Regenerate Start I2C*/
	 I2C_GenerateSTART(I2Cx,ENABLE);
	 uint8_t Wait_Time = 0xff;
	 while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT) && Wait_Time--);
		 if(-1 == Wait_Time){
			 return -2;   /*regenerate is failed*/
		 }
	 /*Send Slaver address when receive data*/
	 I2C_Send7biteAddress(I2Cx,Slave_Addr,I2C_Direction_Receiver);
	 uint8_t Wait_Time = 0xff;
	 while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && Wait_Time--);
		 if(-1 == Wait_Time){
			 return -3;    
		 }
	 /*Continuously receive data*/
	 while(len){
		 if(1 == len){
			 /*generate a no ack when read the last Byte*/
			 I2C_AcknowledgeConfig(I2Cx,DISABLE);
			 I2C_GenerateSTOP(I2Cx,ENABLE);
		 }
	 uint8_t Wait_Time = 0xff;
	 while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED) && Wait_Time--);
		 if(-1 == Wait_Time){
			 return -5;    /*Receive Data failed*/
		 }
	 *(Recv_Buff++) = I2C_ReceiveData(I2Cx;
   len--;
  }
   I2C_AcknowledgeConfig(I2Cx,ENABLE);
   I2C_GenerateSTOP(I2Cx,ENABLE);
   return 0;    /*Receive data successful*/	
}
 
 /**
 * @brief Declare the stm32_i2c_write function that mpu used
 * @param[in] 1)Slave Address.2)Reg Address.
 *            3)Data(Send Buff) and Data length(len).
 * @param[out] NONE
 * @retval 0is successful
 **/
int8_t STM32_i2c_write(uint8_t Slave_Addr,uint8_t Reg_Addr,uint8 len,uint8_t const *Send_Buff)
{
	mI2C_GPIO_Configeration(m_I2C);
	mI2C_Configeration(m_I2C);
	return mI2C_SendByte(m_I2C,Slave_Addr << 1,Reg_Addr,len,Send_Buff);
}
 /**
 * @brief Declare the stm32_i2c_read function that mpu used
 * @param[in] 1)Slave Address.2)Reg Address.
 *            3)Data(Receive Buff) and Data length(len).
 * @param[out] Received data 
 * @retval 0is successful
 **/
 int8_t STM32_i2c_read(uint8_t Slave_Addr,uint8_t Reg_Addr,uint8 len,uint8_t *Recv_Buff)
{
	mI2C_GPIO_Configeration(m_I2C);
	mI2C_Configeration(m_I2C);
  return mI2C_ReceiveByte(m_I2C,Slave_Addr << 1,Reg_Addr,len,Recv_Buff); 	
}
 /**
 * @brief use gpio generate i2c 
 **/
#elif USE_IO_SIMULATE_I2C  

 /**
 * @brief I2C delay
 * @param[in] delay time
 * @param[out] NONE
 * @retval NONE
 **/
static void delay(time)
{
	while(time--);
}
 /**
 * @brief I2C start
 * @param[in] NONE
 * @param[out] NONE
 * @type STATIC
 * @retval -1is failed,0is successful
 **/
static int8_t I2C_Start()
{
	SCL_L;
	SDA_H;
	SCL_H;
	/*detect I2C whether is busy*/
	if(!SDA_Read)  return -1;    /*bus wire is busy*/
	SDA_L;
	if(SDA_Read)   return -1;    /*failed to set the level of the SDA_Pin */
	SCL_L;
	return 0;
}
 /**
 * @brief I2C stop
 * @param[in] NONE
 * @param[out] NONE
 * @type STATIC
 * @retval NONE
 **/
static void I2C_Stop()
{
	SCL_L;
	SDA_L;
	SCL_H;
	SDA_H;
}
 /**
 * @brief I2C generate ACK
 * @param[in] NONE
 * @param[out] NONE
 * @type STATIC
 * @retval NONE
 **/
static void I2C_Ack()
{
	SCL_L;
	SDA_L;
	SCL_H;
	SCL_L;
}
 /**
 * @brief I2C generate NACK
 * @param[in] NONE
 * @param[out] NONE
 * @type STATIC
 * @retval NONE
 **/
static void I2C_NAck()
{
	SCL_L;
	SDA_H;
	SCL_H;
	SCL_L;
}
 /**
 * @brief I2C Wait Ack 
 * @param[in] NONE
 * @param[out] NONE
 * @type STATIC
 * @retval -1is failed,0is successful
 **/
static int8_t I2C_WaitAck()
{
	SCL_L;
	SDA_L;
	SCL_H;
	if(SDA_Read){
		SCL_L;                /*SDA remians high level, fail!*/
		return -1;
	}
	SCL_L;
	return 0;
}
 /**
 * @brief I2C Write a single Byte
 * @param[in] write data
 * @param[out] NONE
 * @type STATIC
 * @retval NONE
 **/
static void Write_aByte(uint8_t Write_Data)
{
	uint8_t count;          
	while(count--){
		SCL_L;
		if(Write_Data&0x80)
			SDA_H;
		else 
			SDA_L;
		Write_Data <<= 1;
		SCL_H;
	}
	SCL_L;
}
 /**
 * @brief I2C Read a single Byte
 * @param[in] the point of Read data buff
 * @param[out] Read Data
 * @type STATIC
 * @retval NONE
 **/
static void Read_aByte(uint8_t *Read_Data)
{
	uint8_t count = 8;
	SDA_H;
	while(count--){
		SCL_L;
		(*Read_Data) <<= 1;
		SCL_H;
		if(SDA_Read)
			(*Read_Data) |= 0x01;
		SCL_L;
}
 /**
 * @brief I2C write sequent Byte
 * @param[in] Slave address,Regsiter address,Write Data Buff,and the length of data
 * @param[out] NONE
 * @type STATIC
 * @retval 0is successful,-1is failed
 **/
static int8_t mI2C_WriteByte(uint8_t Slave_Addr,uint8_t Reg_Addr,uint8_t len,uint8_t *Write_Buff)
{
	if(I2C_Start()){
		return -1;
	}
	Write_aByte(Slave_Addr);
	if(I2C_WaitAck()){
		I2C_Stop();
		return -1;
	}
	Write_aByte(Reg_Addr);
	if(I2C_WaitAck()){
		I2C_Stop();
		return -1;
	}
	while(len--){
	 Write_aByte(*(Write_Buff++));
	 if(I2C_WaitAck()){
		 I2C_Stop();
		 return -1;
	 }
 }
	I2C_Stop();
 return 0;
}
 /**
 * @brief I2C Read sequent Byte
 * @param[in] Slave address,Regsiter address,Read Data Buff,and the length of data
 * @param[out] Read Data 
 * @type STATIC
 * @retval 0is successful,-1is failed
 **/
static int8_t mI2C_ReadByte(uint8_t Slave_Addr,uint8_t Reg_Addr,uint8_t len,uint8_t *Read_Buff)
{
	if(I2C_Start()){
		return -1;
	}
	Write_aByte(Slave_Addr);/*config the write register mod*/
	if(I2C_WaitAck()){
		I2C_Stop();
		return -1;
	}	
	Write_aByte(Reg_Addr);
	if(I2C_WaitAck()){
		I2C_Stop();
		return -1;
	}
  I2C_Start();
	Write_aByte(Slave_Addr+1);/*config the read mod*/	
	I2C_WaitAck();
	while(--len){
		Read_aByte(Read_Buff++);
		I2C_Ack();
	}
	Read_aByte(Read_Buff);
	I2C_NAck();
	I2C_Stop();
	return 0;
}
 /**
 * @brief Declare the stm32_i2c_write function that mpu used
 * @param[in] 1)Slave Address.2)Reg Address.
 *            3)Data(Send Buff) and Data length(len).
 * @param[out] NONE
 * @retval 0is successful
 **/
int8_t STM32_i2c_write(uint8_t Slave_Addr,uint8_t Reg_Addr,uint8 len,uint8_t const *Send_Buff)
{
	mI2C_GPIO_Configeration(m_I2C);
	return mI2C_WriteByte(Slave_Addr << 1,Reg_Addr,len,Send_Buff);        /*/?*/
}
 /**
 * @brief Declare the stm32_i2c_read function that mpu used
 * @param[in] 1)Slave Address.2)Reg Address.
 *            3)Data(Receive Buff) and Data length(len).
 * @param[out] Received data 
 * @retval 0is successful
 **/
 int8_t STM32_i2c_read(uint8_t Slave_Addr,uint8_t Reg_Addr,uint8 len,uint8_t *Recv_Buff)
{
	mI2C_GPIO_Configeration(m_I2C);
  return mI2C_ReadByte(Slave_Addr << 1,Reg_Addr,len,Recv_Buff); 	 /*/?*/
}
/**
 *@}
 */

/**
  *@}
  */

#endif   /*I2C_DRIVER END*/

 