/*************************************************************************************
** @file mpu9150.h
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
++             some functions are borrowed from github.com - wesleymilan
++              
**************************************************************************************/

#ifndef _MPU9150_H
#define _MPU9150_H

  /*include library header files------------------------------------------*/
 	#include <math.h>
	#include "inv_mpu.h"
	#include "inv_mpu_dmp_motion.h"
	#include "dmpmap.h"
	#include "dmpKey"
	#include "stm32f4xx.h" 

	
  /*define---------------------------------------------------------------*/
	
	/*Magnetometer Index*/
	#define Mag_X 0x00
	#define Mag_Y 0x01
	#define Mag_z 0x02
	
	/*Accleration index*/
	#define Accl_X 0x00
	#define Accl_Y 0x01
	#define Accl_Z 0x02

  /*Gyroscope index*/
	#define Gyro_X 0x00
	#define Gyro_Y 0x01
	#define Gyro_Z 0x01
	
	/*Quaternion index*/
	#define Quat_W 0x00
	#define Quat_X 0x01
	#define Quat_Y 0x02
	#define Quat_Z 0x03
	
  /*define X Y Z*/
	#define Vector_X 0
	#define Vector_Y 1
	#define Vector_Z 2
	
	#define m_PI 3.1415926f
	
	/* This mask defines the scaled range for mag and accel values*/
	#define SENSOR_RANGE 4096
	
	/*restrict angle range 0 to 2*PI*/
	#define over2PIAngle(A)  ((A) > (2.0f * m_PI) ? (A - 2.0f * m_PI) : (A))
	#define less0Angle(A)  ((A) < (0) ? (A + 2.0f * m_PI) : (A))
	
	
	#define quatNormlength  sqrt(rawQuat[Quat_W] * rawQuat[Quat_W] + rawQuat[Quat_X] * rawQuat[Quat_X] + \
		                      rawQuat[Quat_Y] * rawQuat[Quat_Y] + rawQuat[Quat_Z] * rawQuat[Quat_Z])
	
  /*end define-------------------------------------------------------------*/
	
  /*struct-----------------------------------------------------------------*/	

/******************************************************************************
  *@struct sensors status struct
	*****************************************************************************/
	
typedef struct {
	unsigned char dmp_on_or_off;
	unsigned short lpf;
	unsigned char fsr[3];
	unsigned char sample_rate;
	unsigned char compass_sample_rate;
	unsigned char fifo_config;		
}mpu9150Status;
	
/******************************************************************************
  *@struct Calibration data struct
	*****************************************************************************/

typedef struct{
	
	/*Accleration calibration maximum and minimum*/
	short calAcclXMax;
	short calAcclXMin;
	short calAcclYMax;
	short calAcclYMin;
	short calAcclZMax;
	short calAcclZMin;
	
	/*Mage calibration maximum and minimum*/	
	short calMagXMax;
	short calMagXMin;
	short calMagYMax;
	short calMagYMin;
	short calMagZMax;
	short calMagZMin;
		
}CalData;

  /*end struct declaration-----------------------------------------------------*/

  /*variable declaration------------------------------------------------------*/

	/*****************************************************************************
   *@raw data getting from dmp
   ****************************************************************************/

	 short m_rawGyro[3];
	 short m_rawAccl[3];
	 long m_rawQuat[4];
	 short m_rawMag[3];

	 float m_normQuat[4];
	
	/*****************************************************************************
   *@euler angles getting from normlized quaternion
   *****************************************************************************/

	 float m_Eulerangles[3];
	
	/*****************************************************************************
   *@the calibration datas of mag
   *****************************************************************************/
	 
	 short m_magCal[3];
	 short m_acclCal[3];
	
	/*****************************************************************************
	 *@the fused euler angles and the fused quaternion
	 *****************************************************************************/
	 
	 float m_fusedEulerangles[3];
	 float m_fusedQuaternion[4];
	 
	/*****************************************************************************
	 *@set true as calibration in use
	 *****************************************************************************/
	 
	 CalData m_CalData;
	 bool m_useMagCal = true;                            
	 bool m_useAcclCal = true;   
   bool m_useGyroCal = false;
	 
	/*****************************************************************************
 	 *@last dmp output yaw and last output yaw
	 *****************************************************************************/
		
	 float m_dmplastYaw;         
   float m_lastYaw;	                           
	
	/******************************************************************************  
	 *@magMix controls the amoutn of influence that the magnetometer has on yaw:
   *0 = just use MPU gyros (will not be referenced to north)
   *1 = just use magnetometer with no input from gyros
   *2-n = mix the two. Higher numbers decrease correction from magnetometer
	 ******************************************************************************/
	 
	 short m_magMix;                            

	
	/******************************************************************************
	 *@offset data
	 ******************************************************************************/
	 
	 long m_acclOffset[3];
	 short m_magOffset[3];
	 short m_acclRange[3];
   short m_magRange[3];        	
	
  /*variable declaration end---------------------------------------------------*/

	/*functions declaration-------------------------------------------------------*/

  /*****************************************************************************
	 *@brief initialize mpu9150 
	 *@param[in] mpuRate set the mpu9150 update rate in hz
   *           	magRate is the magnetometer update in hz which can not be >= 100
	 *                      default is 10
	 *            lpf is the low pass flitter setting, which can be setted between
	 *                5hz and 188hz  if it is 0,use the default number.
	 *@return ture denote initialize successfully and flase denote initialize unsuccessfully
	 ******************************************************************************/
	 
    bool init_mpu9150(int mpuRate,int magRate = 10,int lpf = 0);

  /******************************************************************************
	 *@breif this function is to get sensors data from mpu9150,including gyro,
	 *             acclaration,and quats
	 *@return if which reterval is false denotes somewhere have a problem                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	 ******************************************************************************/
	 
	  bool getDMPdata();
	
	  /****************************************************************************
	   *@breif this function is to get sensor's current state                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
	   ****************************************************************************/
		 
	  void getStatus();
	 
	 
	  void useMagcal();
	  void useGyrocal();
	  void useAcclcal();
	 
	  void disableMagCal();
	  void disableAcclCal();
	
	 	/****************************************************************************
		 *@briefupdate the calibration Data
		 ****************************************************************************/
		 
	  void updateCalData(); 
	
	 /*****************************************************************************
	 *@brief fuse the mag data with quaternion
	 ******************************************************************************/
	 
	 void dataFused();
	 
  /*functions declaration end---------------------------------------------------*/
	
#endif    /*MPU9150_H END*/