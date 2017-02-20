/*************************************************************************************
** @file mpu9150.c
** @author codeKlidd_ming
** @version fly_dream-V1.0
** @date    2017-2-20
** @decription 
**          this file contains such functions     
**          1. inverse matrix to scalar: inv_row_2_scalar and inv_orientation_matrix_2_scalar   
**          2. about quatrnion: normlizeQuat, Quaternion_2_Euler and Euler_2_Quaternion 
**                              quaternionConjugate, quaternionMultiply
**          3. the member function of class mpu9150: initialize function, getDMPdata()
**                                                   dataFused
**              
***********************************************************************************
++ @attention  
++            the definition of class mpu9150 is in mpu9150.h
++            
***********************************************************************************/


/**********************************************************************************      
*       @addtogroup mpu9150 implement the public functions
*       @{             
*
***********************************************************************************/

/*include-------------------------------------------------------------*/
#include "mpu9150.h"

/***********************************************************************
  *@brief these two next functions convert the orienment matrix to
	*       a scalar respresentation for using by dmp, which are borrowed from
	*       Invsense DML   
	**********************************************************************/

static unsigned short inv_row_2_scalar(const signed char *Row)
{
	 unsigned short scalar;
	
	
	 if (Row[0] > 0)
	 {
		 scalar = 0;
	 }
	 else if (Row[0] < 0)
	 {
		 scalar = 4;
	 }
	 else if (Row[1] > 0)
	 {
		 scalar = 1;
	 }
	 else if (Row[1] < 0)
	 {
		 scalar = 5;
	 }
	 else if(Row[2] > 0)
	{
		 scalar = 2;
	 }
	else if(Row[2] < 0)
	{
		 scalar = 6;
	 }
	else
	{
	     scalar = 7;                    //error!!
	}
	 
	 return scalar;
}	

/**************************************************************************
 *@brief orientation matrix
 *      the mounting matrix seen below tells the MPL how to rotate the raw 
 *      data from their drivers
 **************************************************************************/
	
static signed char gyro_orientation[9] = {1,0,0,
                                          0,1,0,
                                          0,0,1};
																								 
static inline unsigned char inv_orientation_matrix_2_scalar(signed short *matrix)
{
	unsigned char scalar;
	
	    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
	
	scalar = inv_row_2_scalar(matrix);
	scalar |= inv_row_2_scalar(matrix + 3) << 3;
	scalar |= inv_row_2_scalar(matrix + 6) << 6;
	
	return scalar;	
}	

/********************************************************************
 *@brief normlize quaternion 
 ********************************************************************/

static void normlizeQuat(float *norm_quat)
{
	float length = quatNormlength(norm_quat);
	
	norm_quat[Quat_W] /= length;
	norm_quat[Quat_X] /= length;
	norm_quat[Quat_y] /= length;
	norm_quat[Quat_Z] /= length;
}

/********************************************************************
 *@brief quaternion tansfer to eluer angles 
 ********************************************************************/

static void Quaternion_2_Euler(float *norm_quat, float *Eulerangles)
{
	float pole = mPI/2.0f - 0.05f;
	
	Eulerangles[Vector_Y] = asin(2.0f * (norm_quat[Quat_W] * norm_quat[Quat_Y] - norm_quat[Quat_X] * norm_quat[Quat_Z]));
	
	Eulerangles[Vector_Z] = atan2(2.0f * (norm_quat[Quat_X] * norm_quat[Quat_Y] + norm_quat[Quat_W] * norm_quat[Quat_Z]), \
	                                       1.0f - 2.0f*(norm_quat[Quat_Y] * norm_quat[Quat_Y] + norm_quat[Quat_Z] * norm_quat[Quat_Z]));	
	
	if ( ( Eulerangles[Vector_Y] < pole ) && ( Eulerangles[Vector_Y] > -pole ) )
	Eulerangles[Vector_X]	= atan2(2.0f * (norm_quat[Quat_Y] * norm_quat[Quat_Z] + norm_quat[Quat_W] * norm_quat[Quat_X]), \ 
                                    1.0f - 2.0f*(norm_quat[Quat_Y] * norm_quat[Quat_Y] + norm_quat[Quat_X] * norm_quat[Quat_X]));		
}

/********************************************************************
 *@brief  Euler angles tansfer to quaternion
 ********************************************************************/

static void Euler_2_Quaternion(const float *Eulerangles, float *norm_quat)
{
	float sinX = sin(Eulerangles[Vector_X] / 2.0f);
	float cosX = cos(Eulerangles[Vector_X] / 2.0f);
	float sinY = sin(Eulerangles[Vector_Y] / 2.0f);
	float cosY = cos(Eulerangles[Vector_Y] / 2.0f);
	float sinZ = sin(Eulerangles[Vector_Z] / 2.0f);
	float cosZ = cos(Eulerangles[Vector_Z] / 2.0f);
	
	norm_quat[Quat_W] = cosX * cosY * cosZ + sinX * sinY * sinZ;
	norm_quat[Quat_X] = sinX * cosY * cosZ + cosX * sinY * sinZ;	
	norm_quat[Quat_Z] = cosX * sinY * cosZ + sinX * cosY * sinZ;
	norm_quat[Quat_Z] = cosX * cosY * sinZ + sinX * sinY * cosZ;
	
	normlizeQuat(norm_quat);
}

/**************************************************************************
 *@brief these next functions are used to tilt compensate mag
 *@{
 **************************************************************************/

/**************************************************************************
 *@brief quaternion conjugate
 **************************************************************************/

static inline void quaternionConjugate(const float *normQuat1, float *normQuat2)
{
	  normQuat2[Quat_W] = normQuat1[Quat_W];
	  normQuat2[Quat_X] = -normQuat1[Quat_X];
	  normQuat2[Quat_Y] = -normQuat1[Quat_Y];
	  normQuat2[Quat_Z] = -normQuat1[Quat_Z];	
}

/***************************************************************************
 *@brief quaternion multiply
 ***************************************************************************/

static void quaternionMultiply(const float *nq_a,const float *nq_b, float *nq_c)
{
	float va[3];
	float vb[3];
	float dotAB;
	float crossAB[3];
	
	va[Vector_X] = nq_a[Quat_X];
	va[Vector_Y] = nq_a[Quat_Y];
	va[Vector_Y] = nq_a[Quat_Z];
	
	vb[Vector_X] = nq_b[Quat_X];
	vb[Vector_Y] = nq_b[Quat_Y];
	vb[Vector_Z] = nq_b[Quat_Z];	
	
	dotAB = va[Vector_X] * vb[Vector_X] + va[Vector_Y] * vb[Vector_Y] + va[Vector_Z] * vb[Vector_Z];
	
	crossAB[Vector_X] = va[Vector_Y] * vb[Vector_Z] - va[Vector_Z] * vb[Vector_Y];
	crossAB[Vector_Y] = va[Vector_Z] * vb[Vector_X] - va[Vector_X] * vb[Vector_Z];
	crossAB[Vector_Z] = va[Vector_X] * vb[Vector_Y] - va[Vector_X] * vb[Vector_Y];
	
	nq_c[Quat_W] = nq_a[Quat_W] * nq_b[Quat_W] - dotAB;
	nq_c[Quat_X] = nq_a[Quat_W] * vb[Vector_X] + nq_b[Quat_W] * va[Vector_X] + crossAB[Vector_X];
	nq_c[Quat_Y] = nq_a[Quat_W] * vb[Vector_Y] + nq_b[Quat_W] * va[Vector_Y] + crossAB[Vector_Y];
	nq_c[Quat_Z] = nq_a[Quat_W] * vb[Vector_Z] + nq_b[Quat_W] * va[Vector_Z] + crossAB[Vector_Z];	

}

/********************************************************************************
 *@}
 ********************************************************************************/

/********************************************************************************
 *@brief enable Magenetometer calibration
 ********************************************************************************/

void useMagcal()
{
	 m_useMagCal = true;
}

/********************************************************************************
 *@brief enable Accleration calibration
 ********************************************************************************/

void useAcclcal()
{
	  m_useAcclCal = true;
}

/*********************************************************************************
 *@brief enable Gyroscope calibration
 *********************************************************************************/

void useGyrocal()
{

 	 m_useGyroCal = true;
}

/*********************************************************************************
 *@brief disable Accleration calibration
 *********************************************************************************/

void disableAcclCal()
{	
	if (!m_useAcclCal)  return;
	m_useAcclCal = false;
	
	m_acclOffset[0] = 0;
	m_acclOffset[1] = 0;
	m_acclOffset[2] = 0;
	
  dmp_set_accel_bias(&m_acclOffset);
}

/*********************************************************************************
 *@brief disable Magnetometer calibration
 *********************************************************************************/

void disableMagCal()
{
	 m_useMagCal = false;
}
/*********************************************************************************
  *@brief mpu9150 initialize function 
  ********************************************************************************/

bool init_mpu9150(int mpuRate, int magRate, int lpf)	
{
	struct int_param_s int_param;
	int result;
	
	
	if ( magRate > 100 ) return false;
	if ( magRate < 1 )   return false;
	
	if ( mpuRate > 1000 ) return false;
	if ( mpuRate < 1 )    return false;
	
	/*get magoffset data when using magnetometer calibration */
	if (m_useMagCal){
		m_magOffset[Mag_X] = (short)(((long) m_CalData.calMagXMax + (long) m_CalData.calMagXMin) / 2);
    m_magOffset[Mag_Y] = (short)(((long) m_CalData.calMagYMax + (long) m_CalData.calMagYMin) / 2);
    m_magOffset[Mag_Z] = (short)(((long) m_CalData.calMagZMax + (long) m_CalData.calMagZMin) / 2);
		
		m_magRange[Mag_X] = m_CalData.calMagXMax - m_magOffset[Mag_X];
		m_magRange[Mag_Y] = m_CalData.calMagYMax - m_magOffset[Mag_Y];
		m_magRange[Mag_Z] = m_CalData.calMagZMax - m_magOffset[Mag_Z];		
	}
	
  /*get Accloffset data when using accleration calibration */
  if (m_useAcclCal)
	{
		m_acclOffset[Accl_X] = ((long) m_CalData.calAcclXMax + (long) m_CalData.calAcclXMin) / 2;
		m_acclOffset[Accl_Y] = ((long) m_CalData.calAcclYMax + (long) m_CalData.calAcclYMin) / 2;
		m_acclOffset[Accl_Z] = ((long) m_CalData.calAcclZMax + (long) m_CalData.calAcclZMin) / 2;
	
		m_acclRange[Accl_X] = m_CalData.calAcclXMax - (short)m_acclOffset[Accl_X];
		m_acclRange[Accl_Y] = m_CalData.calAcclYMax - (short)m_acclOffset[Accl_Y];
		m_acclRange[Accl_Z] = m_CalData.calAcclZMax - (short)m_acclOffset[Accl_Z];
	}
	else
	{
		  m_useMagCal = false;
		  m_useAcclCal = false;
	}
   
	dmp_set_accel_bias(&m_acclOffset);
	
	if (m_useGyroCal)
	{
		dmp_enable_gyro_cal(1);
	}
	
	int_param.GPIOx = GPIOB;
	int_param.GPIO_Pin = GPIO_Pin1;
	int_param.EXTI_Trigger = EXTI_Trigger_Falling;
	if (result != mpu_init(&int_param))
	{
		
#ifdef MPU_DEBUG_PRINT
		printf("init failed! \n");
#endif 
		return false;
	}
	
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
	
	if (mpu_load_motion_driver_firmware())
	{
		
#ifdef MPU_DEBUG_PRINT
		printf("load motion driver failed\n");
#endif
		return false;
	}
	
#ifdef MPU_DEBUG_PRINT
		printf("dmp_set_orientation\n");
#endif	
	 dmp_set_orientation(inv_matrix_2_scalar(gyro_orientation));
	
#ifdef MPU_DEBUG_PRINT
		printf("enable mpu9150 feature\n");
#endif	
	 dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | \
	                    DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | \
	                     DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
	
#ifdef MPU_DEBUG_PRINT
		printf("set dmp fifo rate according to mpuRate\n");
#endif	
	 dmp_set_fifo_rate(mpuRate);
	
#ifdef MPU_DEBUG_PRINT
		printf("set dmp state\n");
#endif		
   if (mpu_set_dmp_state(1) != 0)
	 {
		 
#ifdef MPU_DEBUG_PRINT
		printf("set dmp state failed\n");
#endif			 
		return false;
	 }
	 
#ifdef MPU_DEBUG_PRINT
		printf("set mpu and compass sample rate\n");
#endif		 
	 mpu_set_sample_rate(mpuRate);
   mpu_set_compass_sample_rate(magRate);
	 
	 if (0 != lpf)
	 {
		 mpu_set_lpf(lpf);
	 }
	 
	 return true;
}

/*********************************************************************************
 *@brief mpu9150 read data function 
 *********************************************************************************/

bool getDMPdata()
{
	short intstatus;
	unsigned char sensor;
	unsigned char more;
	unsigned char timestamp;
	
	
	mpu_get_int_status(&intstatus);
	if(intstatus & (MPU_INT_STATUS_DMP | MPU_INV_STATUS_DMP_0) \
		     != (MPU_INT_STATUS_DMP | MPU_INV_STATUS_DMP_0))
    { return false; }
	
	if(dmp_read_fifo(m_rawGyro, m_rawAccl, m_rawQuat, &timestamp, &sensor, &more) != 0)
	  { return false; }
		
	if(mpu_get_compass_reg(m_rawMag, &timestamp) != 0)
	  { return false; }
		
		/*use Magentometer calibration*/
		if (m_useMagCal)
		{
			m_magCal[Mag_Y] = -(short)((((long)m_rawMag[Mag_X] - (long)m_magOffset[Mag_X]) * \
				                           (long)SENSOR_RANGE) / m_magRange[Mag_X]);
			m_magCal[Mag_X] = (short)((((long)m_rawMag[Mag_Y] - (long)m_magOffset[Mag_Y]) * \
				                           (long)SENSOR_RANGE) / m_magRange[Mag_Y]);
      m_magCal[Mag_Z] = (short)((((long)m_rawMag[Mag_Z] - (long)m_magOffset[Mag_Z]) * \
				                           (long)SENSOR_RANGE) / m_magRange[Mag_Z]);
		}
	  else
		{
			m_magCal[Mag_Y] = -m_rawMag[Mag_X];
			m_magCal[Mag_X] = m_rawMag[Mag_Y];
			m_magCal[Mag_Z] = m_rawMag[Mag_Z];
		}
																	 
		m_normQuat[Quat_w] = (float)m_rawQuat[Quat_w];
		m_normQuat[Quat_X] = (float)m_rawQuat[Quat_X];		
		m_normQuat[Quat_Y] = (float)m_rawQuat[Quat_Y];
		m_normQuat[Quat_Z] = (float)m_rawQuat[Quat_Z];
		
		normlizeQuat(m_normQuat);
		Quaternion_2_Euler(m_normQuat,m_Eulerangles);
		
		/*use Accleration calibration*/
		if (m_useAcclCal)
		{
			if ( m_rawAccl[Accl_X] >= 0 )
				m_acclCal[Accl_X] = -(short)((( (long)m_rawAcclp[Accl_X] ) * \
					                            (long)SENSOR_RANGE ) / m_CalData.calAcclXMax );
			else
			  m_acclCal[Accl_X] = -(short)((( (long)m_rawAcclp[Accl_X] ) * \
					                            (long)SENSOR_RANGE ) / -m_CalData.calAcclXMin );

      if ( m_rawAccl[Accl_Y] >=0 )			
				m_acclCal[Accl_Y] = -(short)((( (long)m_rawAcclp[Accl_Y] ) * \
					                            (long)SENSOR_RANGE ) / m_CalData.calAcclYMax );
      else 
				m_acclCal[Accl_Y] = -(short)((( (long)m_rawAcclp[Accl_Y] ) * \
					                            (long)SENSOR_RANGE ) / -m_CalData.calAcclYMin );

      if ( m_rawAccl[Accl_Z] >=0 )			
				m_acclCal[Accl_Z] = -(short)((( (long)m_rawAcclp[Accl_Z] ) * \
					                            (long)SENSOR_RANGE ) / m_CalData.calAcclZMax );
      else 
				m_acclCal[Accl_Z] = -(short)((( (long)m_rawAcclp[Accl_Z]) * \
					                            (long)SENSOR_RANGE ) / -m_CalData.calAcclZMin );
		}
		else
	 {
			m_acclCal[Accl_X] = m_rawAccl[Accl_X];
			m_acclCal[Accl_Y] = m_rawAccl[Accl_Y];			
			m_acclCal[Accl_Z] = m_rawAccl[Accl_Z];	
		}
		
		dataFused();
		
	  return true;
}

/*********************************************************************************
 *@brief magnetometer fuse accleration data 
 *********************************************************************************/

void dataFused()
{
	float qMag[4];
	float deltaDMPYaw, deltaMagYaw;
	float newMagYaw,newYaw;
	float temp[4];
	float unfused[4];
	float unfusedConjugate[4];
	
	m_fusedEulerangles[Vector_X] = m_Eulerangles[Vector_X];
	m_fusedEulerangles[Vector_Y] = m_Eulerangles[Vector_Y];
	m_fusedEulerangles[Vector_Z] = 0;
	
	Euler_2_Quaternion(m_fusedEulerangles,unfused);
	
	deltaDMPYaw = -m_Eulerangles[Vector_Z] + m_dmplastYaw;
	m_dmplastYaw = m_Eulerangles[Vector_Z];
	
	qMag[Quat_W] = 0;
	qMag[Quat_X] = m_magCal[Vector_X];
	qMag[Quat_Y] = m_magCal[Vector_Y];
	qMag[Quat_Z] = m_magCal[Vector_Z];	
	
	
	quaternionConjugate(unfused,unfusedConjugate);                //tilt compenstate
	quaternionMltiply(qMag,unfusedConjugate,temp);
	quaternionMltiply(unfused,temp,qMag);

  newMagYaw = -atan2(qMag[Quat_Y],qMag[Quat_X]);
	
	if ( newMagYaw != newMagYaw )
	{
#ifdef MPU_DEBUG_PRINT
		printf("****nan!\n");
#endif
		return;
	}
	
	if ( newMagYaw < 0 )
		newMagYaw = 2.0f * m_PI + newMagYaw;
	
  	newYaw = m_lastYaw + deltaDMPYaw;
	
	if ( newYaw > ( 2.0f * m_PI ) )
		newYaw = newYaw - 2.0f * m_PI;
	if ( newYaw < 0 )
		newYaw = newYaw + 2.0f * m_PI;
	
	deltaMagYaw = newMagYaw - newYaw;
	
	deltaMagYaw = over2PIAngle(deltaMagYaw);
  deltaMagYaw = less0Angle(deltaMagYaw);
	
	if ( m_magMix > 0 )
	{
		newYaw += deltaMagYaw / m_magMix;
		
		newYaw = over2PIAngle(newYaw);
		newYaw = less0Angle(newYaw);
   }
	 
	 m_lastYaw = newYaw;
	 
	m_fusedEulerangles[Vector_Z] = newYaw;

  Euler_2_Quaternion(m_fusedEulerangles,m_fusedQuaternion); 
}


/*********************************************************************************
 *@}        //mpu_9150.c file end
 *********************************************************************************/

