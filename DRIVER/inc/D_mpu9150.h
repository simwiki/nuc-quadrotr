
/***
**@defgroup The defination of DMP use for mpu9150
**@{
***/
	
/**
 *@brief the status of print data
 *@{
 */	
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
/**@}
  */
	
/**@brief  the status of mpu9150
  *@{
  */
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
/**@}
 */
 
#define MOTION          (0)
#define NO_MOTION       (1)

/**@brief Starting sampling rate. 
 *@{
 */
#define DEFAULT_MPU_HZ  (100)
/**@}
  */
	
/**@brief defined mpu flash size
  *@{
	*/
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
/**@}
   */

/**@brief quat tansfer coeffeciency
  *@{
  */
#define		q30	1073741824.0f
/**@}
  */

/***
**@}
***/

/***
**@defgroup The declaration of data type 
**@{
***/

/**@brief orient  Gyro and accel orientation in body frame
  *@{
  */
const signed char gyro_orientation[9] = {-1, 0, 0,
                                          0,-1, 0,
                                          0, 0, 1,
                                        };
/**@}
  */

/**@brief 
	*
	*/




#endif  /*_D_MPU9150_H*/