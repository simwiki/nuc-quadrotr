#include "PID.h"
#include "math.h"
struct PID_OUT pid_out;	//外环PID
struct PID_IN  pid_in;	//内环pid
void Back_To_Middle(void)
{
	expect_X=0.0;expect_Y=-1.0;expect_Z=0.0;
}

void PID_Init(void)
{
	Back_To_Middle();
	//x,y,z三轴外环PID参数
	pid_out.Kp_x_out = 0.0;	pid_out.Kp_y_out = 0.0; pid_out.Kp_z_out = 0.0;
	pid_out.Ki_x_out = 0.0;	pid_out.Ki_y_out = 0.0; pid_out.Ki_z_out = 0.0;
	pid_out.Kd_x_out = 0.0;	pid_out.Kd_y_out = 0.0; pid_out.Kd_z_out = 0.0;
	//x,y,z三轴内环PID参数 
	pid_in.Kp_x_in   = 0.0; pid_in.Kp_y_in = 0.0; pid_in.Kp_z_in = 0.0;
	pid_in.Ki_x_in   = 0.0; pid_in.Ki_y_in = 0.0; pid_in.Ki_z_in = 0.0;
	pid_in.Kd_x_in   = 0.0; pid_in.Kd_y_in = 0.0; pid_in.Kd_z_in = 0.0;
}
/*****************************************************************************
外环角度PID控制:
		x轴对应俯仰角pitch，y轴对应横滚角roll，z轴对应航向角yaw
		输出三轴对应的经PID控制之后的角度
内环角速度PID控制:
		x轴对应俯仰角角速度，y轴对应横滚角角速度，z轴对应航向角角速度
		输出三周对应的经PID控制之后的角速度，映射为PWM输出，从而控制电机转速
******************************************************************************/
//采用位置式PID，积分采用梯形积分
//x,y,z: 输入的实际角度值,从MPU相关寄存器获得
//gx,gy,gz：输入的给定（期望）值，人为根据情况设定
void PID_Control(float x,float y,float z,float gx,float gy,float gz)
{
	//外环控制x
	AX = x; AY = y; AZ = z;		//输入实际值
	err_ox = expect_X - AX;
	if( fabs(err_ox) > 0.05 )
		flag_x_out = 0;			//积分分离，如果误差过大，取消积分调控
	else						//这里为两毫秒变换 0.05度 = 每秒25度
		flag_x_out = 1; 		
	xi_out += err_ox;
	if(xi_out > 20) {xi_out = 20;}		//积分限幅
	if(xi_out < -20) {xi_out = -20;}
	Out_x_out = pid_out.Kp_x_out * err_ox + pid_out.Ki_x_out * flag_x_out * xi_out/2 + pid_out.Kd_x_out * (err_ox - err_last_x_out);
	err_last_x_out = err_ox;
	//外环控制y
	err_oy = expect_Y - AY;
	if( fabs(err_oy) > 0.05)	
		flag_y_out = 0;			//积分分离，如果误差过大，取消积分调控
	else						//这里为两毫秒变换 0.05度 = 每秒25度
		flag_y_out = 1;
	yi_out += err_oy;
	if(yi_out > 20) {yi_out = 20;}		//积分限幅
	if(yi_out < -20) {yi_out = -20;}
	Out_y_out = pid_out.Kp_y_out * err_oy + pid_out.Ki_y_out * flag_y_out * yi_out/2 + pid_out.Kd_y_out * (err_oy - err_last_y_out);
	err_last_y_out = err_oy;
	//外环控制z
	err_oz = expect_Z - AZ;
	if( fabs(err_oz) > 0.05)	
		flag_z_out = 0;			//积分分离，如果误差过大，取消积分调控
	else						//这里为两毫秒变换 0.05度 = 每秒25度
		flag_z_out = 1;
	zi_out += err_oz;
	if(zi_out > 20) {zi_out = 20;}		//积分限幅
	if(zi_out < -20) {zi_out = -20;}
	Out_z_out = pid_out.Kp_z_out * err_oz + pid_out.Ki_z_out * flag_z_out * zi_out/2 + pid_out.Kd_z_out * (err_oz - err_last_z_out);
	err_last_z_out = err_oz;
	//内环控制x
	err_ix = Out_x_out - gx;
	xi_in += err_ix;
	if( fabs(err_ix) > 0.05)	
		flag_x_in = 0;			//积分分离，如果误差过大，取消积分调控
	else						//这里为两毫秒变换 0.05度 = 每秒25度
		flag_x_in = 1;
	if(xi_in > 30) {xi_in = 30;}
	if(xi_in < -30) {xi_in = -30;}		//积分限幅
	Out_x_in = pid_in.Kp_x_in * err_ix + pid_in.Kp_x_in * flag_x_in * xi_in/2 + pid_in.Kd_x_in * (err_ix - err_last_x_in);
	if(Out_x_in > 600) {Out_x_in = 600;}	
	if(Out_x_in < -600) {Out_x_in = -600;}			//限制调节PWM的值
	err_last_x_in = err_ix;
	//内环控制y
	err_iy = Out_y_out - gy;
	yi_in += err_iy;
	if( fabs(err_iy) > 0.05)	
		flag_y_in = 0;			//积分分离，如果误差过大，取消积分调控
	else						//这里为两毫秒变换 0.05度 = 每秒25度
		flag_y_in = 1;
	if(yi_in > 30) {yi_in = 30;}
	if(yi_in < -30) {yi_in = -30;}		//积分限幅
	Out_y_in = pid_in.Kp_y_in * err_iy + pid_in.Kp_y_in * flag_y_in * yi_in/2 + pid_in.Kd_y_in * (err_iy - err_last_y_in);
	if(Out_y_in > 600) {Out_y_in = 600;}
	if(Out_y_in < -600) {Out_y_in = -600;}			//限制调节PWM的值
	err_last_y_in = err_iy;
	//内环控制z
	err_iz = Out_z_out - gz;
	zi_in += err_iz;
	if( fabs(err_iz) > 0.05)	
		flag_z_in = 0;			//积分分离，如果误差过大，取消积分调控
	else						//这里为两毫秒变换 0.05度 = 每秒25度
		flag_z_in = 1;
	if(zi_in > 30) {zi_in = 30;}
	if(zi_in < -30) {zi_in = -30;}		//积分限幅
	Out_z_in = pid_in.Kp_z_in * err_iz + pid_in.Kp_z_in * flag_z_in * zi_in/2 + pid_in.Kd_z_in * (err_iz - err_last_z_in);
	if(Out_z_in > 600) {Out_z_in = 600;}
	if(Out_z_in < -600) {Out_z_in = -600;}			//限制调节PWM的值
	err_last_z_in = err_iz;
	
	//PID_PWM(Out_x_in,Out_y_in,Out_z_in);
}

