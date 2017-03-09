#ifndef _PID_H
#define _PID_H
//外环PID参数
struct PID_OUT
{	
	float Kp_x_out;
	float Kp_y_out;
	float Kp_z_out;
	
	float Ki_x_out;
	float Ki_y_out;
	float Ki_z_out;
	
	float Kd_x_out;
	float Kd_y_out;
	float Kd_z_out;
};
//内环PID参数
struct PID_IN
{	
	float Kp_x_in;
	float Kp_y_in;
	float Kp_z_in;
	
	float Ki_x_in;
	float Ki_y_in;
	float Ki_z_in;
	
	float Kd_x_in;
	float Kd_y_in;
	float Kd_z_in;	
};
float expect_X,expect_Y,expect_Z;								//定义角度设定（期望）值
float AX,AY,AZ;													//定义角度实际值,A代表actual-实际
float err_ox,err_oy,err_oz;										//定义外环差值,o代表outside-外环
float err_ix,err_iy,err_iz;										//定义内环差值,i代表in-内环
float err_last_x_out,err_last_y_out,err_last_z_out;				//定义外环上一次的偏差值
float err_last_x_in,err_last_y_in,err_last_z_in;				//定义内环上一次的偏差值

float xi_out,yi_out,zi_out;										//定义外环积分，i-代表积分-integral，out-代表外环
float xi_in,yi_in,zi_in;										//定义内环积分，i-代表积分-integral，in-代表内环

//设置积分分离标志位，使系统调节更稳定	
char flag_x_out = 1,flag_y_out = 1,flag_z_out = 1;				//for 外环
char flag_x_in  = 1,flag_y_in =  1,flag_z_in = 1;				//for 内环

//定义内外环输出
float Out_x_out,Out_y_out,Out_z_out;							//输出-x|y|z-外环
float Out_x_in,Out_y_in,Out_z_in;								//输出-x|y|z-内环

void Back_To_Middle(void);										//回中平衡设定值
void PID_Init(void);											//初始化PID参数
void PID_Control(float x,float y,float z,float gx,float gy,float gz);	//内外环PID控制

#endif
