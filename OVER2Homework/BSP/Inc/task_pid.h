#ifndef __TASK_
#define __TASK_
//Part1 pid
#include "sysconfig.h"

/**Enum Definition**/
typedef enum
{
    PID_POSITION = 0,
    PID_DELTA
} PID_MODE;


/**Struct Definition**/
typedef struct
{
    /* data */
	PID_MODE pidmode;

	float error[3]; //error[0]代表最近一次误差  error[1]代表上一次误差 error[2]代表上上次误差
	float sum_error;
	float Dbuf[3];  //微分项 0最新 1上一次 2上上次

	float kp;
	float ki;
	float kd; // PID系数

	float Pout;
	float Iout;
	float Dout; //比例 积分 微分项输出
	float Out; //最终输出

	float dst;  //目标值
	float real; //实际值

	float MaxOut; //PID最大输出(0为不限制)
	float MaxIout; //积分项最大输出(0为不限制)
	
	float MaxBlockIount;//堵转检测时用于判别的积分项最大输出
	
	uint16_t DeadZone; //死区大小

} pid_type;


/**Function Delaration**/
uint8_t PID_Parameter_Init(pid_type *PID, PID_MODE mode, float Pidpara[], uint32_t MaxOut, uint32_t MaxIout, uint16_t DeadZone);
uint8_t PID_Parameter_Update(pid_type * pid, float Pidpara[]);
uint8_t PID_clear(pid_type *pid);
float Pid_cal(pid_type *PID, float dst, float real);




#endif
