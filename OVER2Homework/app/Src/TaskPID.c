#include "TaskPID.h"


//Part1 pid
uint8_t PID_Parameter_Init(pid_type * pid, PID_MODE mode, float Pidpara[], uint32_t MaxOut, uint32_t MaxIout, uint16_t DeadZone)
{
    if (pid == NULL || Pidpara == NULL)
    {
        return 0;
    }

    pid->pidmode = mode;

    pid->kp = Pidpara[0];
    pid->ki = Pidpara[1];
    pid->kd = Pidpara[2];

    pid->MaxOut = MaxOut;

    pid->MaxIout = MaxIout;

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->sum_error = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->Out = 0.0f;

    pid->DeadZone = DeadZone;
    return 1;
}

/**
 * @brief  pid参数更新
 * @note	 用于分段pid参数更新
 * @param  PID: PID结构体指针
 * @retval 1：正常；0：错误
 */
uint8_t PID_Parameter_Update(pid_type * pid, float Pidpara[])
{
    pid->kp = Pidpara[0];
    pid->ki = Pidpara[1];
    pid->kd = Pidpara[2];
    return 1;
}

/**
 * @brief  PID计算函数
 * @note
 * @param  *PID: PID结构体指针
 * @param  dst: 目标值
 * @param  real: 实际值
 * @retval 计算得出的输出值
 */

float Pid_cal(pid_type *PID, float dst, float real)
{
    if (PID == NULL)
    {
        return 0.0f;
    }

    PID->error[2] = PID->error[1];
    PID->error[1] = PID->error[0];
    PID->dst = dst;
    PID->real = real;
    PID->error[0] = dst - real;

    if (PID->pidmode == PID_POSITION)
    {
        PID->Pout = PID->kp * PID->error[0];

        PID->sum_error += PID->error[0];
        PID->Iout = PID->ki * PID->sum_error;
		
        PID->Dbuf[2] = PID->Dbuf[1];
        PID->Dbuf[1] = PID->Dbuf[0];
        PID->Dbuf[0] = (PID->error[0] - PID->error[1]);
        PID->Dout = PID->kd * PID->Dbuf[0];
        LIMIT(PID->Iout, (-1 * PID->MaxIout), PID->MaxIout);
        PID->Out = PID->Pout + PID->Iout + PID->Dout;
        LIMIT(PID->Out, (-1 * PID->MaxOut), PID->MaxOut);
    }
    else if (PID->pidmode == PID_DELTA)
    {
        PID->Pout = PID->kp * (PID->error[0] - PID->error[1]);
        PID->Iout = PID->ki * PID->error[0];
        PID->Dbuf[2] = PID->Dbuf[1];
        PID->Dbuf[1] = PID->Dbuf[0];
        PID->Dbuf[0] = (PID->error[0] - 2.0f * PID->error[1] + PID->error[2]);
        PID->Dout = PID->kd * PID->Dbuf[0];
        PID->Out += PID->Pout + PID->Iout + PID->Dout;
        LIMIT(PID->Out, (-1 * PID->MaxOut), PID->MaxOut);
    }

    return PID->Out;
}

/**
 * @brief  PID清零
 * @note   误差输出清零，防止重启时电机异常输出
 * @param  *pid: 要清的pid
 * @retval 1：正常；0：错误
 */
uint8_t PID_clear(pid_type *pid)
{
    if (pid == NULL)
    {
        return 0;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->sum_error = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->Out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    return 1;
}
