/** @file  pid.c  @brief PID 控制器  @version  v3.0  2026-10-26 */
#include "pid.h"
#include "code_config.h"
#include <math.h>

// PID 参数初始化
void Pid_Init(pid_t *pid, float kp, float ki, float kd, float out_max, float out_min)
{
    *pid = (pid_t){
        .kp      = kp,
        .ki      = ki,
        .kd      = kd,
        .out_max = out_max,
        .out_min = out_min,
    };
}

// PID 单拍更新
void Pid_Update(pid_t *pid, float tar, float meas)
{
    // NaN 入口闸：坏值一旦进 err_sum 会被 0*NaN=NaN 永久锁存，ki=0 也救不回
    if (!isfinite(tar) || !isfinite(meas))
        return;

    pid->tar  = tar;
    pid->meas = meas;
    pid->err  = pid->tar - pid->meas;

    pid->p_out = pid->kp * pid->err;
    pid->i_out = pid->ki * pid->err_sum * CTRL_DT;

    // 旧 kd 按 1 ms 离散误差差值标定 用当前步长归一化微分
    pid->d_out = pid->kd * (pid->err - pid->err_last) * (0.001f / CTRL_DT);

    pid->out = pid->p_out + pid->i_out + pid->d_out;

    if (pid->out > pid->out_max)
    {
        pid->out = pid->out_max;
    }
    else if (pid->out < pid->out_min)
    {
        pid->out = pid->out_min;
    }
    else
    {
        // 先算后累：输出被限幅的当拍冻结积分，防饱和恢复长拖尾
        pid->err_sum += pid->err;
    }

    pid->err_last = pid->err;
}
