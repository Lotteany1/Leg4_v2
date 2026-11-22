// 腿部状态估计头文件
// 版本：v1.2  日期：2026-10-02
#ifndef LEG_H
#define LEG_H

typedef struct
{
    float pitch_1,   pitch_4;               // rad,   直读 DM 编码器 + 安装偏置
    float d_pitch_1, d_pitch_4;             // rad/s, 直读 DM 反馈角速度

    float len, ang;                         // 五连杆极坐标：腿长(m), 腿角(rad)
    float ang_last;                         // 上拍腿角 本拍解缠的接续基准

    float d_len, d_ang;                     // 腿长/腿角一阶导 m/s, rad/s
} leg_state_t;

extern leg_state_t leg_lft, leg_rgt;

void Leg_State_Update(void);

#endif /* LEG_H */
