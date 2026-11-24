/**
 * @file  air.h
 * @brief 腿端支持力与失重、压缩冲击辅助判定接口
 * @version  v1.9  2026-11-24
 */
#ifndef AIR_H
#define AIR_H

#include <stdint.h>
#include <stdbool.h>
#include "code_config.h"

#define AIR_LAND_RELIFT_MS       10u  // ms 两腿支持力都低于离地阈值满此时间，返回 AIR

// 离地检测
#define AIR_FN_TH_LOW      20.0f // N 离地阈，每腿支撑力
#define AIR_FN_TH_HIGH     80.0f // N 落地阈，两腿平均支撑力
#define AIR_DEBOUNCE_MS    100u    // ms  连续低于阈判离地
#define AIR_LAND_MS        1u    // ms  连续高于阈判落地
#define AIR_REARM_MS       500u  // ms  落地后重新武装延迟 防触地反弹判成二次离地
#define AIR_STUCK_MS       0u    // ms  离地持续超时强制落地 假触发无触地尖峰永远出不来 置 0 关闭
#define AIR_EXIT_FREEZE_MS 1000u // ms  非平衡态退出后恢复延迟 站起瞬间振荡期冻结
#define AIR_LEN_MIN        0.05f // m   除法腿长下限 防坏值打穿
#define AIR_LPF_D2         0.75f // 二阶导低通 编码器微分噪声大
#define AIR_LPF_FN         0.99f // 支持力低通

// 单腿判定支持力在最终低通前的三个竖直分量，单位 N。
typedef struct
{
    float motor_z;   // 电机反馈轴力在竖直方向的投影
    float spring_z;  // 被动气弹簧轴力在竖直方向的投影
    float inertia_z; // 轮端质量乘以重力加速度与轮端竖直加速度之和
} air_force_terms_t;

typedef struct
{
    float fn_lft, fn_rgt;         // N 含 Tp 项 观测通道
    float fn_det_lft, fn_det_rgt; // N 剔除 Tp 项 判定通道
    float fn_avg;                 // N 两腿判定通道均值，供落地确认
    uint8_t flag;                 // 1=离地
    uint8_t slip_inhibited;       // 1=本拍已抑制新增离地，已有 AIR/冻结/关闭时为 0
    uint8_t contact_mask;         // 位 0/1 左/右腿本拍有效且支持力超过落地阈
    uint8_t side_valid_mask;      // 位 0/1，分侧力估计和反馈时标有效
    uint8_t side_contact_mask;    // 位 0/1，持续确认的接触
    uint8_t side_unloaded_mask;   // 位 0/1，持续确认的卸载
    bool support_valid;           // 本拍支持力估计有效并执行了 AIR 判定
    bool freefall_confirmed;      // 本轮 AIR 已确认持续失重
    bool last_impact_landing;     // 最近一次触地由 IMU 冲击与压缩辅助确认
    uint16_t unload_ms;           // ms，正常或打滑分支实际使用的离地计时
    uint32_t land_high_samples;   // AIR 中均值超过落地阈值的累计估计拍数，保留遥测间隔内的短峰
    uint32_t land_events;         // 达到确认条件的累计落地次数
    air_force_terms_t terms_lft, terms_rgt; // 当前有效估计拍的左右分量，冻结时保持
} air_status_t;

//! 离地遥测实例
extern volatile air_status_t air_status;

void Air_Init(void);   // 初始化滤波与微分播种 init 任务调用 漏调则首拍自补
void Air_Update(void); // 估计任务 1 ms 每拍 排在 Body_State_Update 之后
bool Air_Update_Support(void); // 发布支持力和分侧接触，返回本拍是否有效
void Air_Update_Detect(void);  // 滑移请求更新后，提交本拍 AIR 电平
uint8_t Air_Get(void); // 离地标志只读 1=离地 受 BALANCE_USE_AIR_CHECK 门控
void Air_Set_Landing(uint8_t active); // 控制拍设置下一估计拍的 LAND 重武装策略
void Air_Set_Step(uint8_t side); // 位 0/1 动作侧，控制拍交接下一估计拍

// 估计任务登记本拍打滑抑制请求，双腿持续卸载和已有 AIR 优先。
void Air_Set_Slip_Inhibit(uint8_t inhibit);

#endif /* AIR_H */
