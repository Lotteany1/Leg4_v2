/** @file  vofa.h  @brief VOFA+ 串口绑定与浮点发送接口  @version  v1.1  2026-11-02 */
#ifndef VOFA_H
#define VOFA_H

#include "usart.h"

#define VOFA_HUART (&hlpuart1) // CubeMX LPUART1，209700 baud、8N1；使用 BDMA 发送

// 首项与变参均按 double 传递，缓冲转为 float；末尾 INFINITY 收尾，每帧最多 64 路。
void Vofa_Send_Float(double first, ...);

#endif /* VOFA_H */
