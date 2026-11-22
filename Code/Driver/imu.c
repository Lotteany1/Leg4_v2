/** @file  imu.c  @brief IMU 数据接收与解析  @version  v3.2  2026-10-26 */
#include "imu.h"
#include "FreeRTOS.h"
#include "code_config.h"
#include "polarity_config.h"
#include "task.h"
#include <math.h>
#include <string.h>

// IMU 协议参数宏定义区
#define IMU_CRC_POLY    0x1021U
#define IMU_LEN_91_DATA (sizeof(imu91_t))

// IMU 运行时状态
static uint8_t imu_dma_buf[IMU_FRAME_LEN] __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t imu_isr_buf[IMU_FRAME_LEN];
static volatile uint8_t imu_flag;
static imu91_t imu_data;
imu_t imu;

// IMU 句柄配置
static UART_HandleTypeDef *const imu_huart = IMU_HUART;

// IMU DMA 启动函数
static HAL_StatusTypeDef Imu_Dma_Start(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{

    if (huart->RxState != HAL_UART_STATE_READY)
        return HAL_BUSY;
    if (data == NULL || size == 0)
        return HAL_ERROR;

    __HAL_LOCK(huart);
    huart->pRxBuffPtr                = data;
    huart->RxXferSize                = size;
    huart->ErrorCode                 = HAL_UART_ERROR_NONE;
    huart->RxState                   = HAL_UART_STATE_BUSY_RX;
    huart->hdmarx->XferAbortCallback = NULL;

    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)data, size);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UNLOCK(huart);

    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);
    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    return HAL_OK;
}

// IMU DMA 重对齐函数
static void Imu_Dma_Resync(void)
{

    // 复位 DMA channel，让下一帧从 buffer 头对齐
    // 任务侧调用前需先 disable UART7_IRQ；ISR 内调用天然安全
    UART_HandleTypeDef *huart = imu_huart;
    DMA_HandleTypeDef *hdma   = huart->hdmarx;

    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_DMA_DISABLE(hdma);

    // H7 的 DMA 句柄 Instance 是 void*（可能是 DMA 流也可能是 BDMA 通道），须显式转型；
    // 流的寄存器是 NDTR/M0AR（CNDTR/CMAR 是 BDMA 通道的叫法）
    __HAL_DMA_SET_COUNTER(hdma, IMU_FRAME_LEN);
    ((DMA_Stream_TypeDef *)hdma->Instance)->M0AR = (uint32_t)imu_dma_buf;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma) | __HAL_DMA_GET_HT_FLAG_INDEX(hdma) |
                                   __HAL_DMA_GET_TE_FLAG_INDEX(hdma) | __HAL_DMA_GET_DME_FLAG_INDEX(hdma) | __HAL_DMA_GET_FE_FLAG_INDEX(hdma));
    __HAL_DMA_ENABLE(hdma);
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
}

// IMU CRC16 校验计算函数
static uint16_t Imu_Crc16_Calc(uint16_t crc, const uint8_t *data, uint16_t len)
{
    while (len--)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
        {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ IMU_CRC_POLY)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// IMU 标定应用函数
static void Imu_Apply_Calib(void)
{
    imu.eul[IMU_EUL_ROLL]  = imu_data.eul[IMU_EUL_ROLL] * IMU_ROLL_POLARITY;
    imu.eul[IMU_EUL_PITCH] = imu_data.eul[IMU_EUL_PITCH] * IMU_PITCH_POLARITY;
    imu.eul[IMU_EUL_YAW]   = imu_data.eul[IMU_EUL_YAW] * IMU_YAW_POLARITY;

    imu.gyr[IMU_GYR_PITCH] = imu_data.gyr[IMU_GYR_PITCH] * IMU_PITCH_POLARITY;
    imu.gyr[IMU_GYR_ROLL]  = imu_data.gyr[IMU_GYR_ROLL] * IMU_ROLL_POLARITY;
    imu.gyr[IMU_GYR_YAW]   = imu_data.gyr[IMU_GYR_YAW] * IMU_YAW_POLARITY;

    imu.acc[IMU_ACC_X] = imu_data.acc[IMU_ACC_X] * IMU_ACC_X_POLARITY;
    imu.acc[IMU_ACC_Y] = imu_data.acc[IMU_ACC_Y] * IMU_ACC_Y_POLARITY;
    imu.acc[IMU_ACC_Z] = imu_data.acc[IMU_ACC_Z] * IMU_ACC_Z_POLARITY;

    imu.quat[IMU_QUAT_W] = imu_data.quat[IMU_QUAT_W] * IMU_QUAT_W_POLARITY;
    imu.quat[IMU_QUAT_X] = imu_data.quat[IMU_QUAT_X] * IMU_QUAT_X_POLARITY;
    imu.quat[IMU_QUAT_Y] = imu_data.quat[IMU_QUAT_Y] * IMU_QUAT_Y_POLARITY;
    imu.quat[IMU_QUAT_Z] = imu_data.quat[IMU_QUAT_Z] * IMU_QUAT_Z_POLARITY;
}

// IMU 初始化函数
void Imu_Init(void)
{
    __HAL_UART_ENABLE_IT(imu_huart, UART_IT_IDLE);
    Imu_Dma_Start(imu_huart, imu_dma_buf, IMU_FRAME_LEN);
}

// IMU 解析函数
bool Imu_Parse(void)
{
    if (!imu_flag)
        return false;

    // 关中断快照，避免 ISR 中途改写造成数据撕裂
    uint8_t imu_parse_buf[IMU_FRAME_LEN];
    taskENTER_CRITICAL();
    memcpy(imu_parse_buf, imu_isr_buf, IMU_FRAME_LEN);
    imu_flag = 0;
    taskEXIT_CRITICAL();

    if (imu_parse_buf[0] != 0x5A || imu_parse_buf[1] != 0xA5)
        return false;

    uint16_t data_len = (imu_parse_buf[3] << 8) + imu_parse_buf[2];
    if (data_len != IMU_LEN_91_DATA)
        return false;

    // 帧头与长度通过后在任务侧校验 CRC 中断不跑校验和浮点解析
    uint16_t crc = Imu_Crc16_Calc(0, imu_parse_buf, 4);
    crc          = Imu_Crc16_Calc(crc, &imu_parse_buf[6], IMU_LEN_91_DATA);

    uint16_t received_crc = (imu_parse_buf[5] << 8) + imu_parse_buf[4];
    if (crc != received_crc)
        return false;

    // 载荷解析与有限性检查通过后才更新标定状态和在线时间
    memcpy(&imu_data, &imu_parse_buf[6], IMU_LEN_91_DATA);
    for (uint32_t i = 0u; i < 3u; i++)
        if (!isfinite(imu_data.eul[i]) || !isfinite(imu_data.gyr[i]) || !isfinite(imu_data.acc[i]))
            return false;
    for (uint32_t i = 0u; i < 4u; i++)
        if (!isfinite(imu_data.quat[i]))
            return false;
    Imu_Apply_Calib();
    imu.online       = true;
    imu.last_rx_tick = HAL_GetTick();
    return true;
}

// IMU IDLE 中断处理函数
void Imu_Idle_Isr(UART_HandleTypeDef *huart)
{

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
        __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (!__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
        return;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);

    if (huart != imu_huart)
        return;

    // IDLE 时只接受完整帧 部分帧错位重对齐 不清连续姿态
    uint16_t cndtr = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (cndtr != 0 && cndtr != huart->RxXferSize)
    {

        Imu_Dma_Resync();
        return;
    }

    // 保存接收快照后重装普通 DMA 下一帧仍从缓冲头开始
    memcpy(imu_isr_buf, imu_dma_buf, IMU_FRAME_LEN);
    imu_flag = 1;
    Imu_Dma_Resync();
}

// IMU 在线监控函数
bool Imu_Online(void)
{

    // 接收超时后连续多拍确认离线 计数按控制任务调用推进
    static uint8_t offline_ticks = 0;

    if (imu.online && HAL_GetTick() - imu.last_rx_tick > IMU_OFFLINE_MS)
    {
        offline_ticks++;
        if (offline_ticks >= IMU_OFFLINE_CNT)
        {
            HAL_NVIC_DisableIRQ(UART7_IRQn);
            imu.online = false;
            Imu_Dma_Resync();
            HAL_NVIC_EnableIRQ(UART7_IRQn);
            offline_ticks = 0;
        }
    }
    else
    {
        offline_ticks = 0;
    }

    return imu.online;
}
