/** @file  dr16.c  @brief 遥控数据接收与解析  @version  v3.3  2026-11-08 */
#include "dr16.h"
#include "FreeRTOS.h"
#include "code_config.h"
#include "task.h"
#include "usart.h"
#include <string.h>

// DR16 实例定义
dr16_t dr16;
dr16_stats_t dr16_stats;

static uint8_t dr16_buf[DR16_FRAME_LEN] __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t dr16_isr_buf[DR16_FRAME_LEN];
static volatile uint8_t dr16_flag;
static UART_HandleTypeDef *const dr16_huart = DR16_HUART;

// DR16 DMA 启动函数
static HAL_StatusTypeDef Dr16_Dma_Start(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
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

// DR16 DMA 重对齐函数
static void Dr16_Dma_Resync(void)
{
    UART_HandleTypeDef *huart = dr16_huart;
    DMA_HandleTypeDef *hdma   = huart->hdmarx;

    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_DMA_DISABLE(hdma);

    // H7 的 DMA 句柄 Instance 是 void*（可能是 DMA 流也可能是 BDMA 通道），须显式转型；
    // 流的寄存器是 NDTR/M0AR（CNDTR/CMAR 是 BDMA 通道的叫法）
    __HAL_DMA_SET_COUNTER(hdma, DR16_FRAME_LEN);
    ((DMA_Stream_TypeDef *)hdma->Instance)->M0AR = (uint32_t)dr16_buf;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    __HAL_DMA_ENABLE(hdma);
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    dr16_stats.resync_cnt++;
}

// DR16 离线清零函数
// 仅 Dr16_Init / Dr16_Online 超时路径调用；单帧异常不再走这里
static void Dr16_Clear(void)
{
    dr16.ch1 = dr16.ch2 = dr16.ch3 = dr16.ch4 = dr16.ch5 = 0;
    dr16.sw_left = dr16.sw_right = 0;
    dr16.mouse_x = dr16.mouse_y = dr16.mouse_z = 0;
    dr16.mouse_l = dr16.mouse_r = 0;
    dr16.key_code               = 0;
    dr16.online                 = false;
    dr16.last_rx_tick           = 0;
}

// DR16 通道校验函数
static bool Dr16_Frame_Check(const dr16_t *r)
{
    if (r->ch1 < -DR16_CH_LIMIT || r->ch1 > DR16_CH_LIMIT)
        return false;
    if (r->ch2 < -DR16_CH_LIMIT || r->ch2 > DR16_CH_LIMIT)
        return false;
    if (r->ch3 < -DR16_CH_LIMIT || r->ch3 > DR16_CH_LIMIT)
        return false;
    if (r->ch4 < -DR16_CH_LIMIT || r->ch4 > DR16_CH_LIMIT)
        return false;
    if (r->ch5 < -DR16_CH_LIMIT || r->ch5 > DR16_CH_LIMIT)
        return false;
    if (r->sw_left < 1 || r->sw_left > 3)
        return false;
    if (r->sw_right < 1 || r->sw_right > 3)
        return false;
    if (r->mouse_l > 1 || r->mouse_r > 1)
        return false;
    return true;
}

// DR16 帧解析函数
static void Dr16_Parse(const uint8_t *buf)
{
    // 先在任务局部结构解析全部通道 校验通过才整帧提交
    dr16_t parsed;
    parsed.ch1      = (int16_t)(((buf[0] | (buf[1] << 8)) & 0x07FF) - 1024);
    parsed.ch2      = (int16_t)((((buf[1] >> 3) | (buf[2] << 5)) & 0x07FF) - 1024);
    parsed.ch3      = (int16_t)((((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07FF) - 1024);
    parsed.ch4      = (int16_t)((((buf[4] >> 1) | (buf[5] << 7)) & 0x07FF) - 1024);
    parsed.sw_left  = (uint8_t)(((buf[5] >> 4) & 0x000C) >> 2);
    parsed.sw_right = (uint8_t)((buf[5] >> 4) & 0x0003);
    parsed.mouse_x  = (int16_t)(buf[6] | (buf[7] << 8));
    parsed.mouse_y  = (int16_t)(buf[8] | (buf[9] << 8));
    parsed.mouse_z  = (int16_t)(buf[10] | (buf[11] << 8));
    parsed.mouse_l  = buf[12];
    parsed.mouse_r  = buf[13];
    parsed.key_code = (uint16_t)(buf[14] | (buf[15] << 8));
    parsed.ch5      = (int16_t)((((uint16_t)buf[16] | ((uint16_t)buf[17] << 8)) & 0x07FF) - 1024);

    // 单帧毛刺丢帧保旧值 连续离线仍由超时判据处理
    if (!Dr16_Frame_Check(&parsed))
    {

        dr16_stats.err_cnt++;
        return;
    }

    parsed.online       = true;
    parsed.last_rx_tick = HAL_GetTick();
    taskENTER_CRITICAL();

    // 短临界区发布遥控快照 FSM 与映射工具只读完整帧
    dr16 = parsed;
    taskEXIT_CRITICAL();
    dr16_stats.frame_cnt++;
}

// DR16 初始化函数
void Dr16_Init(void)
{
    Dr16_Clear();
    __HAL_UART_ENABLE_IT(dr16_huart, UART_IT_IDLE);
    Dr16_Dma_Start(dr16_huart, dr16_buf, DR16_FRAME_LEN);
}

// DR16 串口 IDLE 中断处理函数（ISR 上下文）
void Dr16_Idle_Isr(UART_HandleTypeDef *huart)
{
    if (huart == dr16_huart)
        dr16_stats.irq_cnt++;
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);
        dr16_stats.pe_cnt++;
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
        dr16_stats.fe_cnt++;
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);
        dr16_stats.ne_cnt++;
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        dr16_stats.ore_cnt++;
    }
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (!__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
        return;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);

    if (huart != dr16_huart)
        return;
    dr16_stats.idle_cnt++;

    // 只接受完整 DMA 帧 错位时重对齐 不在 ISR 解码通道
    uint16_t cndtr = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (cndtr != 0 && cndtr != huart->RxXferSize)
    {

        dr16_stats.err_cnt++;
        Dr16_Dma_Resync();
        return;
    }
    // 中断仅保存字节快照并置标志 通道解析交给估计任务
    memcpy(dr16_isr_buf, dr16_buf, DR16_FRAME_LEN);
    dr16_flag = 1u;
}

// 任务侧解析完整遥控帧
void Dr16_Update(void)
{
    uint8_t data[DR16_FRAME_LEN];
    taskENTER_CRITICAL();
    if (!dr16_flag)
    {
        taskEXIT_CRITICAL();
        return;
    }
    memcpy(data, dr16_isr_buf, sizeof(data));
    dr16_flag = 0u;
    taskEXIT_CRITICAL();
    Dr16_Parse(data);
}

// DR16 通道死区处理函数
int16_t Dr16_Deadline(int16_t input, uint16_t deadline)
{
    if (input > (int16_t)deadline || input < -(int16_t)deadline)
        return input;
    return 0;
}

// DR16 整帧快照函数
dr16_t Dr16_Snapshot(void)
{
    dr16_t s;

    taskENTER_CRITICAL();
    s = dr16;
    taskEXIT_CRITICAL();

    return s;
}

// DR16 在线检测函数
bool Dr16_Online(void)
{
    if (!dr16.online)
        return false;

    if (HAL_GetTick() - dr16.last_rx_tick > DR16_OFFLINE_MS)
    {
        HAL_NVIC_DisableIRQ(UART9_IRQn);
        Dr16_Clear();
        Dr16_Dma_Resync();
        HAL_NVIC_EnableIRQ(UART9_IRQn);
        return false;
    }
    return true;
}
