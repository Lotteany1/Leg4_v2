/** @file  led.c  @brief 板载指示灯电平控制与使用权仲裁  @version  v1.1  2026-11-22 */
#include "led.h"
#include "cmsis_os2.h"

/* ============================================================================
 * LED 配置表（唯一编辑点，由 led.h 的 LED_LIST X-macro 展开）
 * ==========================================================================*/
static const led_config_t led_configs[LED_COUNT] =
{
#define X(name, port, pin, active) { .GPIO_Port = port, .GPIO_Pin = pin, .active_state = active },
    LED_LIST
#undef X
};

/* ============================================================================
 * 运行时状态
 * ==========================================================================*/
static osMutexId_t led_mutex = NULL;

/* ============================================================================
 * 内部工具函数
 * ==========================================================================*/
static GPIO_PinState Led_Get_Inactive_State(GPIO_PinState active_state)
{
    return (active_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static void Led_Write_State(const led_config_t *config, GPIO_PinState state)
{
    HAL_GPIO_WritePin(config->GPIO_Port, config->GPIO_Pin,
                      (state == GPIO_PIN_SET) ? config->active_state
                                              : Led_Get_Inactive_State(config->active_state));
}

/* ============================================================================
 * 对外接口
 * ==========================================================================*/
void Led_Set_State(led_name_t led, led_state_t state)
{
    if (led < LED_COUNT)
    {
        switch (state)
        {
            case LED_OFF: // 输出熄灭电平。
                Led_Write_State(&led_configs[led], GPIO_PIN_RESET);
                break;
            case LED_ON: // 输出点亮电平。
                Led_Write_State(&led_configs[led], GPIO_PIN_SET);
                break;
            case LED_TOGGLE: // 翻转当前电平。
                HAL_GPIO_TogglePin(led_configs[led].GPIO_Port, led_configs[led].GPIO_Pin);
                break;
            default: // 未知指令保持当前电平。
                break;
        }
    }
}

void Led_All_Off(void)
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        Led_Write_State(&led_configs[i], GPIO_PIN_RESET);
    }
}

void Led_All_On(void)
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        Led_Write_State(&led_configs[i], GPIO_PIN_SET);
    }
}

/* ============================================================================
 * 初始化与互斥锁仲裁
 * ==========================================================================*/
void Led_Init(void)
{
    led_mutex = osMutexNew(NULL);
}

bool Led_Try_Acquire(led_name_t led)
{
    (void)led;
    if (led_mutex == NULL) return false;
    return osMutexAcquire(led_mutex, 0) == osOK;
}

void Led_Release(led_name_t led)
{
    (void)led;
    if (led_mutex == NULL) return;
    osMutexRelease(led_mutex);
}
