/** @file  led.h  @brief 板载四路指示灯配置与控制接口  @version  v1.1  2026-11-22 */
#ifndef LED_H
#define LED_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>

/* LED_LIST: X(name, GPIO端口, GPIO引脚, 点亮电平) */
#define LED_LIST \
    X(LED1, GPIOB, GPIO_PIN_0, GPIO_PIN_RESET) \
    X(LED2, GPIOB, GPIO_PIN_1, GPIO_PIN_RESET) \
    X(LED3, GPIOB, GPIO_PIN_2, GPIO_PIN_RESET) \
    X(LED4, GPIOE, GPIO_PIN_7, GPIO_PIN_RESET)

typedef struct
{
    GPIO_TypeDef  *GPIO_Port;
    uint16_t       GPIO_Pin;
    GPIO_PinState  active_state;
} led_config_t;

typedef enum
{
#define X(name, port, pin, active) name,
    LED_LIST
#undef X
    LED_COUNT
} led_name_t;

typedef enum
{
    LED_OFF = 0,
    LED_ON,
    LED_TOGGLE
} led_state_t;

void Led_Set_State(led_name_t led, led_state_t state);
void Led_All_Off(void);
void Led_All_On(void);
void Led_Init(void);
bool Led_Try_Acquire(led_name_t led);
void Led_Release(led_name_t led);
#endif // LED_H
