/** @file  led_task.c  @brief 遥控状态显示、CAN 恢复与板载音乐服务  @version  v1.2  2026-11-22 */
#include "cmsis_os2.h"
#include "dr16.h"
#include "led.h"
#include "music.h"
#include "can_bus.h"
#include "system_key.h"

// 遥控离线时 LED1 每隔 100 ms 翻转一次。
#define LED_RC_BLINK_MS 100u

// 每拍取同一帧的在线标志和左拨杆位置，掉线时熄灭档位灯并接管 LED1。
static void Led_Show_Rc_Status(void)
{
    static bool own_led;
    static uint32_t last_blink;
    const dr16_t rc = Dr16_Snapshot();

    if (rc.online)
    {
        if (own_led)
        {
            Led_Set_State(LED1, LED_OFF);
            Led_Release(LED1);
            own_led = false;
        }
    }
    else
    {
        if (!own_led) own_led = Led_Try_Acquire(LED1);
        if (own_led)
        {
            uint32_t now = HAL_GetTick();

            if (now - last_blink >= LED_RC_BLINK_MS)
            {
                Led_Set_State(LED1, LED_TOGGLE);
                last_blink = now;
            }
        }
    }

    Led_Set_State(LED2, rc.online && rc.sw_left == DR16_SW_UP   ? LED_ON : LED_OFF);
    Led_Set_State(LED3, rc.online && rc.sw_left == DR16_SW_MID  ? LED_ON : LED_OFF);
    Led_Set_State(LED4, rc.online && rc.sw_left == DR16_SW_DOWN ? LED_ON : LED_OFF);
}

// 先关闭全部指示灯，再周期刷新遥控显示并处理总线与音乐服务。
void StartLedTask(void *argument)
{
    Led_Init();
    Led_All_Off();

    for (;;)
    {
        Led_Show_Rc_Status();

        // 三路总线允许静默，电机在线检查仍由控制任务执行。
        (void)Can_Bus_Online(false);
#if APP_USE_MUSIC
        Music_Service();
#endif
        osDelay(1u);
    }
}
