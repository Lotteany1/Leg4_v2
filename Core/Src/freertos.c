/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 栈由下方 CubeMX 任务定义提供。命名段将其放到 DTCM，8 字节对齐供浮点上下文使用。
extern uint32_t Tinympc14TaskBuffer[] __attribute__((section(".mpc_task_stack"), zero_init, aligned(8)));

/* USER CODE END Variables */
/* Definitions for InitTask */
osThreadId_t InitTaskHandle;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for VofaTask */
osThreadId_t VofaTaskHandle;
const osThreadAttr_t VofaTask_attributes = {
  .name = "VofaTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EstimatorTask */
osThreadId_t EstimatorTaskHandle;
const osThreadAttr_t EstimatorTask_attributes = {
  .name = "EstimatorTask",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Tinympc14Task */
osThreadId_t Tinympc14TaskHandle;
uint32_t Tinympc14TaskBuffer[ 4096 ];
osStaticThreadDef_t Tinympc14TaskControlBlock;
const osThreadAttr_t Tinympc14Task_attributes = {
  .name = "Tinympc14Task",
  .cb_mem = &Tinympc14TaskControlBlock,
  .cb_size = sizeof(Tinympc14TaskControlBlock),
  .stack_mem = &Tinympc14TaskBuffer[0],
  .stack_size = sizeof(Tinympc14TaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RadarTask */
osThreadId_t RadarTaskHandle;
const osThreadAttr_t RadarTask_attributes = {
  .name = "RadarTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartInitTask(void *argument);
void StartVofaTask(void *argument);
void StartControlTask(void *argument);
void StartEstimatorTask(void *argument);
void StartTinympc14Task(void *argument);
void StartRadarTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of InitTask */
  InitTaskHandle = osThreadNew(StartInitTask, NULL, &InitTask_attributes);

  /* creation of VofaTask */
  VofaTaskHandle = osThreadNew(StartVofaTask, NULL, &VofaTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of EstimatorTask */
  EstimatorTaskHandle = osThreadNew(StartEstimatorTask, NULL, &EstimatorTask_attributes);

  /* creation of Tinympc14Task */
  Tinympc14TaskHandle = osThreadNew(StartTinympc14Task, NULL, &Tinympc14Task_attributes);

  /* creation of RadarTask */
  RadarTaskHandle = osThreadNew(StartRadarTask, NULL, &RadarTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartInitTask */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInitTask */
__weak void StartInitTask(void *argument)
{
  /* USER CODE BEGIN StartInitTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartInitTask */
}

/* USER CODE BEGIN Header_StartVofaTask */
/**
* @brief Function implementing the VofaTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVofaTask */
__weak void StartVofaTask(void *argument)
{
  /* USER CODE BEGIN StartVofaTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartVofaTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
__weak void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartEstimatorTask */
/**
* @brief Function implementing the EstimatorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEstimatorTask */
__weak void StartEstimatorTask(void *argument)
{
  /* USER CODE BEGIN StartEstimatorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartEstimatorTask */
}

/* USER CODE BEGIN Header_StartTinympc14Task */
/**
* @brief Function implementing the Tinympc14Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTinympc14Task */
__weak void StartTinympc14Task(void *argument)
{
  /* USER CODE BEGIN StartTinympc14Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTinympc14Task */
}

/* USER CODE BEGIN Header_StartRadarTask */
/**
* @brief Function implementing the RadarTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRadarTask */
__weak void StartRadarTask(void *argument)
{
  /* USER CODE BEGIN StartRadarTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRadarTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

