/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdlib.h"
#include "string.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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
uint8_t S_Flag = 0 , MSG_FLAG=0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId SerialTaskHandle;
osMessageQId SerialQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StuckLOG(const char*p)
{
    uint8_t i = 0;
    while(p[i] != '\0')
    {
        printf("%c",p[i]);
        i++;
    }
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartSerialTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  LOG("Initializing FreeRTOS %f\r\n",1145141919.810);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */
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

  /* Create the queue(s) */
  /* definition and creation of SerialQueue */
  osMessageQDef(SerialQueue, 8, sizeof(uint8_t*));
  SerialQueueHandle = osMessageCreate(osMessageQ(SerialQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SerialTask */
  osThreadDef(SerialTask, StartSerialTask, osPriorityNormal, 0, 1024);
  SerialTaskHandle = osThreadCreate(osThread(SerialTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)//Similar to SVC 
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 5000);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 5000);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 5000);
    osDelay(10);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartSerialTask */
/**
* @brief Function implementing the SerialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSerialTask */
void StartSerialTask(void const * argument)
{
  /* USER CODE BEGIN StartSerialTask */
  printf("Serial Task Started\r\n");
  vTaskSuspend(defaultTaskHandle);
  static uint8_t SerialName[]="Serial Task:: ";
  static uint8_t *S_Buff_Ptr=NULL;
  static BaseType_t STA;
  /* Infinite loop */
  for(;;)
  {
    STA = xQueuePeek(SerialQueueHandle, &S_Buff_Ptr, 1);
    switch(STA)
    {
      case pdPASS:
        MSG_FLAG = 1;
        break;
      case errQUEUE_EMPTY:
        MSG_FLAG = 0;
        printf("Serial:: Queue Empty\r\n");
        break;
      default:
        MSG_FLAG = 0;
        break;
    }
    if(S_Flag == 1 && MSG_FLAG == 1)
    {
      StuckLOG(S_Buff_Ptr);
      //printf("%c\r\n",S_Buff_Ptr[6]);
      S_Flag = 0;
    }
    osDelay(1);
  }
  /* USER CODE END StartSerialTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint8_t msg[]="KEY Pressed\r\n";
  static uint8_t *msg_ptr = NULL;
  msg_ptr = &msg;
  if (GPIO_Pin == KEY_Pin)
  {
    
    if (xQueueSendToBackFromISR(SerialQueueHandle, &msg_ptr, NULL) != pdTRUE)
    {
      printf("EXTI:: Queue Full\r\n");
    }
    xTaskResumeFromISR(defaultTaskHandle);
    S_Flag = 1;
    //printf("EXTI:: KEY Status Transmitted\r\n");
    //osThreadSuspendAll();
    //printf("Stop All\r\n");
  }
}

/* USER CODE END Application */
