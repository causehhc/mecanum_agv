/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "user/motor.h"
#include "user/data.h"
#include "user/power.h"
#include "user/display.h"
#include "user/debug_com.h"

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
motorInfoType motorInfoList[4];
carInfoType carInfo;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskMotor */
osThreadId_t myTaskMotorHandle;
const osThreadAttr_t myTaskMotor_attributes = {
  .name = "myTaskMotor",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskMotor(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  carInfo.x = 0;
  carInfo.y = 0;
  carInfo.z = 0;
  init_motorInfo(&motorInfoList[0], htim2, htim1, htim8, TIM_CHANNEL_1, TIM_CHANNEL_1);
  init_motorInfo(&motorInfoList[1], htim3, htim1, htim8, TIM_CHANNEL_2, TIM_CHANNEL_2);
  init_motorInfo(&motorInfoList[2], htim4, htim1, htim8, TIM_CHANNEL_3, TIM_CHANNEL_3);
  init_motorInfo(&motorInfoList[3], htim5, htim1, htim8, TIM_CHANNEL_4, TIM_CHANNEL_4);

  start_data();
  start_motor(motorInfoList, 4);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskMotor */
  myTaskMotorHandle = osThreadNew(StartTaskMotor, NULL, &myTaskMotor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
//    printf("%d %d %d\n", carInfo.x, carInfo.y, carInfo.z);
//    for (uint8_t i=0; i<4; i++){
//      printf("%-4d\t%-4d\t%-6d\n", motorInfoList[i].pidInfo.ENC, motorInfoList[i].pidInfo.TGT, motorInfoList[i].pidInfo.PWM);
//    }
//    printf("%-4d\t%-4d\t%-6d\n", motorInfoList[0].pidInfo.ENC, motorInfoList[0].pidInfo.TGT, motorInfoList[0].pidInfo.PWM);
//    printf("%d\n", motorInfoList[0].pidInfo.ENC);
//    printf("%d,%d,%d\r\n",motorInfoList[0].pidInfo.PWM/1000, motorInfoList[0].pidInfo.ENC, carInfo.x);
//    printf("%-4d\t%-4d\t%-6d\n", motorInfoList[2].pidInfo.ENC, motorInfoList[2].pidInfo.TGT, motorInfoList[2].pidInfo.PWM);
//    printf("\n");

//    if(!HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin)){
//      carInfo.x = 10;
//    } else{
//      carInfo.x = 0;
//    }

    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskMotor */
/**
* @brief Function implementing the myTaskMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMotor */
void StartTaskMotor(void *argument)
{
  /* USER CODE BEGIN StartTaskMotor */
  /* Infinite loop */
  for(;;)
  {
    Kinematic_Analysis_4(motorInfoList, &carInfo);
    motor_run(motorInfoList, 4);
    osDelay(10);
  }
  /* USER CODE END StartTaskMotor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

