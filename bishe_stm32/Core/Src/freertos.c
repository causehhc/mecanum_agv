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
carInfoType carInfo;
pidInfoType pidInfoList[4];
motorInfoType motorInfoList[4];
uint8_t ipaddr[20] = "0:192.168.2.1";

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskMotor */
osThreadId_t myTaskMotorHandle;
const osThreadAttr_t myTaskMotor_attributes = {
  .name = "myTaskMotor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTaskDisplay */
osThreadId_t myTaskDisplayHandle;
const osThreadAttr_t myTaskDisplay_attributes = {
  .name = "myTaskDisplay",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void OLED_flash_format(){
  OLED_ShowString(1, 0, (uint8_t *)"IPADDR", 8);
  OLED_ShowString(1, 3, (uint8_t *)"A-ENC", 8);
  OLED_ShowString(1, 4, (uint8_t *)"B-ENC", 8);
  OLED_ShowString(1, 5, (uint8_t *)"C-ENC", 8);
  OLED_ShowString(1, 6, (uint8_t *)"D-ENC", 8);
}
void show_info(int data, uint8_t x, uint8_t y){
  static uint8_t str[6];
  static uint8_t len;

  len = 6;
  for(uint8_t i=0; i<len; i++){
    str[i] = '0';
  }
  if(data >= 0){
    str[0] = '+';
  }else{
    str[0] = '-';
    data = -data;
  }
  str[--len] = '\0';
  while(data){
    str[--len] = data % 10 + '0';
    data /= 10;
  }

  OLED_ShowString(x, y, str, 8);
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskMotor(void *argument);
void StartTaskDisplay(void *argument);

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

  init_motorInfo(&motorInfoList[0],&pidInfoList[0], &htim2, &htim1, &htim8, TIM_CHANNEL_1, TIM_CHANNEL_1);
  init_motorInfo(&motorInfoList[1],&pidInfoList[1], &htim3, &htim1, &htim8, TIM_CHANNEL_2, TIM_CHANNEL_2);
  init_motorInfo(&motorInfoList[2],&pidInfoList[2], &htim4, &htim1, &htim8, TIM_CHANNEL_3, TIM_CHANNEL_3);
  init_motorInfo(&motorInfoList[3],&pidInfoList[3], &htim5, &htim1, &htim8, TIM_CHANNEL_4, TIM_CHANNEL_4);

  start_data();
  start_motor(motorInfoList, 4);
  OLED_Init();
  OLED_preFlash();
  OLED_flash_format();
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

  /* creation of myTaskDisplay */
  myTaskDisplayHandle = osThreadNew(StartTaskDisplay, NULL, &myTaskDisplay_attributes);

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
//    printf("%d,%d,%d\r\n",motorInfoList[0].pidInfo->PWM/1000, motorInfoList[0].pidInfo->ENC, carInfo.x);
    osDelay(100);
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

/* USER CODE BEGIN Header_StartTaskDisplay */
/**
* @brief Function implementing the myTaskDisplay thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDisplay */
void StartTaskDisplay(void *argument)
{
  /* USER CODE BEGIN StartTaskDisplay */
  /* Infinite loop */
  for(;;)
  {
    OLED_ShowString(1, 1, ipaddr, 8);
    show_info(motorInfoList[0].pidInfo->ENC, 1+8*6, 3);
    show_info(motorInfoList[1].pidInfo->ENC, 1+8*6, 4);
    show_info(motorInfoList[2].pidInfo->ENC, 1+8*6, 5);
    show_info(motorInfoList[3].pidInfo->ENC, 1+8*6, 6);
    osDelay(100);
  }
  /* USER CODE END StartTaskDisplay */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

