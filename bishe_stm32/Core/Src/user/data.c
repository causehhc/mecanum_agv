//
// Created by hhc on 2022/4/15.
//

#include "user/data.h"
#include "user/motor.h"
extern carInfoType carInfo;
extern motorInfoType motorInfoList[4];

uint8_t RxBuffer[20];
__IO uint8_t RxCounter = 0;
uint8_t Rx_Temp;

void start_data(){
  __HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
  HAL_UART_Receive_IT(&huart4,(uint8_t*)&Rx_Temp,1);
}

void send_data(uint8_t *data, uint8_t len){
  HAL_UART_Transmit(&huart4, data, len, 0xffff);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == UART4){
    RxBuffer[RxCounter++] = Rx_Temp;
    HAL_UART_Receive_IT(&huart4,(uint8_t*)&Rx_Temp,1);
  }
}

void user_api(){
  if(RxBuffer[0]=='['&&RxBuffer[4]==']'){
    // PZdnx
//    motorInfoList[0].pidInfo.Velocity_KP = (RxBuffer[1]-50)*10;
//    motorInfoList[0].pidInfo.Velocity_KI = (RxBuffer[2]-50)*1;
    carInfo.x = RxBuffer[1]-100;
    carInfo.y = RxBuffer[2]-100;
    carInfo.z = RxBuffer[3]-100;
  }
}

void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == UART4){
    if(RESET != __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE)){
      user_api();
      HAL_UART_Transmit(&huart4, RxBuffer, RxCounter, 0xffff);
      RxCounter = 0;
      __HAL_UART_CLEAR_IDLEFLAG(huart);
    }
  }
}

