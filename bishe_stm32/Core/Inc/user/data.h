//
// Created by hhc on 2022/4/15.
//

#ifndef BISHE_STM32_DATA_H
#define BISHE_STM32_DATA_H
#include "usart.h"
void start_data();
void send_data(uint8_t *data, uint8_t len);

void USER_UART_IDLECallback(UART_HandleTypeDef *huart);

#endif //BISHE_STM32_DATA_H
