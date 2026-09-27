#pragma once
#include "headfile.h"

extern void UART_Poll_Command(int16_t target[Motor_Count]);
uint8_t UART_Command_Ack_Update(uint32_t now_ms);
extern uint8_t uart2_rx_byte ;
