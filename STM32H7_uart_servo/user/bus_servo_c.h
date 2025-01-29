#ifndef __BUS_SERVO_C_H__
#define __BUS_SERVO_C_H__
#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4; // 串口4 负责R1 L1 通讯
// extern UART_HandleTypeDef huart5; // 串口5 负责R2 L2 通讯
// extern UART_HandleTypeDef huart6; // 串口6 负责R3 L3 通讯

#define UART_MODE 0 // 0: 单线 1: 三线

#if UART_MODE == 0
#define BUS_SERVO_UART_HANDLE_1 huart4
#define BUS_SERVO_UART_HANDLE_2 huart4
#define BUS_SERVO_UART_HANDLE_3 huart4

#elif UART_MODE == 1
#define BUS_SERVO_UART_HANDLE_1 huart4
#define BUS_SERVO_UART_HANDLE_2 huart5
#define BUS_SERVO_UART_HANDLE_3 huart6
#endif

void UART_Servo_Receive_Enable(void);
void UART_Servo_Rx_Callback(void);
uint8_t GetVersion(uint8_t ID);
void Set_UART_Servo_Angle(uint8_t ID, uint8_t angle, uint8_t time); // 舵机控制
void Set_LED_Enable(uint8_t ID, uint8_t enable);
uint8_t Get_Servo_Angle(uint8_t ID);
void Set_Servo_ID(uint8_t ID, uint8_t new_ID);
float Read_Servo_Angle_Offset(uint8_t ID);
void Set_Servo_angle_offset(uint8_t ID, float angle_offset);
#endif

/* 串口接收回调函数，放到主函数中
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    UART_Servo_Rx_Callback();
  }
}
*/