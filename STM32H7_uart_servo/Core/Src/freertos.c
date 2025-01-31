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
#include "bus_servo_c.h"
#include "hexapod_servo.h"
#include "tool.h"
#include "user_adc.h"
#include "elrs.h"
#include "IMU948.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern float init_position[6][3];
extern float leg_position[6][3];
extern ELRS_Data elrs_data;
extern uint8_t elrs_is_link;
extern IMU948_Data imu948_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
    .name = "myTask02",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
    .name = "myTask03",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

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
  // hexapod_servo_init();
  init_leg_matrix();
  osDelay(1000);
  // set_test_position();
  memcpy(leg_position, init_position, sizeof(init_position));
  osDelay(50);
  // Set_all_leg_Global_position(leg_position);
  /* Infinite loop */
  for (;;)
  {
    // Set_Servo_ID(0, 3);
    // osDelay(500);
    // Set_Servo_angle_offset(3, 0);
    // osDelay(2000);
    // joint_test(3);
    // osDelay(500);
    // if (elrs_data.A == 1)
    // {
    //   HexapodMove2(0, 0, 60, 30, 10, 2, 1);
    // }
    if (elrs_data.D == 1)
    {
      hexapod_stop_all_servo();
    }
    toggle_led();
    osDelay(50);
    // elrs_Control();
    // HexapodMove_test();
    // Set_servo_Global_position_IK_test();
    // Set_servo_Local_position_IK_test();
    // MoveAndRotateBody_test();
    float v = Get_Voltage();
    if (elrs_is_link == 1)
    {
      // printf("L_X=%.2f,L_Y=%.2f,R_X=%.2f,R_Y=%.2f,A=%d,B=%d,C=%d,D=%d,E=%d,F=%d,V=%.2f,\r\n", elrs_data.Left_X, elrs_data.Left_Y, elrs_data.Right_X, elrs_data.Right_Y, elrs_data.A, elrs_data.B, elrs_data.C, elrs_data.D, elrs_data.E, elrs_data.F, v);
    }
    printf("euler X=%.2f,euler Y=%.2f,euler Z=%.2f,aX=%.2f,aY=%.2f,aZ=%.2f,X=%.1f,Y=%.1f,Z=%.1f,wendu=%.2f,timestamp=%d\r\n", imu948_Data.euler_X, imu948_Data.euler_Y, imu948_Data.euler_Z, imu948_Data.accel_X, imu948_Data.accel_Y, imu948_Data.accel_Z, imu948_Data.pos_X, imu948_Data.pos_Y, imu948_Data.pos_Z, imu948_Data.temperature, imu948_Data.timestamp);
    // printf("Voltage: %f\r\n", v);
    // toggle_led();
    // joint_test(18);
    // osDelay(50);
    // printf("test\r\n");
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
extern uint16_t elrs_heartbeat_counter;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  uint16_t last_elrs_heartbeat_counterl;
  /* Infinite loop */
  for (;;)
  {
    if (last_elrs_heartbeat_counterl == elrs_heartbeat_counter)
    {
      // printf("ELRS 未连接? 重新初始化\r\n");
      elrs_is_link = 0;
      // 停止DMA接收
      HAL_UART_DMAStop(&huart2);
      // 恢复错误中断使能
      ATOMIC_SET_BIT(huart2.Instance->CR3, USART_CR3_EIE);
      // 清除帧错误标��?
      __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_FEF);
      memset(&elrs_data, 0, sizeof(elrs_data));
      ELRS_Init();
    }
    if (HAL_GetTick() - imu948_Data.last_update_time > 1000)
    {
      IMU948_UART_Init();
      printf("IMU948 初始化\r\n");
    }
    last_elrs_heartbeat_counterl = elrs_heartbeat_counter;
    osDelay(1000);
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
