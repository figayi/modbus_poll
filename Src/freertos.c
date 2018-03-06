/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "modbus_poll.h"
#define APP_LOG_MODULE_NAME   "[task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG    
#include "app_log.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId read_weight_task_hdl;
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void read_weight_task(void const * argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* Init FreeRTOS */

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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(read_weight_task, read_weight_task, osPriorityNormal, 0, 256);
  read_weight_task_hdl = osThreadCreate(osThread(read_weight_task), NULL);
  
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  uint16_t reg[10];
  mb_poll_status_t status;
  modbus_poll_init();
  modbus_poll_pre_transmission(0);
  modbus_poll_post_transmission(0);
  /* Infinite loop */
  for(;;)
  {
    /*功能码0x01 读取线圈状态*/
    status= modbus_poll_readCoils(1,10,17,reg);
    if(status!=MODBUS_POLL_STATUS_SUCCESS)
    {
     APP_LOG_ERROR("读线圈错误！\r\n"); 
    }
    else
    {
    APP_LOG_DEBUG("读线圈值：%d.%d.\r\n",reg[0],reg[1]);  
    }
   osDelay(1);
   /*功能码0x0F 写多个线圈*/
   reg[0]=0x5555;
   reg[1]=0xaaaa;
   status= modbus_poll_write_multiple_coils(2,10,17,reg); 
    
   if(status!=MODBUS_POLL_STATUS_SUCCESS)
   {
    APP_LOG_ERROR("写多个线圈错误！\r\n"); 
   }
     osDelay(1);
    status=modbus_poll_read_input_registers(1,20,10,reg);
    
    if(status!=MODBUS_POLL_STATUS_SUCCESS)
    {
     APP_LOG_ERROR("读输入寄存器错误！\r\n"); 
    }
    else
    {
    for(uint8_t i=0;i<10;i++)
    {
    // APP_LOG_DEBUG("读取的输入寄存器值:%d.\r\n",reg[i]); 
    }
    }
    osDelay(1);
    status= modbus_poll_write_single_coil(1,10,0);
    if(status!=MODBUS_POLL_STATUS_SUCCESS)
    {
     APP_LOG_ERROR("写单个线圈错误！\r\n"); 
    }
    osDelay(1);
   /*功能码0x10*/
    status= modbus_poll_write_multiple_registers(1,0,10, reg);
    if(status!=MODBUS_POLL_STATUS_SUCCESS)
    {
     APP_LOG_ERROR("写多个寄存器错误！\r\n"); 
    }
     osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */

void read_weight_task(void const * argument)
{
  uint16_t reg[4];
  mb_poll_status_t status;
  osDelay(1000);
  /* Infinite loop */
  for(;;)
  {
    /*功能码0x03 读保持寄存器*/
  status= modbus_poll_read_holding_registers(2,0,4,reg);
  if(status!=MODBUS_POLL_STATUS_SUCCESS)
  {
   APP_LOG_ERROR("读净重错误！\r\n"); 
  }
  else
  {
  APP_LOG_DEBUG("读净重值：1.%2x 2.%2x 3.%2x 4.%2x\r\n",reg[0],reg[1],reg[2],reg[3]);   
  }
  osDelay(1);
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
