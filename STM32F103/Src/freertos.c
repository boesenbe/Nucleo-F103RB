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
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId send_dataHandle;

/* USER CODE BEGIN Variables */

char Buffer_counter[14];
char Buffer_limitswitch[11];
uint8_t limitswitch[6];
uint8_t CURRENT_status[6];
uint32_t count[3];

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void Send_Data(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void getPinStatus(GPIO_TypeDef*, uint16_t, uint8_t*, uint8_t*);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

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

  /* definition and creation of send_data */
  osThreadDef(send_data, Send_Data, osPriorityRealtime, 0, 128);
  send_dataHandle = osThreadCreate(osThread(send_data), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Send_Data function */
void Send_Data(void const * argument)
{
  /* USER CODE BEGIN Send_Data */
  /* Infinite loop */
  for(;;)
  {
	  count[0] = TIM1->CNT;
	  count[1] = TIM2->CNT;
	  count[2] = TIM3->CNT;

	  getPinStatus(GPIOC,GPIO_PIN_6,&CURRENT_status[0],&limitswitch[0]);
	  getPinStatus(GPIOB,GPIO_PIN_13,&CURRENT_status[1],&limitswitch[1]);
	  getPinStatus(GPIOC,GPIO_PIN_8,&CURRENT_status[2],&limitswitch[2]);
	  getPinStatus(GPIOC,GPIO_PIN_9,&CURRENT_status[3],&limitswitch[3]);
	  getPinStatus(GPIOB,GPIO_PIN_14,&CURRENT_status[4],&limitswitch[4]);
	  getPinStatus(GPIOB,GPIO_PIN_15,&CURRENT_status[5],&limitswitch[5]);

	  for (int var = 0; var <= 2; ++var) {
			  HAL_UART_Transmit(&huart1, (uint8_t*)Buffer_counter, sprintf(Buffer_counter, "C  %d)\t %04lu \r\n",var+1, count[var]), 0xFFFF);
	  }

	  for (int var = 0; var <= 5; ++var) {
		  HAL_UART_Transmit(&huart1, (uint8_t*)Buffer_limitswitch, sprintf(Buffer_limitswitch, "LS %d)\t %d \r\n", var+1, limitswitch[var]), 0xFFFF);
	  }

  }
  /* USER CODE END Send_Data */
}

/* USER CODE BEGIN Application */

void getPinStatus(GPIO_TypeDef *GPIOx, uint16_t Pin, uint8_t* CURRENT_status, uint8_t* status ){

	if((uint8_t*)HAL_GPIO_ReadPin(GPIOx,Pin) != CURRENT_status)
			  {  CURRENT_status = (uint8_t*)HAL_GPIO_ReadPin(GPIOx,Pin);

			  	  *status = 0;

			  if(!HAL_GPIO_ReadPin(GPIOx,Pin)){

				  //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				  *status = 1;

			  }
			  }

}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
