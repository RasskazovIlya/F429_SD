/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "fatfs.h"
#include "MPU9255.h"
#include <string.h>
#include <stdlib.h>

extern FATFS SDFatFs;
extern FIL MyFile;
FRESULT res;

extern int16_t accel[3], gyro[3];

uint16_t buf_accel[3], buf_accel2[3];
uint16_t buf_gyro[3], buf_gyro2[3];
uint8_t buf_accel3[6], buf_accel4[6];
uint8_t buf_gyro3[6], buf_gyro4[6];

uint8_t INT_flag = 0, STOP_flag = 0;

uint32_t time_mark = 0;
uint8_t time_mark8bit[4];

uint32_t num_str = 0;
uint8_t num_str8bit [4] = {0};

uint8_t command[24] = {0};
uint32_t abs_time_mark = 0;

char DATE_Path[28] = {0}, bufDATE_Path[23];
uint16_t date[7];

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	uint32_t byteswritten, bytesread;
	
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
//	if (time_mark == 0)
//		for (uint8_t i = 0; i < 20; i++)
//		{
//			MPU9250_READ_ACCEL();
//			MPU9250_READ_GYRO();
//		}
//	if (read_count < 33)//83
//	{
//		uint16_t i = 3*read_count;
//		
//		MPU9250_READ_ACCEL();//read accelerometer data
//		buf_accel[i] = accel[0];
//		buf_accel[i+1] = accel[1];
//		buf_accel[i+2] = accel[2];

//		//convert accelerometer data from 16 bit to 8 bit (LSB first)
//		buf_accel3[2*i] = (uint8_t)(buf_accel[i]);
//		buf_accel3[2*i+1] = (uint8_t)(((buf_accel[i])>>8));
//		buf_accel3[2*(i+1)] = (uint8_t)(buf_accel[i+1]);
//		buf_accel3[2*(i+1)+1] = (uint8_t)((buf_accel[i+1])>>8);
//		buf_accel3[2*(i+2)] = (uint8_t)(buf_accel[i+2]);
//		buf_accel3[2*(i+2)+1] = (uint8_t)((buf_accel[i+2])>>8);
//		
//		MPU9250_READ_GYRO();//read gyroscope data
//		buf_gyro[i] = gyro[0];
//		buf_gyro[i+1] = gyro[1];
//		buf_gyro[i+2] = gyro[2];
//		
//		//convert gyroscope data from 16 bit to 8 bit (LSB first)
//		buf_gyro3[2*i] = (uint8_t)(buf_gyro[i]);
//		buf_gyro3[2*i+1] = (uint8_t)(((buf_gyro[i])>>8));
//		buf_gyro3[2*(i+1)] = (uint8_t)(buf_gyro[i+1]);
//		buf_gyro3[2*(i+1)+1] = (uint8_t)((buf_gyro[i+1])>>8);
//		buf_gyro3[2*(i+2)] = (uint8_t)(buf_gyro[i+2]);
//		buf_gyro3[2*(i+2)+1] = (uint8_t)((buf_gyro[i+2])>>8);
//		
//		read_count++;//1 second data counter increments
//	}
//	else
//	{
//		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == RESET)
//		{
//			res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);//mount drive on uSD
//			if( res != FR_OK)
//				Error_Handler();
//			else
//			{
//				res = f_open(&MyFile, "1.txt", FA_WRITE | FA_OPEN_ALWAYS);//open file to write data
//				if (res != FR_OK)
//					Error_Handler();
//				else
//				{
//					res = f_write(&MyFile, buf_accel, sizeof(buf_accel), &byteswritten);//write accelerometer data
//					//f_lseek(&MyFile, MyFile.fsize);
//					res = f_sync(&MyFile);
//					res = f_write(&MyFile, buf_gyro, sizeof(buf_gyro), &byteswritten);//write gyroscope data
//					res = f_close(&MyFile);//close file
//				}
//			}
//			f_mount(0, (TCHAR const*)SD_Path, 0);//unmount drive
//			
//			HAL_UART_Transmit(&huart1, buf_accel3, sizeof(buf_accel3), 100);//transmit accelerometer data
//			HAL_UART_Transmit(&huart1, buf_gyro3, sizeof(buf_accel3), 100);//transmit gyroscope data
//		
//			res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);//mount drive on uSD
//			if( res != FR_OK)
//				Error_Handler();
//			else 
//			{
//				res = f_open(&MyFile, "1.txt", FA_READ | FA_OPEN_ALWAYS);//open file to read data
//				if (res != FR_OK)
//					Error_Handler();
//				else
//				{
//					res = f_read(&MyFile, buf_accel2, sizeof(buf_accel2), &byteswritten);//read accelerometer data from file
//					res = f_read(&MyFile, buf_gyro2, sizeof(buf_gyro2), &byteswritten);//read gyroscope data from file
//					res = f_close(&MyFile);//close file
//				}
//			}
//			f_mount(0, (TCHAR const*)SD_Path, 0);//unmount drive
//			
//			for (uint16_t i = 0; i < Max_Write_Size; i+= 3)
//			{
//				buf_accel4[2*i] = (uint8_t)(buf_accel2[i]);
//				buf_accel4[2*i+1] = (uint8_t)(((buf_accel2[i])>>8));
//				buf_accel4[2*(i+1)] = (uint8_t)(buf_accel2[i+1]);
//				buf_accel4[2*(i+1)+1] = (uint8_t)((buf_accel2[i+1])>>8);
//				buf_accel4[2*(i+2)] = (uint8_t)(buf_accel2[i+2]);
//				buf_accel4[2*(i+2)+1] = (uint8_t)((buf_accel2[i+2])>>8);
//				
//				buf_gyro4[2*i] = (uint8_t)(buf_gyro2[i]);
//				buf_gyro4[2*i+1] = (uint8_t)(((buf_gyro2[i])>>8));
//				buf_gyro4[2*(i+1)] = (uint8_t)(buf_gyro2[i+1]);
//				buf_gyro4[2*(i+1)+1] = (uint8_t)((buf_gyro2[i+1])>>8);
//				buf_gyro4[2*(i+2)] = (uint8_t)(buf_gyro2[i+2]);
//				buf_gyro4[2*(i+2)+1] = (uint8_t)((buf_gyro2[i+2])>>8);
//			}
//			
//			HAL_UART_Transmit(&huart1, (uint8_t *)buf_accel4, sizeof(buf_accel4), 100);//transmit accelerometer data from file
//			HAL_UART_Transmit(&huart1, (uint8_t *)buf_gyro4, sizeof(buf_gyro4), 100);//transmit gyroscope data from file
//		
////			time_mark++;
////			time_mark8bit[0] = (uint8_t)(time_mark);
////			time_mark8bit[1] = (uint8_t)(time_mark>>8);
////			
////			HAL_UART_Transmit(&huart1, (uint8_t *)time_mark8bit, sizeof(time_mark8bit), 100);//transmit time mark (counter of read/write operations)
//			
//			read_count = 0;//1 second data counter sets to zero
//			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);//
//			//send_flag = 1;
//		}
//		else 
//		{
//			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
//			HAL_Delay(100);
//		}
//	}
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles I2C1 event interrupt.
*/
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	uint32_t bytesread;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	HAL_UART_Receive(&huart1, command, sizeof(command), 10);//receive command
	if (INT_flag == 0)
	{
		if ( command[0] == 's' )//if start_command
		{
			for (int i = 0; i < 23; i++)
				DATE_Path[i] = command[i+1];
			
			DATE_Path[23] = '.';
			DATE_Path[24] = 't';
			DATE_Path[25] = 'x';
			DATE_Path[26] = 't';
			DATE_Path[27] = '\0';//set file name to format "dd_mm_yyyy_HH_MM_SS_FFF.txt"

			strncpy(bufDATE_Path, DATE_Path, 23);
			char *buf = strtok(bufDATE_Path, "_");
			int i = 0;
			while(buf != NULL)
			{
				date[i++] = atol(buf);
				buf = strtok(NULL, "_");
			}//get numbers from DATE_Path string
			
			EXTI->IMR |= (EXTI_IMR_MR9);//mask on EXTI9 interrupt
			MPU9250_Init();//initialize MPU9255
			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);//turn on EXTI9 interrupt
			
			INT_flag = 1;
			STOP_flag = 0;
		}
		else if ( command[0] == 'h' )//if stop_command
		{
			res = f_close(&MyFile);//(?)close file
			res = f_mount(0, (TCHAR const*)SD_Path, 0);//(?)unmount drive
			//HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
			EXTI->IMR &= ~(EXTI_IMR_MR9);//mask on EXTI9 interrupt, stop interrupts from EXTI9 (MPU INT pin)
			
			//INT_flag = 1;
			STOP_flag = 1;
		}
		else if ( command[0] == 'f' && STOP_flag == 1 )//if send_command and stopped interrupts
		{
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == RESET)
				{
					res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);//mount drive on uSD
					if( res != FR_OK)
						Error_Handler();
					else
					{				
						res = f_open(&MyFile, (const char*)DATE_Path, FA_READ);//open file to read data
						if (res != FR_OK)
							Error_Handler();
						else
						{
							num_str = f_size(&MyFile)/(sizeof(buf_accel) + sizeof(buf_gyro));//get amount of data in file

							//32 to 8 bit conversion 
							num_str8bit[0] = (uint8_t)(num_str);
							num_str8bit[1] = (uint8_t)(num_str >> 8);
							num_str8bit[2] = (uint8_t)(num_str >> 16);
							num_str8bit[3] = (uint8_t)(num_str >> 24);
							HAL_UART_Transmit(&huart1, num_str8bit, sizeof(time_mark), 10);
//							for (uint32_t i = 0; i < num_str; i++)
//							{
//								f_read(&MyFile, buf_accel2, sizeof(buf_accel2), &bytesread);
//								f_read(&MyFile, buf_gyro2, sizeof(buf_accel2), &bytesread);
//								buf_accel4[0] = (uint8_t)(buf_accel2[0]);
//								buf_accel4[1] = (uint8_t)(((buf_accel2[0])>>8));
//								buf_accel4[2] = (uint8_t)(buf_accel2[1]);
//								buf_accel4[3] = (uint8_t)((buf_accel2[1])>>8);
//								buf_accel4[4] = (uint8_t)(buf_accel2[2]);
//								buf_accel4[5] = (uint8_t)((buf_accel2[2])>>8);
//								
//								buf_gyro4[0] = (uint8_t)(buf_gyro2[0]);
//								buf_gyro4[1] = (uint8_t)(((buf_gyro2[0])>>8));
//								buf_gyro4[2] = (uint8_t)(buf_gyro2[1]);
//								buf_gyro4[3] = (uint8_t)((buf_gyro2[1])>>8);
//								buf_gyro4[4] = (uint8_t)(buf_gyro2[2]);
//								buf_gyro4[5] = (uint8_t)((buf_gyro2[2])>>8);
//								
////								HAL_UART_Transmit(&huart1, (uint8_t *)buf_accel4, sizeof(buf_accel4), 100);//transmit accelerometer data from file
////								HAL_UART_Transmit(&huart1, (uint8_t *)buf_gyro4, sizeof(buf_gyro4), 100);//transmit gyroscope data from file
//							}
						}
						res = f_close(&MyFile);//close file
					}
					f_mount(0, (TCHAR const*)SD_Path, 0);//unmount drive
				}
			INT_flag = 1;
		}
	}
	else INT_flag = 0;
	USART1->CR1 |= USART_CR1_RXNEIE;

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	USART1->CR1 &= ~(USART_CR1_RXNEIE);//mask on USART1 RXNE interrupts, so that commands won't interfere with data writing to file
	uint8_t INT_status = 0;
	uint32_t byteswritten, bytesread;
	
	if (command[0] == 's')//if start_command
	{
		HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, INT_STATUS, 1, &INT_status, 1, 100);//get status of MPU INT register 
		if (INT_status == 1)
		{
			//read accelerometer data
			MPU9250_READ_ACCEL();
			buf_accel[0] = accel[0];
			buf_accel[1] = accel[1];
			buf_accel[2] = accel[2];
			
			//read gyroscope data
			MPU9250_READ_GYRO();
			buf_gyro[0] = gyro[0];
			buf_gyro[1] = gyro[1];
			buf_gyro[2] = gyro[2];

			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == RESET)//if uSD is inserted
			{
				res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);//mount drive on uSD
				if( res != FR_OK)
					Error_Handler();
				else
				{				
					res = f_open(&MyFile, (const char*)DATE_Path, FA_WRITE | FA_OPEN_ALWAYS);//open file to write data
					if (res != FR_OK)
						Error_Handler();
					else
					{
						f_lseek(&MyFile, MyFile.fsize);
						res = f_write(&MyFile, buf_accel, sizeof(buf_accel), &byteswritten);//write accelerometer data
						res = f_sync(&MyFile);
						res = f_write(&MyFile, buf_gyro, sizeof(buf_gyro), &byteswritten);//write gyroscope data
						res = f_close(&MyFile);//close file
					}
				}
				res = f_mount(0, (TCHAR const*)SD_Path, 0);//unmount drive
			}
		}
	}
	USART1->CR1 |= USART_CR1_RXNEIE;//mask on USART1 RXNE interrupts, able get commands now
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
