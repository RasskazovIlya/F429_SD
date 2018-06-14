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
#include "fatfs.h"
#include "MPU9255.h"

/* USER CODE BEGIN 0 */
//249
#define	Max_Write_Size 249 

extern FATFS SDFatFs;
extern FIL MyFile;
FRESULT res;
uint16_t kek;

extern int16_t accel[3], gyro[3];

volatile uint16_t read_count = 0;

extern uint16_t send_flag;

uint16_t buf_accel[Max_Write_Size], buf_accel2[Max_Write_Size];
uint16_t buf_gyro[Max_Write_Size], buf_gyro2[Max_Write_Size];
uint8_t buf_accel3[2*Max_Write_Size], buf_accel4[2*Max_Write_Size];
uint8_t buf_gyro3[2*Max_Write_Size], buf_gyro4[2*Max_Write_Size];
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern SD_HandleTypeDef hsd;
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
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	uint32_t byteswritten, bytesread;

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	if (read_count < 83)//83
	{
		uint16_t i = 3*read_count;
		
		MPU9250_READ_ACCEL();//read accelerometer data
		buf_accel[i] = accel[0];
		buf_accel[i+1] = accel[1];
		buf_accel[i+2] = accel[2];

		//converse accelerometer data from 16 bit to 8 bit (LSB first)
		buf_accel3[2*i] = (uint8_t)(buf_accel[i]);
		buf_accel3[2*i+1] = (uint8_t)(((buf_accel[i])>>8));
		buf_accel3[2*(i+1)] = (uint8_t)(buf_accel[i+1]);
		buf_accel3[2*(i+1)+1] = (uint8_t)((buf_accel[i+1])>>8);
		buf_accel3[2*(i+2)] = (uint8_t)(buf_accel[i+2]);
		buf_accel3[2*(i+2)+1] = (uint8_t)((buf_accel[i+2])>>8);
		
		MPU9250_READ_GYRO();//read gyroscope data
		buf_gyro[i] = gyro[0];
		buf_gyro[i+1] = gyro[1];
		buf_gyro[i+2] = gyro[2];
		
		buf_gyro3[2*i] = (uint8_t)(buf_gyro[i]);
		buf_gyro3[2*i+1] = (uint8_t)(((buf_gyro[i])>>8));
		buf_gyro3[2*(i+1)] = (uint8_t)(buf_gyro[i+1]);
		buf_gyro3[2*(i+1)+1] = (uint8_t)((buf_gyro[i+1])>>8);
		buf_gyro3[2*(i+2)] = (uint8_t)(buf_gyro[i+2]);
		buf_gyro3[2*(i+2)+1] = (uint8_t)((buf_gyro[i+2])>>8);
		
		read_count++;//1 second data counter increments
	}
	else
	{
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == RESET)
		{
			res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);//mount drive on uSD
			if( res != FR_OK)
				Error_Handler();
			else
			{
				res = f_open(&MyFile, "1.txt", FA_WRITE | FA_OPEN_ALWAYS);//open file to write data
				if (res != FR_OK)
					Error_Handler();
				else
				{
					res = f_write(&MyFile, buf_accel, sizeof(buf_accel), &byteswritten);//write accelerometer data
					//f_lseek(&MyFile, MyFile.fsize);
					res = f_sync(&MyFile);
					res = f_write(&MyFile, buf_gyro, sizeof(buf_gyro), &byteswritten);//write gyroscope data
					res = f_close(&MyFile);//close file
				}
			}
			f_mount(0, (TCHAR const*)SD_Path, 0);//unmount drive
			
			HAL_UART_Transmit(&huart1, buf_accel3, sizeof(buf_accel3), 100);
			HAL_UART_Transmit(&huart1, buf_gyro3, sizeof(buf_accel3), 100);
//			HAL_UART_Transmit(&huart1, (uint8_t *)buf_accel, sizeof(buf_accel), 100);//transmit accelerometer data
//			HAL_UART_Transmit(&huart1, (uint8_t *)buf_gyro, sizeof(buf_gyro), 100);//transmit gyroscope data
		
			res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);//mount drive on uSD
			if( res != FR_OK)
				Error_Handler();
			else 
			{
				res = f_open(&MyFile, "1.txt", FA_READ | FA_OPEN_ALWAYS);//open file to read data
				if (res != FR_OK)
					Error_Handler();
				else
				{
					res = f_read(&MyFile, buf_accel2, sizeof(buf_accel2), &byteswritten);//read accelerometer data from file
					res = f_read(&MyFile, buf_gyro2, sizeof(buf_gyro2), &byteswritten);//read gyroscope data from file
					res = f_close(&MyFile);//close file
				}
			}
			f_mount(0, (TCHAR const*)SD_Path, 0);//unmount drive
			
			for (uint16_t i = 0; i < Max_Write_Size; i+= 3)
			{
				buf_accel4[2*i] = (uint8_t)(buf_accel2[i]);
				buf_accel4[2*i+1] = (uint8_t)(((buf_accel2[i])>>8));
				buf_accel4[2*(i+1)] = (uint8_t)(buf_accel2[i+1]);
				buf_accel4[2*(i+1)+1] = (uint8_t)((buf_accel2[i+1])>>8);
				buf_accel4[2*(i+2)] = (uint8_t)(buf_accel2[i+2]);
				buf_accel4[2*(i+2)+1] = (uint8_t)((buf_accel2[i+2])>>8);
				
				buf_gyro4[2*i] = (uint8_t)(buf_gyro2[i]);
				buf_gyro4[2*i+1] = (uint8_t)(((buf_gyro2[i])>>8));
				buf_gyro4[2*(i+1)] = (uint8_t)(buf_gyro2[i+1]);
				buf_gyro4[2*(i+1)+1] = (uint8_t)((buf_gyro2[i+1])>>8);
				buf_gyro4[2*(i+2)] = (uint8_t)(buf_gyro2[i+2]);
				buf_gyro4[2*(i+2)+1] = (uint8_t)((buf_gyro2[i+2])>>8);
			}
			
				HAL_UART_Transmit(&huart1, (uint8_t *)buf_accel4, sizeof(buf_accel4), 100);//transmit accelerometer data from file
				HAL_UART_Transmit(&huart1, (uint8_t *)buf_gyro4, sizeof(buf_gyro4), 100);//transmit gyroscope data from file
//			HAL_UART_Transmit(&huart1, (uint8_t *)buf_gyro2, sizeof(buf_gyro2), 100);
		
			read_count = 0;//1 second data counter sets to zero
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);//
			//send_flag = 1;
		}
		else 
		{
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
			HAL_Delay(100);
		}
	}
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

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles SDIO global interrupt.
*/
void SDIO_IRQHandler(void)
{
  /* USER CODE BEGIN SDIO_IRQn 0 */

  /* USER CODE END SDIO_IRQn 0 */
  HAL_SD_IRQHandler(&hsd);
  /* USER CODE BEGIN SDIO_IRQn 1 */

  /* USER CODE END SDIO_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
