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

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC
uint32_t count_tic;
#define DEVICE_ADDRESS 0x0810C000

uint32_t flash_read(uint32_t address);
void ReadDeviceAddress(char* Dout);

extern FATFS SDFatFs;
extern FIL MyFile;
FRESULT res;

extern int16_t accel[3], gyro[3];

uint16_t buf_accel[3], buf_accel2[3];
uint16_t buf_gyro[3], buf_gyro2[3];
uint8_t buf_accel4[6];
uint8_t buf_gyro4[6];

uint8_t INT_flag = 0, STOP_flag = 1;

uint32_t num_str = 0;
uint8_t num_str8bit [4] = {0};

uint8_t command[24] = {0};
uint32_t abs_time_mark = 0;
uint32_t time_mark = 0;
uint8_t time_mark8bit[4];

char DATE_Path[28] = {0}, flash_DATE_Path[28] = {0};

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

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
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	uint32_t bytesread;
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	HAL_UART_Receive(&huart3, command, sizeof(command), 10);//receive command
	
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == RESET)//if uSD is inserted
	{
		if (INT_flag == 0)//if interrupt flag not set
		{
			if ( command[0] == 's' )//if start_command
			{
				for (int i = 0; i < 23; i++)
					DATE_Path[i] = command[i+1];
				
				DATE_Path[23] = '.';
				DATE_Path[24] = 't';
				DATE_Path[25] = 'x';
				DATE_Path[26] = 't';
				DATE_Path[27] = '\0';//set file name to format "dd_mm_yyyy_HH_MM_SS_FFF.txt\0"
				
				HAL_FLASH_Unlock();//unlock flash memory
				FLASH_Erase_Sector(FLASH_SECTOR_15, VOLTAGE_RANGE_3);//erase sector (needed to rewrite file name)
				HAL_FLASH_Lock();//lock flash memory
				HAL_FLASH_Unlock();//unlock flash memory
				for (int i = 0; i < 28; i++)
				{
					HAL_FLASH_Program(TYPEPROGRAM_BYTE, DEVICE_ADDRESS+i, DATE_Path[i]);
				}//write file name to flash memory
				HAL_FLASH_Lock();//write file name to flash memory

				MPU9250_Init();//initialize MPU9255
				
				for (int i = 0; i < 10; i++)
				{
					MPU9250_READ_ACCEL();
					MPU9250_READ_GYRO();
				}//read data to sort out trash right after MPU starts working
				
				res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);//mount drive on uSD
				res = f_open(&MyFile, (const char*)DATE_Path, FA_WRITE | FA_OPEN_ALWAYS);//open file to write data
				abs_time_mark = HAL_GetTick();//get time since start of F429

				EXTI->IMR |= (EXTI_IMR_MR9);//mask on EXTI9 interrupt
				HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);//turn on EXTI9 interrupt
				
				INT_flag = 1;
				STOP_flag = 0;
			}
			else if ( command[0] == 'h' )//if stop_command
			{
				res = f_close(&MyFile);//close file
				res = f_mount(0, (TCHAR const*)SD_Path, 0);//unmount drive

				EXTI->IMR &= ~(EXTI_IMR_MR9);//mask on EXTI9 interrupt, stop interrupts from EXTI9 (MPU INT pin)
				HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);//turn on EXTI9 interrupt

				STOP_flag = 1;
			}
			else if ( command[0] == 'f' && STOP_flag == 1 )//if send_command and stopped interrupts
			{
				USART3->CR1 &= ~(USART_CR1_RXNEIE);//mask on USART3 RXNE interrupts, so that commands won't interfere with data transmitting
				
				ReadDeviceAddress(flash_DATE_Path);//read last file name from flash
				
				res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);//mount drive on uSD
				if( res != FR_OK)//if error occured
					Error_Handler();
				else
				{				
					res = f_open(&MyFile, (const char*)flash_DATE_Path, FA_READ);//open file to read data
					if (res != FR_OK)//if error occured
						Error_Handler();
					else
					{
						num_str = f_size(&MyFile)/(sizeof(buf_accel) + sizeof(buf_gyro) + sizeof(time_mark));//get amount of data in file

						num_str8bit[0] = (uint8_t)(num_str);
						num_str8bit[1] = (uint8_t)(num_str >> 8);
						num_str8bit[2] = (uint8_t)(num_str >> 16);
						num_str8bit[3] = (uint8_t)(num_str >> 24);//data amount 32 to 8 bit conversion

						HAL_UART_Transmit(&huart3, num_str8bit, sizeof(num_str8bit), 100);//transmit data amount
						for (uint32_t i = 0; i < num_str; i++)
						{
							res = f_read(&MyFile, buf_accel2, sizeof(buf_accel2), &bytesread);//read accel 3 axis data
							res = f_read(&MyFile, buf_gyro2, sizeof(buf_accel2), &bytesread);//read gyro 3 axis data
							res = f_read(&MyFile, &time_mark, sizeof(time_mark), &bytesread);//read time mark
							
							buf_accel4[0] = (uint8_t)(buf_accel2[0]);
							buf_accel4[1] = (uint8_t)(((buf_accel2[0])>>8));
							buf_accel4[2] = (uint8_t)(buf_accel2[1]);
							buf_accel4[3] = (uint8_t)((buf_accel2[1])>>8);
							buf_accel4[4] = (uint8_t)(buf_accel2[2]);
							buf_accel4[5] = (uint8_t)((buf_accel2[2])>>8);//accel 3 axis data 16 to 8 bit conversion
							
							buf_gyro4[0] = (uint8_t)(buf_gyro2[0]);
							buf_gyro4[1] = (uint8_t)(((buf_gyro2[0])>>8));
							buf_gyro4[2] = (uint8_t)(buf_gyro2[1]);
							buf_gyro4[3] = (uint8_t)((buf_gyro2[1])>>8);
							buf_gyro4[4] = (uint8_t)(buf_gyro2[2]);
							buf_gyro4[5] = (uint8_t)((buf_gyro2[2])>>8);//gyro 3 axis data 16 to 8 bit conversion
							
							time_mark8bit[0] = (uint8_t)(time_mark);
							time_mark8bit[1] = (uint8_t)(time_mark >> 8);
							time_mark8bit[2] = (uint8_t)(time_mark >> 16);
							time_mark8bit[3] = (uint8_t)(time_mark >> 24);//time mark 32 to 8 bit conversion
							
							HAL_UART_Transmit(&huart3, (uint8_t *)buf_accel4, sizeof(buf_accel4), 100);//transmit accelerometer data from file
							HAL_UART_Transmit(&huart3, (uint8_t *)buf_gyro4, sizeof(buf_gyro4), 100);//transmit gyroscope data from file
							HAL_UART_Transmit(&huart3, (uint8_t *)time_mark8bit, sizeof(time_mark8bit), 100);//transmit time mark
						}
					}
					res = f_close(&MyFile);//close file
				}
				res = f_mount(0, (TCHAR const*)SD_Path, 0);//unmount drive and stopped interrupts
				
				USART3->CR1 |= USART_CR1_RXNEIE;
			}
			else if ( command[0] == 'e'  && STOP_flag == 1 ) //if erase_command 
			{
				ReadDeviceAddress(flash_DATE_Path);//read latest file name from flash
				res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);//mount drive on uSD
				res = f_unlink(flash_DATE_Path);//remove latest file from uSD
				res = f_mount(0, (TCHAR const*)SD_Path, 0);//unmount drive
			}
		}
		else INT_flag = 0;
	}
	else
	{
		for (int i = 0; i < 20; i++)//flash LD3 10 times to show, that uSD is not inserted
		{
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
			HAL_Delay(200);
		}
	}
	
	USART3->CR1 |= USART_CR1_RXNEIE;//mask on USART3 RXNE interrupt, HAL_UART_Receive sets RXNEIE to 0
  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	USART3->CR1 &= ~(USART_CR1_RXNEIE);//mask on USART3 RXNE interrupt, so that commands won't interfere with data writing to file

	uint8_t INT_status = 0;
	uint32_t byteswritten;

	if (command[0] == 's')//if start_command
	{
		HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, INT_STATUS, 1, &INT_status, 1, 100);//get status of MPU INT register
		if (INT_status == 1)//if "data ready to read" interrupt flag set
		{
			MPU9250_READ_ACCEL();//read accelerometer data
			buf_accel[0] = accel[0];
			buf_accel[1] = accel[1];
			buf_accel[2] = accel[2];
			
			MPU9250_READ_GYRO();//read gyroscope data
			buf_gyro[0] = gyro[0];
			buf_gyro[1] = gyro[1];
			buf_gyro[2] = gyro[2];
						
			time_mark = HAL_GetTick() - abs_time_mark;//get time since start command
			f_lseek(&MyFile, MyFile.fsize);//set file pointer to the end of file
			res = f_write(&MyFile, buf_accel, sizeof(buf_accel), &byteswritten);//write accelerometer data
			res = f_write(&MyFile, buf_gyro, sizeof(buf_gyro), &byteswritten);//write gyroscope data
			res = f_write(&MyFile, &time_mark, sizeof(time_mark), &byteswritten);//write time mark
		}
	}
	USART3->CR1 |= USART_CR1_RXNEIE;//mask on USART1 RXNE interrupt, able get commands now
}

uint32_t flash_read(uint32_t address)
{
	return (*(__IO uint32_t*) address);
}

void ReadDeviceAddress(char* Dout) //read file name from flash memory
{
	uint32_t temp, k = 0;

	for (int i = 0; i < 7; i++)
	{
		temp = flash_read(DEVICE_ADDRESS + (4*i));

		for (int j = 0; j < 4; j++)
		{
			Dout[k] = (char)( (temp>>(j*8)) & 0xFF );
			k++;
		}
	}
	Dout[27]=0;
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
