#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "MPU9255.h"

extern I2C_HandleTypeDef hi2c1;
extern int16_t accel[3], gyro[3];// x, y, z axis for accelerometer and gyroscope
MPU9250_TypeDef MPU9250_Offset={0};

unsigned char BUF[10];

void MPU9250_Init(void)
{
	uint8_t R;
  HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, PWR_MGMT_1, 1, 0x00, 1, 100);
	HAL_Delay(100);
	
	//Set clock source to PLL if ready, else - internal oscillator (20 MHz)
	R = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, PWR_MGMT_1, 1, &R, 1, 100);
	
	//Set Sample Rate Divider to 0, so that SAMPLE_RATE=Internal_Sample_Rate/1
	R = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, SMPLRT_DIV, 1, &R, 1, 100);
	
	//Set accelerometer and gyroscope bandwidth to 41 Hz, frequency to 1 kHz
	R = 0x03;
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, CONFIG, 1, &R, 1, 100);
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, ACCEL_CONFIG2, 1, &R, 1, 100);
	
	//Set gyro full scale select to 1000 dps
	R = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, GYRO_CONFIG, 1, &R, 1, 100);
	
	//Set accelerator full scale select to 4g
	R = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, ACCEL_ADDRESS, ACCEL_CONFIG, 1, &R, 1, 100);
	
	//Enable interrupt on INT pin
	R = 0x22;
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, INT_PIN_CFG, 1, &R, 1, 100);
	R = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, INT_ENABLE, 1, &R, 1, 100);
	
	//INT_STATUS Register check
	uint8_t INT_status = 0;
	HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, INT_STATUS, 1, &INT_status, 1, 100);
	HAL_Delay(10);

	//WHO_I_AM check
	if (MPU9250_Check() == 0)
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
}

//WHO_I_AM check
int MPU9250_Check(void) 
{
		uint8_t WhoAmI = 0x00;
		HAL_I2C_Mem_Read(&hi2c1, DEFAULT_ADDRESS, WHO_AM_I, 1, &WhoAmI, 1, 100);
   	if(WHO_AM_I_VAL == WhoAmI)  
   	{
   		return 1;
   	}
   	else 
   	{
   		return 0;
   	}	
}

//Read data from accelerator
void MPU9250_READ_ACCEL(void)//code execution time ~0.6 ms
{ 
   uint8_t i;
	 int16_t InBuffer[3] = {0}; 
	 static int32_t OutBuffer[3] = {0};
	 static MPU9250_AvgTypeDef MPU9250_Filter[3];

	 //read accel  x, y, z axis from MPU registers
	 HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS, ACCEL_XOUT_H, 1, &BUF[0], 6, 10);
   InBuffer[0]=	(BUF[0]<<8)|BUF[1];
   InBuffer[1]=	(BUF[2]<<8)|BUF[3];   
   InBuffer[2]=	(BUF[4]<<8)|BUF[5];			       
   
   for(i = 0; i < 3; i ++)	
   {
      MPU9250_CalAvgValue(&MPU9250_Filter[i].Index, MPU9250_Filter[i].AvgBuffer, InBuffer[i], OutBuffer + i);
   }
	 
   accel[0] = *(OutBuffer + 0);
   accel[1] = *(OutBuffer + 1);
   accel[2] = *(OutBuffer + 2); 
}

//Read data from gyroscope
void MPU9250_READ_GYRO(void)//code execution time ~0.6 ms
{ 
   uint8_t i;
	 int16_t InBuffer[3] = {0}; 
	 static int32_t OutBuffer[3] = {0};
	 static MPU9250_AvgTypeDef MPU9250_Filter[3];
	 
	 //read accel  x, y, z axis from MPU registers
	 HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, GYRO_XOUT_H, 1, &BUF[0], 6, 10);
   InBuffer[0]=	(BUF[0]<<8)|BUF[1];
   InBuffer[1]=	(BUF[2]<<8)|BUF[3];	
   InBuffer[2]=	(BUF[4]<<8)|BUF[5];	

   for(i = 0; i < 3; i ++)	
   {
      MPU9250_CalAvgValue(&MPU9250_Filter[i].Index, MPU9250_Filter[i].AvgBuffer, InBuffer[i], OutBuffer + i);
   }
	 
   gyro[0] = *(OutBuffer + 0) - MPU9250_Offset.X;
   gyro[1] = *(OutBuffer + 1) - MPU9250_Offset.Y;
   gyro[2] = *(OutBuffer + 2) - MPU9250_Offset.Z;
}


void MPU9250_CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{	
	uint8_t i;
	
	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
  	*pIndex &= 0x07;
  	
  	*pOutVal = 0;
	for(i = 0; i < 8; i ++) 
  	{
    	*pOutVal += *(pAvgBuffer + i);
  	}
  	*pOutVal >>= 3;
}


