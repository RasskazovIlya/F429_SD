#include "MPU9255.h"

void MPU9250_Init(void)
{
	uint8_t R, c;
  HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, PWR_MGMT_1, 1, 0x00, 1, 100);
	HAL_Delay(100);
	
//	R = 0x01;
//	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, PWR_MGMT_1, 1, &R, 1, 100);
//	
//	R = 0x03;
//	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, PWR_MGMT_1, 1, &R, 1, 100);
	
	R = 0x07;
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, SMPLRT_DIV, 1, &R, 1, 100);
	
	R = 0x06;
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, CONFIG, 1, &R, 1, 100);
	
	R = 0x010;
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, GYRO_CONFIG, 1, &R, 1, 100);
	
	R = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, ACCEL_CONFIG, 1, &R, 1, 100);
	
	//HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, GYRO_CONFIG, 1, &c, 6, 100);
	if (MPU9250_Check() == 0)
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
}

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

void MPU9250_READ_ACCEL(void)
{ 
   uint8_t i;
	 int16_t InBuffer[3] = {0}; 
	 static int32_t OutBuffer[3] = {0};
	 static MPU9250_AvgTypeDef MPU9250_Filter[3];

	 HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS, ACCEL_XOUT_L, 1, &BUF[0], 1, 100);
	 HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS, ACCEL_XOUT_H, 1, &BUF[1], 1, 100);
   //BUF[0] = I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_XOUT_L); 
   //BUF[1]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_XOUT_H);
   InBuffer[0]=	(BUF[1]<<8)|BUF[0];

	 HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS, ACCEL_YOUT_L, 1, &BUF[2], 1, 100);
	 HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS, ACCEL_YOUT_H, 1, &BUF[3], 1, 100);
//   BUF[2]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_YOUT_L);
//   BUF[3]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_YOUT_H);
   InBuffer[1]=	(BUF[3]<<8)|BUF[2];
					
	HAL_I2C_Mem_Read(&hi2c1,ACCEL_ADDRESS, ACCEL_ZOUT_L, 1, &BUF[4], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1,ACCEL_ADDRESS, ACCEL_ZOUT_H, 1, &BUF[5], 1, 100);   
//   BUF[4]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_ZOUT_L);
//   BUF[5]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_ZOUT_H);
   InBuffer[2]=	(BUF[5]<<8)|BUF[4];			       
   
   for(i = 0; i < 3; i ++)	
   {
      MPU9250_CalAvgValue(&MPU9250_Filter[i].Index, MPU9250_Filter[i].AvgBuffer, InBuffer[i], OutBuffer + i);
   }
   accel[0] = *(OutBuffer + 0);
   accel[1] = *(OutBuffer + 1);
   accel[2] = *(OutBuffer + 2); 
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