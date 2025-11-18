/*
 * mpu6050.c
 *
 *  Created on: Sep 18, 2025
 *      Author: Girish
 */


#include "main.h"
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

char *a = "Initialization Complete\r\n";
char *err = "MPU Error Occurred\r\n";
MPU_Data_t data;
void mpu6050_init(void){
	// Wake up by configuring the clock
	uint8_t wake_up_cmd = 0x00;
	if(HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 107, I2C_MEMADD_SIZE_8BIT, &wake_up_cmd, 1, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}
	//Configure the DLPF
	uint8_t dlpf_config = 0x01;
	if(HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 26, I2C_MEMADD_SIZE_8BIT, &dlpf_config, 1, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}
	//Configuring the Sample rate
	uint8_t samp_rate =255;
	if(HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 25, I2C_MEMADD_SIZE_8BIT, &samp_rate, 1, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}
	//Configuring the gyroscope - 500 dps
	uint8_t gyro_config =0x08;
	if(HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 27, I2C_MEMADD_SIZE_8BIT, &gyro_config, 1, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}
	//Configuring Accelerometer - 8g
	uint8_t acc_config =0x10;
	if(HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 28, I2C_MEMADD_SIZE_8BIT, &acc_config, 1, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}
#if(INTR_ENABLED == 1)
	//Configure Interrupt pin to be active low and push pull
	uint8_t intr_level= 0x80;
	if(HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 55, I2C_MEMADD_SIZE_8BIT, &intr_level, 1, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}
	//Enable the interrupt
	uint8_t intr_en = 0x01;
	if(HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 56, I2C_MEMADD_SIZE_8BIT, &intr_en, 1, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}
#endif

	HAL_UART_Transmit(&huart2, (uint8_t*)a, strlen(a), HAL_MAX_DELAY);

}


MPU_Data_t MPU6050_read_all(void){
	uint8_t reg[14]={0};
	//Read the Accelerometer XOUT_H values
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 59, I2C_MEMADD_SIZE_8BIT, reg, 14, HAL_MAX_DELAY);
	//ACC _X
	int16_t accx= (int16_t)((reg[0]<<8) | reg[1]);
	data.acc_x = accx/SENS_ACC_8G;
	//ACC _Y
	int16_t accy= (int16_t)((reg[2]<<8) | reg[3]);
	data.acc_y = accy/SENS_ACC_8G;
	//ACC _Z
	int16_t accz= (int16_t)((reg[4]<<8) | reg[5]);
	data.acc_z = accz/SENS_ACC_8G;
	//ACC _X
	int16_t gyrx= (int16_t)((reg[8]<<8) | reg[9]);
	data.gyr_x = gyrx/SENS_GYRO_500DPS;
	//ACC _Y
	int16_t gyry= (int16_t)((reg[10]<<8) | reg[11]);
	data.gyr_y = gyry/SENS_GYRO_500DPS;
	//ACC _Z
	int16_t gyrz= (int16_t)((reg[12]<<8) | reg[13]);
	data.gyr_z = gyrz/SENS_GYRO_500DPS;
	return data;
}






void Error_Handler(){
	HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
}
