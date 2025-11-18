/*
 * mpu6050.h
 *
 *  Created on: Sep 18, 2025
 *      Author: Girish
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#define MPU_ADDR (0x68<<1)
#define SENS_ACC_8G 4096.0f
#define SENS_GYRO_500DPS 65.5
#define MPU_INT_PIN GPIO_PIN_10
#define MPU_INT_PORT GPIOC
#define INTR_ENABLED 0UL
typedef struct{
	float acc_x;
	float acc_y;
	float acc_z;
	float gyr_x;
	float gyr_y;
	float gyr_z;
} MPU_Data_t;

void mpu6050_init(void);
MPU_Data_t MPU6050_read_all(void);
void Error_Handler();

#endif /* INC_MPU6050_H_ */
