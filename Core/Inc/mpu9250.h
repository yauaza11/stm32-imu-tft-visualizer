#ifndef __MPU9250_H_
#define __MPU9250_H_

#include "stm32f4xx_hal.h"

// AD0=GND → 주소 0x68, HAL은 8bit 주소 사용
#define MPU9250_ADDR (0x68<<1)
#define AK8963_ADDR  (0x0C << 1)
extern I2C_HandleTypeDef hi2c1; // CubeMX에서 생성된 I2C 핸들 사용

// 함수 선언
void MPU9250_Init(void);
void MPU9250_Read_Accel_Gyro(int16_t* AccelGyroData);
void AK8963_Init(void);
void MPU9250_Read_Mag(int16_t* MagData);

#endif /* MPU6050_H_ */
