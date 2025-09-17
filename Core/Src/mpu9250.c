#include "mpu9250.h"
#include <string.h>

void MPU9250_Init(void) {
    uint8_t check, data;

    // WHO_AM_I 확인 (0x75)
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x75, 1, &check, 1, 1000);
    if (check == 0x71) {  // MPU9250 WHOAMI = 0x71
        // 슬립 해제
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x6B, 1, &data, 1, 1000);

        // 가속도 ±2g
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x1C, 1, &data, 1, 1000);

        // 자이로 ±250 dps
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x1B, 1, &data, 1, 1000);

        // I2C Bypass Enable (자력계 직접 접근 허용)
        data = 0x02;
        HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x37, 1, &data, 1, 1000);
    }
}

void MPU9250_Read_Accel_Gyro(int16_t* AccelGyroData) {
    uint8_t Rec_Data[14];
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x3B, 1, Rec_Data, 14, 1000);

    // 가속도
    AccelGyroData[0] = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);  // X
    AccelGyroData[1] = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);  // Y
    AccelGyroData[2] = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);  // Z

    // 자이로
    AccelGyroData[3] = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);   // X
    AccelGyroData[4] = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]); // Y
    AccelGyroData[5] = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]); // Z
}

void AK8963_Init(void) {
    uint8_t data;

    // Power down mode
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, 1000);
    HAL_Delay(10);

    // Fuse ROM access mode
    data = 0x0F;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, 1000);
    HAL_Delay(10);

    // 다시 Power down
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, 1000);
    HAL_Delay(10);

    // Continuous measurement mode 2 (100Hz, 16-bit)
    data = 0x16;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, 1000);
    HAL_Delay(10);
}

void MPU9250_Read_Mag(int16_t* MagData) {
    uint8_t Rec_Data[7];

    // 데이터 읽기 (ST1 + HXL..HZH + ST2)
    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, 0x02, 1, Rec_Data, 7, 1000);

    // X, Y, Z (리틀엔디안 → Low, High 순서)
    MagData[0] = (int16_t)(Rec_Data[1] << 8 | Rec_Data[0]);
    MagData[1] = (int16_t)(Rec_Data[3] << 8 | Rec_Data[2]);
    MagData[2] = (int16_t)(Rec_Data[5] << 8 | Rec_Data[4]);
}
