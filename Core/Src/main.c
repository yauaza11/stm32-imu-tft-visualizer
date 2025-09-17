/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <mpu9250.h>
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t AccelGyroData[6];
int16_t MagData[3];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  MPU9250_Init();
  AK8963_Init();

  ILI9341_Init();
  ILI9341_SetRotation(SCREEN_HORIZONTAL_1);  // 가로 모드 설정
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    MPU9250_Read_Accel_Gyro(AccelGyroData);
    MPU9250_Read_Mag(MagData);

    // 변환 (±2g 설정 → 16384 LSB = 1g)
    float ax_g = (float)AccelGyroData[0] / 16384.0f;
    float ay_g = (float)AccelGyroData[1] / 16384.0f;
    float az_g = (float)AccelGyroData[2] / 16384.0f;

    // Pitch, Roll 계산 (단위: 라디안 → degree 변환)
    float pitch = atan2f(ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / M_PI;
    float roll  = atan2f(ay_g, sqrtf(ax_g*ax_g + az_g*az_g)) * 180.0f / M_PI;

    // 자력계 원시값
    float mx = (float)MagData[0];
    float my = (float)MagData[1];
    float mz = (float)MagData[2];

    // Pitch, Roll 값 (라디안으로 변환)
    float pitchRad = pitch * M_PI / 180.0f;
    float rollRad  = roll  * M_PI / 180.0f;

    // Tilt 보정
    float mx_comp = mx * cosf(pitchRad) + mz * sinf(pitchRad);
    float my_comp = mx * sinf(rollRad) * sinf(pitchRad)
                  + my * cosf(rollRad)
                  - mz * sinf(rollRad) * cosf(pitchRad);

    // Yaw 계산 (단위: 도)
    float yaw = atan2f(-my_comp, mx_comp) * 180.0f / M_PI;
    if (yaw < 0) yaw += 360.0f;  // 0~360° 범위로 보정



    // ---------------------- LCD 출력 ---------------------------------

    char buf[30];
    ILI9341_FillScreen(BLACK); // 새로 그리기 전에 화면 지움


    sprintf(buf, "Pitch: %.2f deg", pitch);
    ILI9341_DrawText(buf, Arial_Narrow12x16, 10, 120, WHITE, BLACK);

    sprintf(buf, "Roll : %.2f deg", roll);
    ILI9341_DrawText(buf, Arial_Narrow12x16, 10, 140, WHITE, BLACK);

    sprintf(buf, "Yaw  : %.2f deg", yaw);
	ILI9341_DrawText(buf, Arial_Narrow12x16, 10, 160, WHITE, BLACK);

    // 숫자 출력 ----------------------
    sprintf(buf, "AX: %.2f g", ax_g);
    ILI9341_DrawText(buf, Arial_Narrow12x16, 10, 10, WHITE, BLACK);

    sprintf(buf, "AY: %.2f g", ay_g);
    ILI9341_DrawText(buf, Arial_Narrow12x16, 10, 30, WHITE, BLACK);

    sprintf(buf, "AZ: %.2f g", az_g);
    ILI9341_DrawText(buf, Arial_Narrow12x16, 10, 50, WHITE, BLACK);

    // 그래프 출력 ----------------------
    // 기준점
    int baseX = 180;  // 그래프 시작 X (화면 가운데)
    int startY = 10; // Y 위치

    // 스케일링 (±2g → ±100픽셀)
    int scale = 50; // 1g = 50픽셀
    int bar_ax = (int)(ax_g * scale);
    int bar_ay = (int)(ay_g * scale);
    int bar_az = (int)(az_g * scale);

    // X축 그래프 (빨강)
    if (bar_ax >= 0)
        ILI9341_DrawFilledRectangleCoord(baseX, startY, baseX + bar_ax, startY + 10, RED);
    else
        ILI9341_DrawFilledRectangleCoord(baseX + bar_ax, startY, baseX, startY + 10, RED);

    // Y축 그래프 (초록)
    if (bar_ay >= 0)
        ILI9341_DrawFilledRectangleCoord(baseX, startY + 20, baseX + bar_ay, startY + 30, GREEN);
    else
        ILI9341_DrawFilledRectangleCoord(baseX + bar_ay, startY + 20, baseX, startY + 30, GREEN);

    // Z축 그래프 (파랑)
    if (bar_az >= 0)
        ILI9341_DrawFilledRectangleCoord(baseX, startY + 40, baseX + bar_az, startY + 50, BLUE);
    else
        ILI9341_DrawFilledRectangleCoord(baseX + bar_az, startY + 40, baseX, startY + 50, BLUE);

    HAL_Delay(5); // 10Hz
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
