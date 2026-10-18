/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MyMPU6050.h"
#include <stdio.h>
#include <math.h>
uint16_t print_cnt = 0;

int16_t acc_raw[3], gyro_raw[3], temp_raw;
float   acc[3], gyro[3], temp;
int16_t bias[3];
#define RAD_TO_DEG      57.2957795f     /* 180/π，弧度转角度 */
//下面是计算互补滤波的变量
#define COMP_ALPHA      0.98f       /* 互补滤波系数 */

static float    s_roll      = 0.0f;     /* 融合后的 roll */
static float    s_pitch     = 0.0f;     /* 融合后的 pitch */
static float    s_yaw       = 0.0f;     /* 融合后的 yaw */
static uint32_t s_last_tick = 0;        /* 上次积分时刻 */

static volatile uint32_t s_int_count = 0;

static volatile uint8_t  s_data_ready = 0;
static volatile uint32_t s_prev_tick  = 0;
static volatile uint32_t s_dt_min     = 9999;
static volatile uint32_t s_dt_max     = 0;
static volatile uint32_t s_dt_sum     = 0;
static volatile uint32_t s_dt_cnt     = 0;

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//为了能输出数据给VOFA+,必须把浮点数转化为保留小数点后一位的数据
static void PrintFloat1(float v)
{
    int32_t n;
    if (v >= 0.0f)
    {
        n = (int32_t)(v * 10.0f + 0.5f);
    }
    else
    {
        n = (int32_t)(v * 10.0f - 0.5f);
    }
    if (n < 0)
    {
        printf("-");
        n = -n;
    }
    printf("%ld.%ld", n / 10, n % 10);
}

//用加速度求roll，pitch,静态时准确;动态时会受干扰
static void CalcAngleFromAccel(float ax, float ay, float az,
                               float *roll, float *pitch)
{
    *roll  = atan2f(ay, az) * RAD_TO_DEG;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

//角度=α×(角度+陀螺×Δt)+(1−α)×加速度角度
static void Attitude_Update(float ax, float ay, float az,
                            float gx, float gy, float gz)
{
    uint32_t now;
    float    dt;
    float    roll_acc, pitch_acc;

    now = HAL_GetTick();

	if (s_last_tick == 0)//第一次不能使用
    {
        s_last_tick = now;
        return;
    }

    dt = (float)(now - s_last_tick) / 1000.0f;
    s_last_tick = now;

    if (dt > 0.5f) { dt = 0.5f; }
    if (dt <= 0.0f) { return; }
    CalcAngleFromAccel(ax, ay, az, &roll_acc, &pitch_acc);
    s_roll  += gx * dt;
    s_pitch += gy * dt;
    s_yaw   += gz * dt;
    s_roll  = COMP_ALPHA * s_roll  + (1.0f - COMP_ALPHA) * roll_acc;
    s_pitch = COMP_ALPHA * s_pitch + (1.0f - COMP_ALPHA) * pitch_acc;
}

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("\r\n\r\n");
	printf("========================================\r\n");
	printf("  MPU6050 Test - Exam 2\r\n");
	printf("========================================\r\n");

	printf("ID = 0x%02X\r\n", MyMPU6050_GetID());

	if (MyMPU6050_Init() != 0)
	{
    printf("Init FAILED!\r\n");
    while (1) { }
	}

	printf("Init OK\r\n");
	
	MyMPU6050_EnableDataReadyInt();

	printf("Keep the board STILL, calibrating...\r\n");
	HAL_Delay(1000);

	MyMPU6050_CalibrateGyro(1000);
	MyMPU6050_GetGyroBias(bias);
	printf("GYRO BIAS = %d, %d, %d\r\n", bias[0], bias[1], bias[2]);

	printf("Roll(deg),Pitch(deg),Yaw(deg)\n");
	
	uint32_t loop_cnt = 0;
	uint32_t t_print;
	uint16_t print_cnt = 0;
	t_print = HAL_GetTick();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (s_data_ready != 0)
    {
        s_data_ready = 0;

        MyMPU6050_ReadPhysical(acc, gyro, &temp);
        Attitude_Update(acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);

        
        print_cnt++;
        if (print_cnt >= 2)
        {
            print_cnt = 0;

            PrintFloat1(s_roll);   printf(",");
            PrintFloat1(s_pitch);  printf(",");
            PrintFloat1(s_yaw);    printf("\n");
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now;
    uint32_t d;

    if (GPIO_Pin == MPU_INT_Pin)
    {
        now = HAL_GetTick();

        if (s_prev_tick != 0)
        {
            d = now - s_prev_tick;

            if (d < s_dt_min) { s_dt_min = d; }
            if (d > s_dt_max) { s_dt_max = d; }

            s_dt_sum += d;
            s_dt_cnt++;
        }
        s_prev_tick = now;

        s_data_ready = 1;
    }
}
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

#ifdef  USE_FULL_ASSERT
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
