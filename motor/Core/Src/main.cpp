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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsp.hpp"
#include "bsp_dwt.hpp"
#include "DJIMotorHandler.hpp"
#include "GM6020.hpp"

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
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//创建motorhandler实例，使这个项目中始终只有一个motorhandler，不另外多次创建实例
DJIMotorHandler *motor_handler = DJIMotorHandler::Instance();
//创建一个电机6020
GM6020 motor6020;

//下面是两个电机跟踪目标曲线
static double signal(double t)
{
  constexpr double T = 0.6;           // 周期

  constexpr double buffer_ratio = 1.0 / 5.0;
  constexpr double rise_ratio = 4.0 / 5.0;

  const double t_mod = std::fmod(t, T);

  constexpr double buffer_time = buffer_ratio * T;
  constexpr double rise_time = rise_ratio * T;

  if (t_mod < buffer_time)
  {
    // 前1/5缓冲段
    return 0.0;
  }
  else if (t_mod < T)
  {
    double step_value = 0.1f;

    const double rise_t = t_mod - buffer_time;
    const double progress = rise_t / rise_time; // 0 ~ 1

    const double smooth = (1 - std::cos(Math::Pi * progress)) / 2.0;

    return step_value * smooth;
  }
  else
  {
    return 0.0;
  }
}

static double tri_signal(double t)
{
  constexpr double T = 0.6;           // 周期

  constexpr double buffer_ratio = 1.0 / 5.0;
  constexpr double rise_ratio = 4.0 / 5.0;

  const double t_mod = std::fmod(t, T);

  constexpr double buffer_time = buffer_ratio * T;
  constexpr double rise_time = rise_ratio * T;

  if (t_mod < buffer_time)
  {
    // 前1/5缓冲段
    return 0.0;
  }
  else if (t_mod < T)
  {
    double step_value = 0.1f;
    // 后4/5 线性上升段（三角波）
    const double rise_t = t_mod - buffer_time;
    const double progress = rise_t / rise_time; // 0 ~ 1

    // 线性
    const double smooth = progress;

    return step_value * smooth;
  }
  else
  {
    return 0.0;
  }
}

static double sin_signal(double t)
{
  const double amplitude = 0.03;  // 振幅（小幅度）
  double T = 0.2f;          // 周期（快速振动）
  const double omega = 2.0 * M_PI / T; // 角频率 ω = 2π / T

  return amplitude * std::sin(omega * t);
}

float dbg_signal = 0.0f;
float dbg_pos_fdb = 0.0f;
float dbg_spd_fdb = 0.0f;
float dbg_pos_ref = 0.0f;
float dbg_spd_ref = 0.0f;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //在系统init后，首先初始化can，完成过滤等等配置
  bsp_Init();
  //然后把电机注册到motorhandler里
  motor_handler->registerMotor(&motor6020, &hfdcan1, 0x205);
  //motor mode: position
  motor6020.controlMode = DJIMotor::POS_MODE;
  //for safety
  motor6020.currentSet = 0;

  //set and modify PID paras here
  motor6020.positionPid.kp = 30.0f;
  motor6020.positionPid.ki = 0.0f;
  motor6020.positionPid.kd = 0.0f;
  motor6020.speedPid.kp = 400.0f;
  motor6020.speedPid.ki = 0.0f;
  motor6020.speedPid.kd = 0.0f;

  //init original position to current position
  static float offset = motor6020.motorFeedback.positionFdb;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    motor6020.positionPid.ref = signal(DWT_GetTimeline_s())+offset;
    motor6020.positionPid.fdb = motor6020.motorFeedback.positionFdb;
    motor6020.positionPid.UpdateResult();
    motor6020.speedPid.ref = motor6020.positionPid.result;
    motor6020.speedPid.fdb = motor6020.motorFeedback.speedFdb;
    motor6020.speedPid.UpdateResult();
    motor6020.currentSet = motor6020.speedPid.result;

    dbg_signal = motor6020.positionPid.ref;
    dbg_pos_fdb = motor6020.positionPid.fdb;
    dbg_spd_fdb = motor6020.speedPid.fdb;
    dbg_pos_ref = motor6020.positionPid.ref;
    dbg_spd_ref = motor6020.speedPid.ref;

    motor_handler->sendControlData();

    HAL_Delay(1);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
