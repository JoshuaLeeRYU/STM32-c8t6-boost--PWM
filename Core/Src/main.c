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
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "INA219.h"
#include "font.h"
#include "oled.h"
#include "stdlib.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DUTY_MAX 0.7
#define DUTY_MIN 0.0
#define UMIN 5.0
#define UMAX 16.0
#define IMIN 0.0
#define IMAX 0.4
#define XZMAX 10.0
#define DELTA_U 0.01 // 误差
#define DELTA_I -0.006
#define vkp 0.5
#define vki 4
#define akp 3
#define aki 500
#define I_HYST 0.01f
#define U_HYST 0.05f
#define CURRENT_LPF 0.2f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float targetv = 10.0, targeta = 0.1;
volatile float voltage = 0.0;
volatile float current = 0.0;
volatile int key = 0;
volatile int clcflag = 0;
volatile int showoled = 0;
float vP = 0, vI = 0, vintegral = 0;
float aP = 0, aI = 0, aintegral = 0;
float vduty = 0.02, aduty = 0.0;
float targeta1, targetv1, xz1;
float b = 0.0;
float xz = 0;
float INA219_voltage = 0, INA219_current = 0;
float current_f = 0.0f;
float voltage_f = 0.0f;
char V[20], A[20], MODE[20], XZ[20];
char a0[3] = {'='}, a1[3] = {'<'};
int d = 0;
int page = 0;
int counter = 0;
int inited = 0;
int mode = 0;
int ready = 0;
int i = 0;
int limit_active = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static uint32_t last_tick = 0;
  uint32_t now = HAL_GetTick();
  if (now - last_tick > 20) // 20ms 内只算一次，消抖
  {
    last_tick = now;
    key++;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim3) {
    showoled = 1;
    // TIM3 显示标志
  }
  if (htim == &htim2) {
    clcflag = 1;
    // TIM2 控制标志
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_NewFrame();
  OLED_DrawImage(0, 0, &MeguminImg, OLED_COLOR_NORMAL);
  OLED_PrintString(64, 8, "可调电源", &font16x16, OLED_COLOR_NORMAL);
  OLED_PrintString(64, 24, "0-0.4A", &font16x16, OLED_COLOR_NORMAL);
  OLED_PrintString(64, 40, "5-16V", &font16x16, OLED_COLOR_NORMAL); // 启动画面
  OLED_ShowFrame();

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3); // stm32各种初始化
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,
                        (uint32_t)(__HAL_TIM_GET_AUTORELOAD(&htim4) * b));

  int ina219_err = INA219_Init();
  HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if (ina219_err == 1) {
      OLED_NewFrame();
      OLED_PrintString(0, 24, "INA219 Init Failed", &font16x16,
                       OLED_COLOR_NORMAL);
      OLED_ShowFrame();
      ina219_err = INA219_Init();
      continue; // INA219初始化失败，跳过本循环,且重新初始化
    }

    INA219_voltage = INA219_ReadBusVoltage() / 1000.0f;
    INA219_current = (INA219_ReadCurrent() * 0.4 / 32768); // 电流电压计算

    if (INA219_voltage == 65535 / 1000.0f ||
        INA219_current == 65535 / 1000.0f) {
      OLED_NewFrame();
      OLED_PrintString(0, 0, "ERR", &font16x16, OLED_COLOR_NORMAL);
      OLED_ShowFrame();
      i++;
      if (i > 1000) {
        i = 0;
        ina219_err = INA219_Init();
      }
      continue; // 读取失败，超过10次重新尝试初始化，跳过本循环
    }
    current = INA219_current + DELTA_I;
    voltage = INA219_voltage - current * xz +
              DELTA_U; // 电压电流赋值，线阻压降和INA219的误差补偿

    static uint32_t stable_start = 0;
    if (fabsf(voltage - targetv) < 0.02) {
      if (stable_start == 0) {
        stable_start = HAL_GetTick();
      } else if (HAL_GetTick() - stable_start >= 100) {
        ready = 1;
      }
    } else {
      stable_start = 0;
    }

    if (clcflag == 1) {
      clcflag = 0;

      const float dt = 0.001f;

      vP = vkp * (targetv - voltage);
      aP = akp * (targeta - current);

      // 迟滞判断
      if (mode == 0) { // 恒压限流
        if (!limit_active) {
          if (current > targeta + I_HYST)
            limit_active = 1;
        } else {
          if (current < targeta - I_HYST)
            limit_active = 0;
        }
      } else { // 恒流限压
        if (!limit_active) {
          if (voltage > targetv + U_HYST)
            limit_active = 1;
        } else {
          if (voltage < targetv - U_HYST)
            limit_active = 0;
        }
      }

      if (mode == 0) {
        if (!limit_active) {
          // 电压环工作，电流环跟踪 b
          vintegral += (targetv - voltage) * dt;

          float vint_max = (DUTY_MAX - vP) / vki;
          float vint_min = (DUTY_MIN - vP) / vki;
          if (vintegral > vint_max)
            vintegral = vint_max;
          if (vintegral < vint_min)
            vintegral = vint_min;

          vduty = vP + vki * vintegral;
          b = vduty;

          // 电流环无扰跟踪
          aintegral = (b - aP) / aki;
          aduty = b;
        } else {
          // 电流环限流，电压环跟踪 b
          aintegral += (targeta - current) * dt;

          float aint_max = (DUTY_MAX - aP) / aki;
          float aint_min = (DUTY_MIN - aP) / aki;
          if (aintegral > aint_max)
            aintegral = aint_max;
          if (aintegral < aint_min)
            aintegral = aint_min;

          aduty = aP + aki * aintegral;
          b = aduty;

          // 电压环无扰跟踪
          vintegral = (b - vP) / vki;
          vduty = b;
        }
      } else { // mode == 1 恒流限压
        if (!limit_active) {
          // 电流环工作，电压环跟踪 b
          aintegral += (targeta - current) * dt;

          float aint_max = (DUTY_MAX - aP) / aki;
          float aint_min = (DUTY_MIN - aP) / aki;
          if (aintegral > aint_max)
            aintegral = aint_max;
          if (aintegral < aint_min)
            aintegral = aint_min;

          aduty = aP + aki * aintegral;
          b = aduty;

          vintegral = (b - vP) / vki;
          vduty = b;
        } else {
          // 电压环限压，电流环跟踪 b
          vintegral += (targetv - voltage) * dt;

          float vint_max = (DUTY_MAX - vP) / vki;
          float vint_min = (DUTY_MIN - vP) / vki;
          if (vintegral > vint_max)
            vintegral = vint_max;
          if (vintegral < vint_min)
            vintegral = vint_min;

          vduty = vP + vki * vintegral;
          b = vduty;

          aintegral = (b - aP) / aki;
          aduty = b;
        }
      }

      if (b > DUTY_MAX)
        b = DUTY_MAX;
      if (b < DUTY_MIN)
        b = DUTY_MIN;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,
                          (uint32_t)(__HAL_TIM_GET_AUTORELOAD(&htim4) * b));

    counter = (__HAL_TIM_GET_COUNTER(&htim1) / 2); // 编码器计数
    if (key == 0)
      page = ((counter % 4) + 4) % 4; // 第几页 防负数

    if (showoled == 1 && ready == 1) // oled显示及调整模式
    {
      showoled = 0;
      OLED_NewFrame();

      if (key == 1) // 调整模式及UI（非阻塞）
      {
        if (inited == 0) {
          TIM1->CNT = 32768; // 初始化
          inited++;
          counter = (__HAL_TIM_GET_COUNTER(&htim1) / 2);
          targeta1 = targeta;
          targetv1 = targetv;
          xz1 = xz;
        }
        counter = (__HAL_TIM_GET_COUNTER(&htim1) / 2);
        d = (16384 - counter);
        switch (page) {
        case 0:
          targetv = targetv1 + d * 0.05; // V
          if (targetv > UMAX)
            targetv = UMAX;
          if (targetv < UMIN)
            targetv = UMIN;
          break;
        case 1:
          targeta = targeta1 + d * 0.01; // A
          if (targeta > IMAX)
            targeta = IMAX;
          if (targeta < IMIN)
            targeta = IMIN;
          break;
        case 2:
          xz = xz1 + d * 0.1; // 线阻
          if (xz < 0)
            xz = 0;
          if (xz > XZMAX)
            xz = XZMAX;
          break;
        case 3:
          vduty = b;
          aduty = b;
          if (mode == 0) {
            mode = 1;
            aintegral = (b - aP) / aki; // 积分过渡换算
            strcpy(a0, "<");
            strcpy(a1, "=");
          } else {
            mode = 0;
            vintegral = (b - vP) / vki;
            strcpy(a0, "=");
            strcpy(a1, "<");
          }
          inited = 0;
          key = 2;
          break;
        }
      }
      if (key >= 2) {
        TIM1->CNT = 0;
        key = 0;
        inited = 0; // 退出调整模式
      }
      sprintf(V, "电压:%.2f%s%.2f", voltage, a0, targetv);
      OLED_PrintString(0, 0, V, &font16x16, page == 0);
      if (current < 0)
        current = 0;
      sprintf(A, "电流:%.2f%s%.2f", current, a1, targeta);
      OLED_PrintString(0, 16, A, &font16x16, page == 1);
      sprintf(XZ, "线阻Ω:%.1f", xz);
      OLED_PrintString(0, 32, XZ, &font16x16, page == 2);
      if (mode == 0) {
        sprintf(MODE, "模式:恒压限流");
      } else
        sprintf(MODE, "模式:恒流限压");
      OLED_PrintString(0, 48, MODE, &font16x16, page == 3);
      OLED_ShowFrame(); // OLED显示
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
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
