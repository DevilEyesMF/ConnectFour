/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum: uint8_t
{
    RED = 0,
    YELLOW = 1
} Color;

typedef enum: uint8_t
{
    LEFT = 0b0001,
    MID = 0b0010,
    RIGHT = 0b0100,
    RESTART = 0b1000
} Button;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_DRIVER_ADDRESS 0x8a
#define NUMBER_OF_ROWS 7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
uint8_t matrix[2][NUMBER_OF_ROWS];

const uint16_t ROW[] = {
        ROW0_Pin,
        ROW1_Pin,
        ROW2_Pin,
        ROW3_Pin,
        ROW4_Pin,
        ROW5_Pin,
        ROW6_Pin
};

uint8_t buttonFlags = 0; // 0000 SRML (restart, right, mid, left)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void heartScroll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t matrix_buffer[2][NUMBER_OF_ROWS] = {0};
    Color currentPlayer = RED;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    __enable_irq();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim14);

    matrix_buffer[currentPlayer][0] = 0b00001000;

    heartScroll();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1)
    {
        // LEFT BUTTON PRESSED
        if (buttonFlags & LEFT)
        {
            // shift the LED to the left
            matrix_buffer[currentPlayer][0] <<= 1;
            // account for the 8th bit
            if(matrix_buffer[currentPlayer][0] >= 0b10000000) matrix_buffer[currentPlayer][0] = 0b01000000;
            // reset flag
            buttonFlags ^= LEFT;
        }

        // RIGHT BUTTON PRESSED
        if (buttonFlags & RIGHT)
        {
            // shift the LED to the left
            matrix_buffer[currentPlayer][0] >>= 1;
            // account for the 8th bit
            if(matrix_buffer[currentPlayer][0] == 0) matrix_buffer[currentPlayer][0] = 0b00000001;
            // reset flag
            buttonFlags ^= RIGHT;
        }

        // MIDDLE BUTTON PRESSED
        if (buttonFlags & MID)
        {
            // TODO middle button
            // switch player
            matrix_buffer[currentPlayer][0] = 0; //temp
            currentPlayer ^= 1;
            matrix_buffer[currentPlayer][0] = 0b00001000;
            // reset flag
            buttonFlags ^= MID;
        }

        // RESTART BUTTON PRESSED
        if(buttonFlags & RESTART)
        {
            heartScroll();
            memset(matrix_buffer, 0, sizeof(matrix_buffer));
            // RED starts the game
            currentPlayer = RED;
            matrix_buffer[currentPlayer][0] = 0b00001000;
            // reset flag
            buttonFlags ^= RESTART;
        }

        memcpy(matrix, matrix_buffer, sizeof(matrix)); // send the new frame
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
#pragma clang diagnostic pop
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 6400 - 1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 5 - 1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_YEL_GPIO_Port, EN_YEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin
                          |ROW4_Pin|ROW5_Pin|ROW6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_RED_GPIO_Port, EN_RED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : EN_YEL_Pin */
  GPIO_InitStruct.Pin = EN_YEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_YEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_LEFT_Pin BTN_MID_Pin BTN_RIGHT_Pin BTN_RESTART_Pin */
  GPIO_InitStruct.Pin = BTN_LEFT_Pin|BTN_MID_Pin|BTN_RIGHT_Pin|BTN_RESTART_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW0_Pin ROW1_Pin ROW2_Pin ROW3_Pin
                           ROW4_Pin ROW5_Pin ROW6_Pin */
  GPIO_InitStruct.Pin = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin
                          |ROW4_Pin|ROW5_Pin|ROW6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_RED_Pin */
  GPIO_InitStruct.Pin = EN_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_RED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM14)
    {
        static int row = 0;
        static int prevRow = 0;
        static Color color = RED;

        HAL_GPIO_WritePin(GPIOA, ROW[prevRow], GPIO_PIN_SET);
        if (color == RED)
        {
            HAL_GPIO_WritePin(EN_YEL_GPIO_Port, EN_YEL_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(EN_RED_GPIO_Port, EN_RED_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(EN_RED_GPIO_Port, EN_RED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(EN_YEL_GPIO_Port, EN_YEL_Pin, GPIO_PIN_SET);
        }
        HAL_I2C_Mem_Write(&hi2c1, LED_DRIVER_ADDRESS, 0x02, 1,
                          &matrix[color][row], sizeof(matrix[color][row]), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, ROW[row], GPIO_PIN_RESET);

        if(color == YELLOW)
        {
            prevRow = row;
            if (++row == NUMBER_OF_ROWS) row = 0; // reset the row counter
        }

        color ^= 1; // switches between red and yellow
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BTN_RESTART_Pin)
    {
        buttonFlags |= RESTART;
    }

    if (GPIO_Pin == BTN_LEFT_Pin)
    {
        buttonFlags |= LEFT;
    }

    if (GPIO_Pin == BTN_RIGHT_Pin)
    {
        buttonFlags |= RIGHT;
    }

    if (GPIO_Pin == BTN_MID_Pin)
    {
        buttonFlags |= MID;
    }
}

void heartScroll()
{
    uint8_t matrix_buffer[2][NUMBER_OF_ROWS];

    static const uint8_t heart[2][NUMBER_OF_ROWS] = {
            {
                    0b00000000,
                    0b00110110,
                    0b01001001,
                    0b01000001,
                    0b00100010,
                    0b00010100,
                    0b00001000
            },
            {
                    0b00000000,
                    0b00000000,
                    0b00110110,
                    0b00111110,
                    0b00011100,
                    0b00001000,
                    0b00000000
            }
    };

    for (int i = -7; i <= 7; i++)
    {
        for (int row = 0; row < NUMBER_OF_ROWS; row++)
        {
            if (i < 0)
            {
                matrix_buffer[RED][row] = (uint8_t)(heart[RED][row] << -i);
                matrix_buffer[YELLOW][row] = (uint8_t)(heart[YELLOW][row] << -i);
            }
            else
            {
                matrix_buffer[RED][row] = heart[RED][row] >> i;
                matrix_buffer[YELLOW][row] = heart[YELLOW][row] >> i;
            }
        }
        memcpy(matrix, matrix_buffer, sizeof(matrix));
        HAL_Delay(250);
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
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1)
    {
        //
    }
#pragma clang diagnostic pop
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
