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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define  sw_limit1 GPIO_PIN_4  // define PIN4 of Port 'B' is limit switch at left endpoint as input.
#define  sw_limit2 GPIO_PIN_5  // define PIN5 of Port 'B' is limit switch at right endpoint as input.
#define  sw_extend GPIO_PIN_6 // define PIN6 of Port 'B' as expansion switch as input
#define  sw_shrink GPIO_PIN_7 // define PIN7 of Port 'B' as shrink switch as input


#define DIR_out GPIO_PIN_8 // define PIN8 of Port 'B' as direction pin to control clockwise/ anti clockwise direction.
#define PWM_out GPIO_PIN_9 // define PIN9 of Port 'B' as PWM Pin to generate pulse to control stepper motor.


/* USER CODE END PM */

TIM_HandleTypeDef htim1; // set timer to generate delay in pulse

/* USER CODE BEGIN PV */

uint16_t total_step;     // store total steps in actuator.
uint16_t mid_step = 0 ; // store the mid point in a actuator.


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); // declaration of system clock
static void MX_GPIO_Init(void); // declaration of GPIO pins
static void MX_TIM1_Init(void);  // declaration  TIMER



/* USER CODE BEGIN PFP */

void step_pulse(uint16_t us); // declaration of step pulse to control stepper motor speed
void homing(void);           // declaration of homing function to find mid point of actuator.


/* USER CODE END PFP */


int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); // Initialize GPIO pins
  MX_TIM1_Init(); // Initialize Timer

  HAL_TIM_Base_Start(&htim1); // start the timer
  homing();                   // function initialize the actuator to mid point

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  // check if only sw_extend pin is set to enter the loop

 while(HAL_GPIO_ReadPin(GPIOB, sw_extend) && !(HAL_GPIO_ReadPin(GPIOB, sw_shrink))&& !(HAL_GPIO_ReadPin(GPIOB, sw_limit2))){

	   HAL_GPIO_WritePin(GPIOB,DIR_out,RESET ); // To extend move the stepper to CLOCKWISE direction.
	   step_pulse(60);                          // pulse of 60us delay is generated
       }
   HAL_Delay(100); // delay is added to avoid a jitter.


   // check if only sw_shrink pin is set to enter the loop
 while(HAL_GPIO_ReadPin(GPIOB, sw_shrink) && !(HAL_GPIO_ReadPin(GPIOB, sw_extend)) && !(HAL_GPIO_ReadPin(GPIOB, sw_limit1))){

   	   HAL_GPIO_WritePin(GPIOB,DIR_out,SET ); // To shrink move the stepper to ANTICLOCKWISE direction.
   	   step_pulse(90);                        // pulse of 60us delay is generated
        }
 HAL_Delay(100);  // delay is added to avoid a jitter.


 // check if both sw_extend and sw_shrink pin is set to enter homing loop
 while(HAL_GPIO_ReadPin(GPIOB, sw_shrink) && HAL_GPIO_ReadPin(GPIOB, sw_extend)){
     homing();  // call homing function
      }

 HAL_Delay(100); // delay is added to avoid a jitter.

  }



  /* USER CODE END 3 */
}


// generates the PWM signal

void step_pulse(uint16_t us){

	HAL_GPIO_WritePin(GPIOB,PWM_out, SET); // set the PWM out high

	__HAL_TIM_SET_COUNTER(&htim1,0);          // Initially set counter to 0
	while(__HAL_TIM_GET_COUNTER(&htim1)<us); // count till us

	HAL_GPIO_WritePin(GPIOB,PWM_out, RESET); // set the PWM out to low

}


void homing(void){

	uint16_t count = 0; // Initialization of count

	while(!HAL_GPIO_ReadPin(GPIOB, sw_limit1)){   // move actuator till it reached sw_limi1.

		HAL_GPIO_WritePin(GPIOB,DIR_out,SET );    // Move stepper motor in ANTICLOCK.
		step_pulse(30);                           // set a pulse of 30 us
        }

    total_step = 0;                                //Total steps to be measured.

	while(!HAL_GPIO_ReadPin(GPIOB, sw_limit2)){    // move actuator till it reached sw_limi2.

			HAL_GPIO_WritePin(GPIOB,DIR_out,RESET ); // Move stepper motor in CLOCK
			step_pulse(30);                          // set a pulse of 30 us
			total_step++;                            // total step covered from endpoint to endpoint
		}


	mid_step = total_step/2;                  // calculate the mid point


	while(count <mid_step){                   // move actuator till it reached mid_step.

			HAL_GPIO_WritePin(GPIOB,DIR_out,SET ); // Move stepper motor in ANTICLOCK
			step_pulse(30);                        // set a pulse of 30 us
			count++;                               //count till it reach the mid point
		}


// Initialize systemclock function.

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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
