/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#define ARM_MATH_CM4
#define __FPU_PRESENT				1U
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include <string.h>
#include "arm_math.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
#define PID_PARAM_KP				2000.0
#define PID_PARAM_KI				0.0
#define PID_PARAM_KD				20.0

#define CLOCK_SPEED					16000000
#define HIGH_ACCE						200    /* define highest motor speed, still need to 
																			be config and changed */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_TIM10_Init(int period, int pulse);
static void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void encoder_init(void);
void get_two_encoder(char* encoder_Data, int* encoder_data);
void ResetPWM(int speed);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	char encoder_Data[200];
	int encoder_data[2];
	int last_speed = 0;
	
		/* PID error */
	float pid_error;
	/* Duty cycle for PWM */
	int duty_speed = 0;
	/* ARM PID Instance, float_32 format */
	arm_pid_instance_f32 PID;
	
	/* Set PID parameters */
	/* Set this for your needs */
	PID.Kp = PID_PARAM_KP;		/* Proporcional */
	PID.Ki = PID_PARAM_KI;		/* Integral */
	PID.Kd = PID_PARAM_KD;		/* Derivative */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM10_Init(0, 0);
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	encoder_init();
	arm_pid_init_f32(&PID, 1);
	HAL_TIM_PWM_Init(&htim10);
	HAL_GPIO_WritePin(MOTOR_DIR_1_GPIO_Port, MOTOR_DIR_1_Pin, GPIO_PIN_SET);	//need an initial fuction for motor

	
	sprintf(encoder_Data, "test serial port first...");
	HAL_UART_Transmit_IT(&huart2, (uint8_t *)encoder_Data, strlen(encoder_Data));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		get_two_encoder(encoder_Data, encoder_data);
		
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)encoder_Data, strlen(encoder_Data));
		
		pid_error = encoder_data[1] - encoder_data[0];
		
		duty_speed = (int)(arm_pid_f32(&PID, pid_error));
		
		if(abs(duty_speed - last_speed) > HIGH_ACCE)
		{
			if(duty_speed > last_speed)
				duty_speed = last_speed + HIGH_ACCE;
			else
				duty_speed = last_speed - HIGH_ACCE;
		}
		
		if(duty_speed < 0)
			HAL_GPIO_WritePin(MOTOR_DIR_1_GPIO_Port, MOTOR_DIR_1_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(MOTOR_DIR_1_GPIO_Port, MOTOR_DIR_1_Pin, GPIO_PIN_SET);
		
		ResetPWM(duty_speed);
		
		last_speed = duty_speed;
		
		HAL_Delay(1);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM10 init function */
void MX_TIM10_Init(int period, int pulse)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = period;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim10);

  HAL_TIM_PWM_Init(&htim10);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_MspPostInit(&htim10);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin : ENCODER_DAT_2_Pin */
  GPIO_InitStruct.Pin = ENCODER_DAT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_DAT_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_CLK_2_Pin */
  GPIO_InitStruct.Pin = ENCODER_CLK_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENCODER_CLK_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_CSn_2_Pin */
  GPIO_InitStruct.Pin = ENCODER_CSn_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENCODER_CSn_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_DAT_1_Pin */
  GPIO_InitStruct.Pin = ENCODER_DAT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_DAT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_CLK_1_Pin ENCODER_CSn_1_Pin */
  GPIO_InitStruct.Pin = ENCODER_CLK_1_Pin|ENCODER_CSn_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_DIR_1_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_DIR_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_STEP_1_Pin MOTOR_EN_1_Pin */
  GPIO_InitStruct.Pin = MOTOR_STEP_1_Pin|MOTOR_EN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENCODER_CLK_2_GPIO_Port, ENCODER_CLK_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENCODER_CSn_2_GPIO_Port, ENCODER_CSn_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENCODER_CLK_1_Pin|ENCODER_CSn_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_DIR_1_GPIO_Port, MOTOR_DIR_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MOTOR_STEP_1_Pin|MOTOR_EN_1_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
void encoder_init(void)
{
	HAL_GPIO_WritePin(ENCODER_CSn_1_GPIO_Port, ENCODER_CSn_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENCODER_CSn_2_GPIO_Port, ENCODER_CSn_2_Pin, GPIO_PIN_SET);
	return ;
}

void get_two_encoder(char* encoder_Data, int* encoder_data)
{
	int i;
	uint16_t dataa_1 = 0;
	uint16_t dataa_2 = 0;
	uint16_t temp = 0;
	
	HAL_GPIO_WritePin(ENCODER_CSn_1_GPIO_Port, ENCODER_CSn_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ENCODER_CSn_2_GPIO_Port, ENCODER_CSn_2_Pin, GPIO_PIN_RESET);
	__ASM("NOP");
	__ASM("NOP");
	
	for(i=0; i<12; i++)
	{
		HAL_GPIO_WritePin(ENCODER_CLK_1_GPIO_Port, ENCODER_CLK_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENCODER_CLK_2_GPIO_Port, ENCODER_CLK_2_Pin, GPIO_PIN_RESET);
		
		__ASM("NOP"); __ASM("NOP");		
		__ASM("NOP"); __ASM("NOP");
		__ASM("NOP"); __ASM("NOP");		
//		__ASM("NOP"); __ASM("NOP");
		
		HAL_GPIO_WritePin(ENCODER_CLK_1_GPIO_Port, ENCODER_CLK_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ENCODER_CLK_2_GPIO_Port, ENCODER_CLK_2_Pin, GPIO_PIN_SET);
		
		__ASM("NOP"); __ASM("NOP");		
		__ASM("NOP"); __ASM("NOP");
		__ASM("NOP"); __ASM("NOP");		
//		__ASM("NOP"); __ASM("NOP");
		
		temp = HAL_GPIO_ReadPin(ENCODER_DAT_1_GPIO_Port, ENCODER_DAT_1_Pin);
		dataa_1 = (dataa_1 << 1) + temp;
		
		temp = HAL_GPIO_ReadPin(ENCODER_DAT_2_GPIO_Port, ENCODER_DAT_2_Pin);
		dataa_2 = (dataa_2 << 1) + temp;
		
		__ASM("NOP"); __ASM("NOP");
		__ASM("NOP"); __ASM("NOP");
//		__ASM("NOP"); __ASM("NOP");
	}
	
	for(i=0 ; i<4; i++)
	{
		HAL_GPIO_WritePin(ENCODER_CLK_1_GPIO_Port, ENCODER_CLK_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENCODER_CLK_2_GPIO_Port, ENCODER_CLK_2_Pin, GPIO_PIN_RESET);
		
		__ASM("NOP"); __ASM("NOP");		
		__ASM("NOP"); __ASM("NOP");
		
		HAL_GPIO_WritePin(ENCODER_CLK_1_GPIO_Port, ENCODER_CLK_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ENCODER_CLK_2_GPIO_Port, ENCODER_CLK_2_Pin, GPIO_PIN_SET);
		
		__ASM("NOP"); __ASM("NOP");		
		__ASM("NOP"); __ASM("NOP");
	}
	encoder_data[0] = dataa_1;
	encoder_data[1] = dataa_2;
	sprintf(encoder_Data, "encoder1: %d  encoder2: %d\r\n", dataa_1, dataa_2);
	HAL_GPIO_WritePin(ENCODER_CSn_1_GPIO_Port, ENCODER_CSn_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENCODER_CSn_2_GPIO_Port, ENCODER_CSn_2_Pin, GPIO_PIN_SET);
}

void ResetPWM(int speed)
{
	int period;
	period = CLOCK_SPEED / speed;
	HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
	MX_TIM10_Init(period, period/2);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
