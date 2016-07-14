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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
void encoder_init(void);
void get_four_encoder(char* encoder_Data);
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	encoder_init();
	HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	char encoder_data[200];
	char *fuck = "fuck-test \r\n";
	GPIO_PinState state;
//	sprintf(encoder_data, "%d/n", t);
//	*encoder_data = "7\r\n";
  while (1)
  {
  /* USER CODE END WHILE */
		get_four_encoder(encoder_data);
//		HAL_UART_Transmit_IT(&huart2, (uint8_t *)encoder_data, strlen(encoder_data));
//		HAL_Delay(1);

  /* USER CODE BEGIN 3 */
//		state = HAL_GPIO_ReadPin(ENCODER_DAT_1_GPIO_Port, ENCODER_DAT_1_Pin);
//		sprintf(encoder_data, "%d\r\n", state);
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)encoder_data, strlen(encoder_data));

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
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ENCODER_CLK_1_Pin|ENCODER_CSn_1_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
void encoder_init(void)
{
	HAL_GPIO_WritePin(ENCODER_CSn_1_GPIO_Port, ENCODER_CSn_1_Pin, GPIO_PIN_SET);
	return ;
}

void get_four_encoder(char* encoder_Data)
{
	int i;
	uint16_t dataa_1 = 0, temp = 0;
	HAL_GPIO_WritePin(ENCODER_CSn_1_GPIO_Port, ENCODER_CSn_1_Pin, GPIO_PIN_RESET);
	__ASM("NOP");
	__ASM("NOP");
	
	for(i=0; i<12; i++)
	{
		HAL_GPIO_WritePin(ENCODER_CLK_1_GPIO_Port, ENCODER_CLK_1_Pin, GPIO_PIN_RESET);
		__ASM("NOP"); __ASM("NOP");		
		__ASM("NOP"); __ASM("NOP");
		__ASM("NOP"); __ASM("NOP");		
		__ASM("NOP"); __ASM("NOP");
		HAL_GPIO_WritePin(ENCODER_CLK_1_GPIO_Port, ENCODER_CLK_1_Pin, GPIO_PIN_SET);
		__ASM("NOP"); __ASM("NOP");		
		__ASM("NOP"); __ASM("NOP");
		__ASM("NOP"); __ASM("NOP");		
		__ASM("NOP"); __ASM("NOP");
		temp = HAL_GPIO_ReadPin(ENCODER_DAT_1_GPIO_Port, ENCODER_DAT_1_Pin);
		dataa_1 = (dataa_1 << 1) + temp;
		__ASM("NOP"); __ASM("NOP");
		__ASM("NOP"); __ASM("NOP");
		__ASM("NOP"); __ASM("NOP");
	}
	
	for(i=0 ; i<4; i++)
	{
		HAL_GPIO_WritePin(ENCODER_CLK_1_GPIO_Port, ENCODER_CLK_1_Pin, GPIO_PIN_RESET);
		__ASM("NOP"); __ASM("NOP");		
		__ASM("NOP"); __ASM("NOP");
		HAL_GPIO_WritePin(ENCODER_CLK_1_GPIO_Port, ENCODER_CLK_1_Pin, GPIO_PIN_SET);
		__ASM("NOP"); __ASM("NOP");		
		__ASM("NOP"); __ASM("NOP");
	}
	sprintf(encoder_Data, "%d\r\n", dataa_1);
	HAL_GPIO_WritePin(ENCODER_CSn_1_GPIO_Port, ENCODER_CSn_1_Pin, GPIO_PIN_SET);
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
