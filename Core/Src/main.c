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
#define CE_PORT GPIOB // PB6 chip enable (aka slave select)
#define CE_PIN GPIO_PIN_6

#define DC_PORT GPIOC // PA0 data/control
#define DC_PIN GPIO_PIN_7

#define RESET_PORT GPIOA // PA1 reset
#define RESET_PIN GPIO_PIN_9

#define GLCD_WIDTH 84
#define GLCD_HEIGHT 48
#define NUM_BANKS 6
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
int time;
const char font_table_other[][6] = {
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // blank
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // block
};
// number design 0-9
const char numbers[][6][6] = {
		{ // 0
				{0,1,1,1,0,0},
				{1,0,0,0,1,0},
				{1,0,0,0,1,0},
				{1,0,0,0,1,0},
				{1,0,0,0,1,0},
				{0,1,1,1,0,0}
		},
		{ // 1
				{0,0,1,0,0,0},
				{0,1,1,0,0,0},
				{0,0,1,0,0,0},
				{0,0,1,0,0,0},
				{0,0,1,0,0,0},
				{1,1,1,1,1,0}
		},
		{ // 2
				{0,1,1,1,0,0},
				{1,0,0,0,1,0},
				{0,0,0,1,0,0},
				{0,0,1,0,0,0},
				{0,1,0,0,0,0},
				{1,1,1,1,1,0},
		},
		{ // 3
				{0,1,1,1,0,0},
				{1,0,0,0,1,0},
				{0,0,0,0,1,0},
				{0,0,1,1,1,0},
				{1,0,0,0,1,0},
				{0,1,1,1,0,0},
		},
		{ // 4
				{0,0,0,1,0,0},
				{0,0,1,1,0,0},
				{0,1,0,1,0,0},
				{1,1,1,1,1,0},
				{0,0,0,1,0,0},
				{0,0,0,1,0,0},
		},
		{ // 5
				{1,1,1,1,1,0},
				{1,0,0,0,0,0},
				{1,1,1,1,0,0},
				{0,0,0,0,1,0},
				{1,0,0,0,1,0},
				{0,1,1,1,0,0},
		},
		{ // 6
				{0,1,1,1,0,0},
				{1,0,0,0,1,0},
				{1,0,0,0,0,0},
				{1,1,1,1,0,0},
				{1,0,0,0,1,0},
				{0,1,1,1,0,0},
		},
		{ // 7
				{1,1,1,1,1,0},
				{0,0,0,0,1,0},
				{0,0,0,1,0,0},
				{0,0,1,0,0,0},
				{0,1,0,0,0,0},
				{1,0,0,0,0,0},
		},
		{ // 8
				{0,1,1,1,0,0},
				{1,0,0,0,1,0},
				{0,1,1,1,0,0},
				{1,0,0,0,1,0},
				{1,0,0,0,1,0},
				{0,1,1,1,0,0},
		},
		{ // 9
				{0,1,1,1,1,0},
				{1,0,0,0,1,0},
				{1,0,0,0,1,0},
				{0,1,1,1,1,0},
				{0,0,0,0,1,0},
				{0,0,0,0,1,0},
		},


};
int ten = 1;
int one = 1;
int change = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void StopLightState(char time);
void SPI_write(unsigned char data);
void GLCD_data_write(unsigned char data);
void GLCD_command_write(unsigned char data);
void GLCD_init(void);
void GLCD_setCursor(unsigned char x, unsigned char y);
void GLCD_clear(void);
void GLCD_putchar(int font_table_row);
void GLCD_putblock(int font_table_row);
void GLCD_writeNumberLeft(int num);
void GLCD_writeNumberRight(int num);
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
  MX_TIM16_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  GLCD_init(); // initialize the screen
  GLCD_clear(); // clear the screen
  HAL_TIM_Base_Start_IT(&htim16); // Start Timer 16

  	HAL_GPIO_WritePin(GreenM_GPIO_Port, GreenM_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(YellowM_GPIO_Port, YellowM_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RedM_GPIO_Port, RedM_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GreenS_GPIO_Port, GreenS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(YellowS_GPIO_Port, YellowS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RedS_GPIO_Port, RedS_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, YellowS_Pin|GreenS_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GreenM_Pin|YellowM_Pin|RedS_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RedM_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : YellowS_Pin GreenS_Pin PC7 */
  GPIO_InitStruct.Pin = YellowS_Pin|GreenS_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GreenM_Pin YellowM_Pin RedS_Pin PA9 */
  GPIO_InitStruct.Pin = GreenM_Pin|YellowM_Pin|RedS_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RedM_Pin PB6 */
  GPIO_InitStruct.Pin = RedM_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : plate_Pin */
  GPIO_InitStruct.Pin = plate_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(plate_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// interrupt for the timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim16){
		// checks it one of the buttons has been pressed
		if (time > 5 & HAL_GPIO_ReadPin(GPIOA, plate_Pin) == 0 & change == 0){
			time = 0;
			change = 1;
		}
		// Starts the to cycle the lights after one of the buttons has been pressed
		if (time > 21 & change == 1){
			time = 0;
			change = 0;
			StopLightState(0);
		}
	}
	// Progresses the number on the timer.
	if (change == 1){
		StopLightState(time);

		GLCD_writeNumberLeft(ten);
		GLCD_writeNumberRight(one);
		if (ten == 0 & one == 0){
		}
		else if (one == 0){
			  ten = ten -1;
			  one = 9;
		}
		else {
			  one = one - 1;
		}
	}
	time = time + 1;
}

void SPI_write(unsigned char data){

	// Chip Enable (low is asserted)
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);

	// Send data over SPI1
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, 1, HAL_MAX_DELAY);

	// Chip Disable
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
}

void GLCD_data_write(unsigned char data){

	// Switch to "data" mode (D/C pin high)
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);

	// Send data over SPI
	SPI_write(data);

}

void GLCD_command_write(unsigned char data){

	// Switch to "command" mode (D/C pin low)
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);

	// Send data over SPI
	SPI_write(data);
}

void GLCD_init(void){

	// Keep CE high when not transmitting
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);

	// Reset the screen (low pulse - down & up)
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);

	// Configure the screen (according to the datasheet)
	GLCD_command_write(0x21); // enter extended command mode
	GLCD_command_write(0xA0); // set LCD Vop for contrast (this may be adjusted)
	GLCD_command_write(0x04); // set temp coefficient
	GLCD_command_write(0x11); // set LCD bias mode (this may be adjusted)
	GLCD_command_write(0x20); // return to normal command mode
	GLCD_command_write(0x0C); // set display mode normal
}
//  Sets the location of the cursor to a given x and y coordinate
void GLCD_setCursor(unsigned char x, unsigned char y){

	GLCD_command_write(0x80 | x); // column
	GLCD_command_write(0x40 | y); // bank
}
//  Sets all of the pixels on the screen to off clearing the screen.
void GLCD_clear(void){

	int i;
	for(i = 0; i < (GLCD_WIDTH * NUM_BANKS); i++){
		GLCD_data_write(0x00); // write zeros
	}
	GLCD_setCursor(0,0); // return cursor to top left
}
// fills an entire bank or leaves it empty.
void GLCD_putblock(int font_table_row){
	int i;
	for (i=0; i<6; i++){
		GLCD_data_write(font_table_other[font_table_row][i]);
	}
}
// Writes to the left of the screen starting at coordinates 0,0
void GLCD_writeNumberLeft(int num){
	int i;
	int i2;

	for (i=0; i<6; i++){
		GLCD_setCursor(0,(9*i)); // keeps the cursor on the left side of the screen
		for (i2=0; i2<6; i2++){

			// determines whether to fill bank with a block or a blank
			if ((numbers[num][i][i2]) == 1){
				GLCD_putblock(1); // fills bank with a block
			}
			else {
				GLCD_putblock(0); // fills bank with a blank
			}
		}
	}
}
// Whites to the Right of the creen starting at coordinates 36, 0
void GLCD_writeNumberRight(int num){
	int i;
	int i2;

	for (i=0; i<6; i++){
		GLCD_setCursor(36,(9*i)); // keeps the cursor on the right side of the screen
		for (i2=0; i2<6; i2++){

			// determines whether to fill bank with a block or a blank
			if ((numbers[num][i][i2]) == 1){
				GLCD_putblock(1); // fills bank with a block
			}
			else {
				GLCD_putblock(0); // fills bank with a blank
			}
		}
	}
}
// State machine for the stop light changes.
void StopLightState(char time){
	switch (time){

	case 0x0: // Main on green, Side on red
		HAL_GPIO_WritePin(GreenM_GPIO_Port, GreenM_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(YellowM_GPIO_Port, YellowM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RedM_GPIO_Port, RedM_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GreenS_GPIO_Port, GreenS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowS_GPIO_Port, YellowS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RedS_GPIO_Port, RedS_Pin, GPIO_PIN_SET);
		ten = 1;
		one = 1;
		break;

	case 0x7: // Main on yellow, Side on red
		HAL_GPIO_WritePin(GreenM_GPIO_Port, GreenM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowM_GPIO_Port, YellowM_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RedM_GPIO_Port, RedM_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GreenS_GPIO_Port, GreenS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowS_GPIO_Port, YellowS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RedS_GPIO_Port, RedS_Pin, GPIO_PIN_SET);
		break;

	case 0x0A: // Main on red, Side on red
		HAL_GPIO_WritePin(GreenM_GPIO_Port, GreenM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowM_GPIO_Port, YellowM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RedM_GPIO_Port, RedM_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GreenS_GPIO_Port, GreenS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowS_GPIO_Port, YellowS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RedS_GPIO_Port, RedS_Pin, GPIO_PIN_SET);
		break;

	case 0x0B: // Main on red, Side on green
		HAL_GPIO_WritePin(GreenM_GPIO_Port, GreenM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowM_GPIO_Port, YellowM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RedM_GPIO_Port, RedM_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GreenS_GPIO_Port, GreenS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(YellowS_GPIO_Port, YellowS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RedS_GPIO_Port, RedS_Pin, GPIO_PIN_RESET);
		break;

	case 0x12: // Main on red, Side on yellow
		HAL_GPIO_WritePin(GreenM_GPIO_Port, GreenM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowM_GPIO_Port, YellowM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RedM_GPIO_Port, RedM_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GreenS_GPIO_Port, GreenS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowS_GPIO_Port, YellowS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RedS_GPIO_Port, RedS_Pin, GPIO_PIN_RESET);
		break;

	case 0x15: // Main on red, Side on red
		HAL_GPIO_WritePin(GreenM_GPIO_Port, GreenM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowM_GPIO_Port, YellowM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RedM_GPIO_Port, RedM_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GreenS_GPIO_Port, GreenS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowS_GPIO_Port, YellowS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RedS_GPIO_Port, RedS_Pin, GPIO_PIN_SET);
		break;


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
