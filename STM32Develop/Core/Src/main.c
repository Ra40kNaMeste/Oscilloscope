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



const int8_t ADC_NOP_COMMAND = 0x00;

int8_t ADC_RESET_COMMAND = 0x06;
int8_t ADC_START1_COMMAND = 0x08;
int8_t ADC_STOP1_COMMAND = 0xA;
int8_t ADC_START2_COMMAND = 0xC;
int8_t ADC_STOP2_COMMAND = 0x0E;

int8_t ADC_READ_DATA1_COMMAND = 0x12;
int8_t ADC_READ_DATA2_COMMAND = 0x14;

int8_t ADC_SYSTEM_OFFSET_CALIBRATION1_COMMAND = 0x16;
int8_t ADC_SYSTEM_GAIN_CALIBRATION1_COMMAND = 0x17;
int8_t ADC_SELF_OFFSET_CALLIBRATION1_COMMAND = 0x19;
int8_t ADC_SYSTEM_OFFSET_CALIBRATION2_COMMAND = 0x1B;
int8_t ADC_SYSTEM_GAIN_CALIBRATION2_COMMAND = 0x1C;
int8_t ADC_SELF_OFFSET_CALLIBRATION2_COMMAND = 0x1E;

int8_t ADC_MASK_READ_REGISTERS_COMMAND = 0x20;
int8_t ADC_MASK_WRITE_REGISTERS_COMMAND = 0x40;

GPIO_TypeDef* ADC_CONTROL_GPIO = GPIOA;
int8_t ADC_CONTROL_PIN_DRDY = GPIO_PIN_0;
int8_t ADC_CONTROL_PIN_DOUT = GPIO_PIN_1;
int8_t ADC_CONTROL_PIN_DIN = GPIO_PIN_2;
int8_t ADC_CONTROL_PIN_SCLK = GPIO_PIN_3;
int8_t ADC_CONTROL_PIN_CS = GPIO_PIN_4;
int8_t ADC_CONTROL_PIN_START = GPIO_PIN_5;
int8_t ADC_CONTROL_PIN_RESET = GPIO_PIN_6;

GPIO_TypeDef* MONOCHROME_GPIO = GPIOB;
int8_t MONOCHROME_FORWARD_PIN = GPIO_PIN_5;
int8_t MONOCHROME_STOP_PIN = GPIO_PIN_4;
int8_t MONOCHROME_REWIND_PIN = GPIO_PIN_3;

int8_t ADC_REGISTER_ID = 0x00;
int8_t ADC_REGISTER_POWER = 0x01;
int8_t ADC_REGISTER_INTERFACE = 0x02;
int8_t ADC_REGISTER_MODE0 = 0x03;
int8_t ADC_REGISTER_MODE1 = 0x04;
int8_t ADC_REGISTER_MODE2 = 0x05;
int8_t ADC_REGISTER_INPMUX = 0x06;
int8_t ADC_REGISTER_OFCAL0 = 0x07;
int8_t ADC_REGISTER_OFCAL1 = 0x08;
int8_t ADC_REGISTER_OFCAL2 = 0x09;
int8_t ADC_REGISTER_FSCAL0 = 0x0A;
int8_t ADC_REGISTER_FSCAL1 = 0x0B;
int8_t ADC_REGISTER_FSCAL2 = 0x0C;
int8_t ADC_REGISTER_IDACMUX = 0x0D;
int8_t ADC_REGISTER_IDACMAG = 0x0E;
int8_t ADC_REGISTER_REFMUX = 0x0F;
int8_t ADC_REGISTER_TDACP = 0x10;
int8_t ADC_REGISTER_TDACN = 0x11;
int8_t ADC_REGISTER_GPIOCON = 0x12;
int8_t ADC_REGISTER_GPIODIR = 0x13;
int8_t ADC_REGISTER_GPIODAT = 0x14;

int8_t ADC_INPMUX_TEMP_Sensor = 0x0B;
int8_t ADC_INPMUX_Analog_Monitor = 0x0C;
int8_t ADC_INPMUX_Digital_Monitor = 0x0D;
int8_t ADC_INPMUX_TDAC = 0x0E;
int8_t ADC_INPMUX_Float = 0x0F;

int8_t CanTranspData = 1;
int32_t summ;
int8_t count;

uint32_t time_convert = 0;
uint32_t timeout = 0;
uint8_t rx_buffer[4];
uint8_t props_buffer[30];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void ADC_Reset();
void ADC_Convert();
void ADC_One_Measure_Convert();
void ADC_Self_Offset_Calibration();
void ADC_System_Offset_Calibration();
void ADC_System_Gain_Calibration_5V();
void ADC_SET_Input_Mux(uint8_t inputN, uint8_t inputP);
void ADC_Send_Command(int8_t command);
uint8_t ADC_Read_Byte();

int8_t ADC_GET_Chop_Mode();
void ADC_SET_Chop_Mode(int8_t value);
int8_t ADC_GET_Delay();
void ADC_SET_Delay(int8_t value);
int8_t ADC_GET_REFREV();
void ADC_SET_REFREV(int8_t value);
int8_t ADC_READ_DEVID();
int8_t ADC_READ_REVID();
int8_t ADC_GET_VBIAS();
void ADC_SET_VBIAS(int8_t value);
int8_t ADC_GET_INTREF();
void ADC_SET_INTREF(int8_t value);
int8_t ADC_GET_Checksum_State();
void ADC_SET_Checksum_State(int8_t value);
int8_t ADC_GET_Status();
void ADC_SET_Status(int8_t value);
int8_t ADC_GET_TIMEOUT();
void ADC_SET_TIMEOUT(int8_t value);
int8_t ADC_GET_SBMAG();
void ADC_SET_SBMAG(int8_t value);
int8_t ADC_GET_SBPOL();
void ADC_SET_SBPOL(int8_t value);
int8_t ADC_GET_SBADC();
void ADC_SET_SBADC(int8_t value);
int8_t ADC_GET_FILTER();
void ADC_SET_FILTER(int8_t value);
int8_t ADC_GET_Data_Rate();
void ADC_SET_Data_Rate(int8_t value);
int8_t ADC_GET_GAIN();
void ADC_SET_GAIN(int8_t value);
int8_t ADC_GET_BYPASS();
void ADC_SET_BYPASS(int8_t value);
int8_t ADC_GET_IDACMUX1();
void ADC_SET_IDACMUX1(int8_t value);
int8_t ADC_GET_IDACMUX2();
void ADC_SET_IDACMUX2(int8_t value);
int8_t ADC_GET_IDACMAG1();
void ADC_SET_IDACMAG1(int8_t value);
int8_t ADC_GET_IDACMAG2();
void ADC_SET_IDACMAG2(int8_t value);
int8_t ADC_GET_RMUXN();
void ADC_SET_RMUXN(int8_t value);
int8_t ADC_GET_RMUXP();
void ADC_SET_RMUXP(int8_t value);
int8_t ADC_GET_MAGP();
void ADC_SET_MAGP(int8_t value);
int8_t ADC_GET_OUTP();
void ADC_SET_OUTP(int8_t value);
int8_t ADC_GET_MAGN();
void ADC_SET_MAGN(int8_t value);
int8_t ADC_GET_OUTN();
void ADC_SET_OUTN(int8_t value);
int8_t ADC_GET_COM();
void ADC_SET_COM(int8_t value);
int8_t ADC_GET_DIR();
void ADC_SET_DIR(int8_t value);
int8_t ADC_GET_DAT();
void ADC_SET_DAT(int8_t value);
void Convert_Input_Data(uint8_t* data);

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, rx_buffer, 4);
  HAL_UART_Transmit_IT(&huart1, rx_buffer, 4);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		time_convert++;
	}
}

void START_MONOCHROME()
{
	HAL_GPIO_WritePin(MONOCHROME_GPIO, MONOCHROME_REWIND_PIN , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MONOCHROME_GPIO, MONOCHROME_FORWARD_PIN | MONOCHROME_STOP_PIN, GPIO_PIN_SET);
}

void STOP_MONOCHROME()
{
	HAL_GPIO_WritePin(MONOCHROME_GPIO, MONOCHROME_FORWARD_PIN | MONOCHROME_STOP_PIN | MONOCHROME_REWIND_PIN, GPIO_PIN_RESET);
}

void REWIND_MONOCHROME()
{
	HAL_GPIO_WritePin(MONOCHROME_GPIO, MONOCHROME_FORWARD_PIN , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MONOCHROME_GPIO, MONOCHROME_REWIND_PIN | MONOCHROME_STOP_PIN, GPIO_PIN_SET);
}
//Delay tick by ADC (less that)
void ADC_Delay_CLK_Tiks(int8_t tiks){
	TIM3->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim3);
	while(TIM3->CNT <tiks){}
	HAL_TIM_Base_Stop_IT(&htim3);
}
//Send command to ADC 
//!Use command:
//!CS-LOW; Send command and CS Hight
void ADC_Send_Command(int8_t command){
	for(int i = 0; i<8; i++){
		if(command >> i & 0x01)
			HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_DIN, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_DIN, GPIO_PIN_RESET);
		ADC_Delay_CLK_Tiks(2);
		HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_SCLK, GPIO_PIN_SET);
		ADC_Delay_CLK_Tiks(2);
		HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_SCLK, GPIO_PIN_RESET);
		
	}
}

//Send command by two byte to ADC 
//!Use command:
//!CS-LOW; Send command and CS Hight

//Read byte register ADC
 uint8_t ADC_Read_Byte()
{
	uint8_t result = 0;
	for(int i = 0; i<8; i++)
	{
		HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_SCLK, GPIO_PIN_SET);
		uint8_t val = HAL_GPIO_ReadPin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_DOUT) << i;
		ADC_Delay_CLK_Tiks(2);
		result = result | val;
		HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_SCLK, GPIO_PIN_RESET);
		ADC_Delay_CLK_Tiks(2);

	}
	return result;
}

void ADC_Reset()
{
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
	ADC_Send_Command(ADC_RESET_COMMAND);
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
	ADC_Delay_CLK_Tiks(9);
}

void ADC_Start_By_Pin()
{
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_START, GPIO_PIN_SET);
	ADC_Delay_CLK_Tiks(5);
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_START, GPIO_PIN_RESET);
	ADC_Delay_CLK_Tiks(5);
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_START, GPIO_PIN_SET);
}
void ADC_Stop_By_Pin()
{
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_START, GPIO_PIN_RESET);
}

void ADC_START1()
{
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
	ADC_Send_Command(ADC_START1_COMMAND);
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
	
}
void ADC_STOP1()
{
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
	ADC_Send_Command(ADC_STOP1_COMMAND);
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
}

uint32_t ADC_READ_Data1()
{
	uint8_t status = ADC_Read_Byte();
	uint32_t data = 0;
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
	for(int i=0; i<4; i++){
		data = data | ADC_Read_Byte();
		data = data << 8;
	}
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
	return data;
}

//adress  - 4 bit; size - 4 bit
uint8_t ADC_READ_Ceil_Memory(uint8_t adress)
{
	uint8_t command = ADC_MASK_READ_REGISTERS_COMMAND + adress;
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
	ADC_Send_Command(command);
	ADC_Send_Command(0x00);
	uint8_t res = ADC_Read_Byte();
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
	return res;
}

void ADC_WRITE_Ceil_Memory(uint8_t adress, uint8_t value)
{
	int16_t command = ADC_MASK_WRITE_REGISTERS_COMMAND + adress;
	command = command<<8;
	command = command | 0x00;
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
	ADC_Send_Command(command);
	ADC_Send_Command(0x00);
	ADC_Send_Command(value);
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
}

void ADC_APPEND_Byte_To_Register(uint8_t adress, uint8_t mask, uint8_t value)
{
	uint8_t old_data = ADC_READ_Ceil_Memory(adress);
	value = value & mask;
	mask = ~mask;
	old_data = (old_data & mask) | value;
	ADC_WRITE_Ceil_Memory(adress, old_data);
}

int8_t ADC_READ_Bytes_By_Mask_From_Register(uint8_t adress, uint8_t offset, uint8_t maskSize)
{
	uint8_t res = ADC_READ_Ceil_Memory(adress);
	res = res >> offset;
	res = res & maskSize;
	return res;
}

void ADC_Convert()
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE0, 0x40, 0);
	TIM2->CNT = 0;
	summ = 0;
	count = 0;
	time_convert = 0;
	START_MONOCHROME();
	HAL_TIM_Base_Start_IT(&htim2);
	ADC_Start_By_Pin();
	int i = 0;
		while(time_convert < timeout)
		{
			if(!HAL_GPIO_ReadPin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_DRDY))
			{
				HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
				uint8_t status = ADC_Read_Byte();
				uint8_t dates[4];
				dates[0] = ADC_Read_Byte();
				dates[1] = ADC_Read_Byte();
				dates[2] = ADC_Read_Byte();
				dates[3] = ADC_Read_Byte();
				int8_t optional = ADC_Read_Byte();
				if(CanTranspData)
				{
					CanTranspData = 0;
					HAL_UART_Transmit_IT(&huart1, &dates[0], 4);
				}
				HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
			}
	}
	ADC_Stop_By_Pin();
	STOP_MONOCHROME();
	HAL_TIM_Base_Stop_IT(&htim2);
}

void RunMonochrome()
{
	TIM2->CNT = 0;
	time_convert = 0;
	START_MONOCHROME();
	HAL_TIM_Base_Start_IT(&htim2);
}

void StopMonochromeAndSaveTimeout()
{
	STOP_MONOCHROME();
	HAL_TIM_Base_Stop_IT(&htim2);
	timeout = time_convert;
}
void RunMonochromeByTimeConvert()
{
	TIM2->CNT = 0;
	time_convert = 0;
	START_MONOCHROME();
	HAL_TIM_Base_Start_IT(&htim2);
	while(time_convert < timeout)
	{
		
	}
	STOP_MONOCHROME();
	HAL_TIM_Base_Stop_IT(&htim2);
}

void Rewind()
{
	TIM2->CNT = 0;
	time_convert = 0;
	REWIND_MONOCHROME();
	HAL_TIM_Base_Start_IT(&htim2);
}

void RewindByTime()
{
	TIM2->CNT = 0;
	time_convert = 0;
	
	START_MONOCHROME();
	HAL_TIM_Base_Start_IT(&htim2);
	while(time_convert < timeout)
	{
		
	}
	STOP_MONOCHROME();
	HAL_TIM_Base_Stop_IT(&htim2);
	time_convert = 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	CanTranspData = 1;
}

void ADC_One_Measure_Convert()
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE0, 0x40, 0x40);
	
	ADC_Start_By_Pin();
	while(0x01)
	{
		if(!HAL_GPIO_ReadPin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_DRDY))
		{
			HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
			uint8_t status = ADC_Read_Byte();
			uint8_t dates[4];
			dates[0] = ADC_Read_Byte();
			dates[1] = ADC_Read_Byte();
			dates[2] = ADC_Read_Byte();
			dates[3] = ADC_Read_Byte();
			int8_t optional = ADC_Read_Byte();
			HAL_UART_Transmit(&huart1, &dates[0], 4, 0xFFFF);
			HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
			break;
		}
	}	
}

void ADC_Self_Offset_Calibration()
{
	uint8_t old_impmux;
	ADC_READ_Ceil_Memory(ADC_REGISTER_INPMUX);
	uint8_t old_mode0;
	ADC_READ_Ceil_Memory(ADC_REGISTER_MODE0);
	
	uint8_t temp = old_mode0 & 0x11001111;
	ADC_WRITE_Ceil_Memory(ADC_REGISTER_INPMUX, 0xFF);
	ADC_WRITE_Ceil_Memory(ADC_REGISTER_MODE0, temp);
	
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
	ADC_Send_Command(ADC_SELF_OFFSET_CALLIBRATION1_COMMAND);
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
	
	ADC_WRITE_Ceil_Memory(ADC_REGISTER_INPMUX, old_impmux);
	ADC_WRITE_Ceil_Memory(ADC_REGISTER_MODE0, old_mode0);
}

void ADC_System_Offset_Calibration()
{
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
	ADC_Send_Command(ADC_SYSTEM_GAIN_CALIBRATION1_COMMAND);
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
}

void ADC_SET_Input_Mux(uint8_t inputN, uint8_t inputP)
{
	inputP = inputP << 8;
	inputP = inputP | inputN;
	ADC_WRITE_Ceil_Memory(ADC_REGISTER_INPMUX, inputP);
}

void ADC_System_Gain_Calibration_5V()
{
	uint8_t old_impmux;
	ADC_READ_Ceil_Memory(ADC_REGISTER_INPMUX);
	
	ADC_SET_Input_Mux(0x0A, ADC_INPMUX_Analog_Monitor);
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_RESET);
	ADC_Send_Command(ADC_SYSTEM_GAIN_CALIBRATION1_COMMAND);
	HAL_GPIO_WritePin(ADC_CONTROL_GPIO, ADC_CONTROL_PIN_CS, GPIO_PIN_SET);
	
	ADC_WRITE_Ceil_Memory(ADC_REGISTER_INPMUX, old_impmux);
}



int8_t ADC_GET_Chop_Mode()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE0, 4, 0x03);
}
void ADC_SET_Chop_Mode(int8_t value)
{
	value = value << 4;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE0, 0x30, value);
}

int8_t ADC_GET_Delay()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE0, 0, 0x0F);
}
void ADC_SET_Delay(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE0, 0x0F, value);
}

int8_t ADC_GET_REFREV()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE0, 7, 0x01);
}
void ADC_SET_REFREV(int8_t value)
{
	value = value << 7;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE0, 0x80, value);
}

int8_t ADC_READ_DEVID()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_ID, 5, 0x07);
}

int8_t ADC_READ_REVID()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_ID, 0, 0x1F);
}

int8_t ADC_GET_VBIAS()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_POWER, 1, 0x01);
}
void ADC_SET_VBIAS(int8_t value)
{
	value = value << 1;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE0, 0x02, value);
}

int8_t ADC_GET_INTREF()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE0, 0, 0x01);
}
void ADC_SET_INTREF(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE0, 0x01, value);
}

int8_t ADC_GET_Checksum_State()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_INTERFACE, 0, 0x03);
}
void ADC_SET_Checksum_State(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_INTERFACE, 0x03, value);
}

int8_t ADC_GET_Status()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_INTERFACE, 2, 0x01);
}
void ADC_SET_Status(int8_t value)
{
	value = value << 2;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_INTERFACE, 0x04, value);
}

int8_t ADC_GET_TIMEOUT()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_INTERFACE, 3, 0x01);
}
void ADC_SET_TIMEOUT(int8_t value)
{
	value = value << 3;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_INTERFACE, 0x08, value);
}

int8_t ADC_GET_SBMAG()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE1, 0, 0x07);
}
void ADC_SET_SBMAG(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE1, 0x07, value);
}

int8_t ADC_GET_SBPOL()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE1, 3, 0x01);
}
void ADC_SET_SBPOL(int8_t value)
{
	value = value << 3;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE1, 0x08, value);
}

int8_t ADC_GET_SBADC()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE1, 4, 0x01);
}
void ADC_SET_SBADC(int8_t value)
{
	value = value << 4;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE1, 0x10, value);
}

int8_t ADC_GET_FILTER()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE1, 5, 0x07);
}
void ADC_SET_FILTER(int8_t value)
{
	value = value << 5;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE1, 0xE0, value);
}

int8_t ADC_GET_Data_Rate()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE2, 0, 0x0F);
}
void ADC_SET_Data_Rate(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE2, 0x0F, value);
}

int8_t ADC_GET_GAIN()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE2, 4, 0x07);
}
void ADC_SET_GAIN(int8_t value)
{
	value = value << 4;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE2, 0x70, value);
}

int8_t ADC_GET_BYPASS()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_MODE2, 7, 0x01);
}
void ADC_SET_BYPASS(int8_t value)
{
	value = value << 7;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_MODE2, 0x80, value);
}

int8_t ADC_GET_IDACMUX1()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_IDACMUX, 0, 0x0F);
}
void ADC_SET_IDACMUX1(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_IDACMUX, 0x0F, value);
}

int8_t ADC_GET_IDACMUX2()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_IDACMUX, 7, 0x0F);
}
void ADC_SET_IDACMUX2(int8_t value)
{
	value = value << 4;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_IDACMUX, 0xF0, value);
}

int8_t ADC_GET_IDACMAG1()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_IDACMAG, 0, 0x0F);
}
void ADC_SET_IDACMAG1(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_IDACMAG, 0x0F, value);
}

int8_t ADC_GET_IDACMAG2()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_IDACMAG, 7, 0x0F);
}
void ADC_SET_IDACMAG2(int8_t value)
{
	value = value << 4;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_IDACMAG, 0xF0, value);
}

int8_t ADC_GET_RMUXN()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_REFMUX, 0, 0x07);
}
void ADC_SET_RMUXN(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_REFMUX, 0x07, value);
}

int8_t ADC_GET_RMUXP()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_REFMUX, 3, 0x07);
}
void ADC_SET_RMUXP(int8_t value)
{
	value = value << 3;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_REFMUX, 0x38, value);
}

int8_t ADC_GET_MAGP()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_TDACP, 0, 0x1F);
}
void ADC_SET_MAGP(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_TDACP, 0x1F, value);
}

int8_t ADC_GET_OUTP()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_TDACP, 7, 0x01);
}
void ADC_SET_OUTP(int8_t value)
{
	value = value << 7;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_TDACP, 0x80, value);
}

int8_t ADC_GET_MAGN()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_TDACN, 0, 0x1F);
}
void ADC_SET_MAGN(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_TDACN, 0x1F, value);
}

int8_t ADC_GET_OUTN()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_TDACN, 7, 0x01);
}
void ADC_SET_OUTN(int8_t value)
{
	value = value << 7;
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_TDACN, 0x80, value);
}

int8_t ADC_GET_COM()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_GPIOCON, 0, 0xFF);
}
void ADC_SET_COM(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_GPIOCON, 0xFF, value);
}

int8_t ADC_GET_DIR()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_GPIODIR, 0, 0xFF);
}
void ADC_SET_DIR(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_GPIODIR, 0xFF, value);
}

int8_t ADC_GET_DAT()
{
	return ADC_READ_Bytes_By_Mask_From_Register(ADC_REGISTER_GPIODAT, 0, 0xFF);
}
void ADC_SET_DAT(int8_t value)
{
	ADC_APPEND_Byte_To_Register(ADC_REGISTER_GPIODAT, 0xFF, value);
}


int8_t GetADCPropertySwitch(uint8_t index)
{
	
	switch(index)
		{
		case 0:
			return ADC_GET_Chop_Mode();
		case 1:
			return ADC_GET_Delay();
		case 2:
			return ADC_GET_REFREV();
		case 3:
			return ADC_READ_DEVID();
		case 4:
			return ADC_READ_REVID();
		case 5:
			return ADC_GET_VBIAS();
		case 6:
			return ADC_GET_INTREF();
		case 7:
			return ADC_GET_Checksum_State();
		case 8:
			return ADC_GET_Status();
		case 9:
			return ADC_GET_TIMEOUT();
		case 10:
			return ADC_GET_SBMAG();
		case 11:
			return ADC_GET_SBPOL();
		case 12:
			return ADC_GET_SBADC();
		case 13:
			return ADC_GET_FILTER();
		case 14:
			return ADC_GET_DIR();
		case 15:
			return ADC_GET_GAIN();
		case 16:
			return ADC_GET_BYPASS();
		case 17:
			return ADC_GET_IDACMUX1();
		case 18:
			return ADC_GET_IDACMUX2();
		case 19:
			return ADC_GET_IDACMAG1();
		case 20:
			return ADC_GET_IDACMAG2();
		case 21:
			return ADC_GET_RMUXN();
		case 22:
			return ADC_GET_RMUXP();
		case 23:
			return ADC_GET_OUTP();
		case 24:
			return ADC_GET_MAGP();
		case 25:
			return ADC_GET_OUTN();
		case 26:
			return ADC_GET_MAGN();
		case 27:
			return ADC_GET_COM();
		case 28:
			return ADC_GET_DIR();
		case 29:
			return ADC_GET_DAT();
		}
		return 0;
}

void SetADCPropertySwitch(uint8_t index, int8_t value)
{
	switch(index)
		{
		case 0:
			ADC_SET_Chop_Mode(value);
		case 1:
			ADC_SET_Delay(value);
		case 2:
			ADC_SET_REFREV(value);
		case 5:
			ADC_SET_VBIAS(value);
		case 6:
			ADC_SET_INTREF(value);
		case 7:
			ADC_SET_Checksum_State(value);
		case 8:
			ADC_SET_Status(value);
		case 9:
			ADC_SET_TIMEOUT(value);
		case 10:
			ADC_SET_SBMAG(value);
		case 11:
			ADC_SET_SBPOL(value);
		case 12:
			ADC_SET_SBADC(value);
		case 13:
			ADC_SET_FILTER(value);
		case 14:
			ADC_SET_DIR(value);
		case 15:
			ADC_SET_GAIN(value);
		case 16:
			ADC_SET_BYPASS(value);
		case 17:
			ADC_SET_IDACMUX1(value);
		case 18:
			ADC_SET_IDACMUX2(value);
		case 19:
			ADC_SET_IDACMAG1(value);
		case 20:
			ADC_SET_IDACMAG2(value);
		case 21:
			ADC_SET_RMUXN(value);
		case 22:
			ADC_SET_RMUXP(value);
		case 23:
			ADC_SET_OUTP(value);
		case 24:
			ADC_SET_MAGP(value);
		case 25:
			ADC_SET_OUTN(value);
		case 26:
			ADC_SET_MAGN(value);
		case 27:
			ADC_SET_COM(value);
		case 28:
			ADC_SET_DIR(value);
		case 29:
			ADC_SET_DAT(value);
		}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Convert_Input_Data(rx_buffer);
	HAL_UART_Receive_IT(&huart1, rx_buffer, 4);
}


//Array 4 byte
void Convert_Input_Data(uint8_t* data)
{
	
	switch(data[0])
	{
		case 0:
		{
			HAL_UART_Transmit_IT(&huart1, rx_buffer, 4);
			break;
		}
		case 1:
		{
			ADC_Reset();
			break;
		}
		case 2:
		{

			ADC_Convert();
			break;
		}
		case 3:
		{
			ADC_One_Measure_Convert();
			break;
		}
		case 4:
		{
			ADC_Self_Offset_Calibration();
			break;
		}
		case 5:
		{
			ADC_System_Offset_Calibration();
		}
		case 6:
		{
			ADC_System_Gain_Calibration_5V();
		}
		case 7:
		{
			if(data[3]&0x01)
			{
				ADC_SET_Input_Mux(7, 5);
			}
			else
			{
				ADC_SET_Input_Mux(3,5);
			}
		}
		case 8:
		{
			for(int i=0; i<30; i++){
				props_buffer[i] = GetADCPropertySwitch(i);
			}
			
			HAL_UART_Transmit_IT(&huart1, props_buffer, 30);
			break;
		}
		case 9:
		{
			if(data[3] < 30)
			{
				props_buffer[1] = GetADCPropertySwitch(data[3]);
				HAL_UART_Transmit_IT(&huart1, props_buffer, 1);
			}
			break;
		}
		case 10:
		{
			if(data[3] < 30)
			{
				SetADCPropertySwitch(data[2], data[3]);
			}
			break;
		}
		case 11:
		{
			RunMonochrome();
			break;
		}
		case 12:
		{
			Rewind();
			break;
		}
		case 13:
		{
			StopMonochromeAndSaveTimeout();
			break;
		}
		case 14:
		{
			RunMonochromeByTimeConvert();
			break;
		}
		case 15:
		{
			RewindByTime();
			break;
		}
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
