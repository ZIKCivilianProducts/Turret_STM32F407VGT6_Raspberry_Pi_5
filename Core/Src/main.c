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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile unsigned char UART_ready = 1;

size_t Size_Rx_UART;
size_t Size_Tx_UART;

float Difference;







uint32_t freq_az;
uint32_t freq_el;

char See_buffer_UART[21];
char Mode_work = 10;
char Working_zon_AZ = 10;
char Working_zon_EL = 10;
float Angular_AZ = 0.0f;
float Angular_EL = 0.0f;
uint32_t Level_AZ;
uint32_t Level_EL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

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
  	Size_Rx_UART = sizeof(Target.Rx_data);
	Size_Tx_UART = sizeof(Target.Tx_data);

	Motor_AZ.Config.PWM.Timer = &htim3; Motor_AZ.Config.Convertor.Convertor = &hadc1;
	Motor_EL.Config.PWM.Timer = &htim2; Motor_EL.Config.Convertor.Convertor = &hadc2;

	PID_AZ.last_time = HAL_GetTick();
	PID_EL.last_time = HAL_GetTick();
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t*)Target.Rx_data, Size_Rx_UART);

//  IMM_Init(Motor_AZ.Status.Angular);
//  IMM_Init(Motor_EL.Status.Angular);

  // Без PWM пращения происходить не будет, при этом происходит блокировка
  HAL_GPIO_WritePin(Motor_AZ.Config.GPIO.ENA_port, Motor_AZ.Config.GPIO.ENA_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Motor_EL.Config.GPIO.ENA_port, Motor_EL.Config.GPIO.ENA_pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  See_buffer_UART[0] = Target.Rx_data[0];
	  See_buffer_UART[1] = Target.Rx_data[1];
	  See_buffer_UART[2] = Target.Rx_data[2];
	  See_buffer_UART[3] = Target.Rx_data[3];
	  See_buffer_UART[4] = Target.Rx_data[4];
	  See_buffer_UART[5] = Target.Rx_data[5];
	  See_buffer_UART[6] = Target.Rx_data[6];
	  See_buffer_UART[7] = Target.Rx_data[7];
	  See_buffer_UART[8] = Target.Rx_data[8];
	  See_buffer_UART[9] = Target.Rx_data[9];
	  See_buffer_UART[0] = Target.Rx_data[0];
	  See_buffer_UART[11] = Target.Rx_data[11];
	  See_buffer_UART[12] = Target.Rx_data[12];
	  See_buffer_UART[13] = Target.Rx_data[13];
	  See_buffer_UART[14] = Target.Rx_data[14];
	  See_buffer_UART[15] = Target.Rx_data[15];
	  See_buffer_UART[16] = Target.Rx_data[16];
	  See_buffer_UART[17] = Target.Rx_data[17];
	  See_buffer_UART[18] = Target.Rx_data[18];
	  See_buffer_UART[19] = Target.Rx_data[19];
	  See_buffer_UART[20] = Target.Rx_data[20];

	  Level_AZ = Motor_AZ.Status.Discrete_level;
	  Level_EL = Motor_EL.Status.Discrete_level;

	  Angular_AZ = Motor_AZ.Status.Angular;
	  Angular_EL = Motor_EL.Status.Angular;

	  Working_zon_AZ = Working_area(&Motor_AZ);
	  Working_zon_EL = Working_area(&Motor_EL);

	  freq_az = Motor_AZ.Status.Frequency;
	  freq_el = Motor_EL.Status.Frequency;

	  Read_AD_Conversion(&Motor_AZ);
	  Read_AD_Conversion(&Motor_EL);

//	  switch (Target.Rx_data[15])
//	  {
//	  case '0': // Управление_стрелками_(ручной_решим)
//		  if (Working_area(&Motor_AZ))
//			  if (Target.Azimuth != 0)
//				  if (!Motor_AZ.Status.Moving) Start_motor(&Motor_AZ, Target.Azimuth > 0 ? Left : Right);
//				  else Up_fequency(&Motor_AZ);
//			  else Stop_motor(&Motor_AZ);
//		  else Moving_away_from_borders(&Motor_AZ, 1000, Motor_AZ.Status.Angular > 0 ? Right : Left);
//
//		  if (Working_area(&Motor_EL))
//			  if (Target.Elevation != 0)
//				  if (!Motor_EL.Status.Moving) Start_motor(&Motor_EL, Target.Elevation > 0 ? Up : Down);
//				  else Up_fequency(&Motor_EL);
//			  else Stop_motor(&Motor_EL);
//		  else Moving_away_from_borders(&Motor_EL, 100, Motor_EL.Status.Angular > 0 ? Down : Up);
//
//		  if (Target.Rx_data[17] == '0') // Пиф-паф
//		  break;

//	  case '1': // Передача_конечного_угла_(полуавтоматический_режим)
//		  Difference = Target.Azimuth - Motor_AZ.Status.Angular;
//		  if (Working_area(&Motor_AZ))
//			  if (fabsf(Difference) > Motor_AZ.Config.Angular.Guidance_accuracy)
//				  if (!Motor_AZ.Status.Moving) Start_motor(&Motor_AZ, Difference > 0 ? Left : Right);
//				  else Up_fequency(&Motor_AZ);
//			  else Stop_motor(&Motor_AZ);
//		  else Moving_away_from_borders(&Motor_AZ, 100, Motor_AZ.Status.Angular > 0 ? Right : Left);
//
//		  Difference = Target.Elevation - Motor_EL.Status.Angular;
//		  if (Working_area(&Motor_EL))
//			  if (fabsf(Difference) > Motor_EL.Config.Angular.Guidance_accuracy)
//				  if (!Motor_EL.Status.Moving) Start_motor(&Motor_EL, Difference > 0 ? Up : Down);
//				  else Up_fequency(&Motor_EL);
//			  else Stop_motor(&Motor_EL);
//		  else Moving_away_from_borders(&Motor_EL, 10, Motor_EL.Status.Angular > 0 ? Down : Up);
//
//		  if (Target.Rx_data[17] == '0') // Пиф-паф
//		  break;
//
//	  case '2': // Передача_разности_углов_(автономный_режим)
//		  Difference = Target.Azimuth;
//		  if (Working_area(&Motor_AZ))
//			  if (fabsf(Difference) > Motor_AZ.Config.Angular.Guidance_accuracy)
//				  if (!Motor_AZ.Status.Moving) Start_motor(&Motor_AZ, Difference > 0 ? Left : Right);
//				  else Up_fequency(&Motor_AZ);
//			  else Stop_motor(&Motor_AZ);
//		  else Moving_away_from_borders(&Motor_AZ, 100, Motor_AZ.Status.Angular > 0 ? Right : Left);
//
//		  Difference = Target.Elevation;
//		  if (Working_area(&Motor_EL))
//			  if (fabsf(Difference) > Motor_EL.Config.Angular.Guidance_accuracy)
//				  if (!Motor_EL.Status.Moving) Start_motor(&Motor_EL, Difference > 0 ? Up : Down);
//				  else Up_fequency(&Motor_EL);
//			  else Stop_motor(&Motor_EL);
//		  else Moving_away_from_borders(&Motor_EL, 10, Motor_EL.Status.Angular > 0 ? Down : Up);
//
//		  if (Target.Rx_data[17] == '0') // Пиф-паф
//		  // break;
//		  // Выполнение должно проволиться в default
//	  default: // Отправка_данных_компьютеру
//		  if (!Target.transmitting)
//		  {
//			  uint16_t angular_az = (uint16_t)fabs(Motor_AZ.Status.Angular * 10);
//			  uint16_t angular_el = (uint16_t)fabs(Motor_EL.Status.Angular * 10);
//
//			  Target.Tx_data[1] = (Motor_AZ.Status.Angular >= 0) ? '1' : '0';
//			  Target.Tx_data[2] = '0' + (angular_az / 1000) % 10;
//			  Target.Tx_data[3] = '0' + (angular_az / 100) % 10;
//			  Target.Tx_data[4] = '0' + (angular_az / 10) % 10;
//			  Target.Tx_data[5] = '0' + (angular_az) % 10;
//
//			  Target.Tx_data[6] = (Motor_EL.Status.Angular >= 0) ? '1' : '0';
//			  Target.Tx_data[7] = '0' +  (angular_el / 1000) % 10;
//			  Target.Tx_data[8] = '0' +  (angular_el / 100) % 10;
//			  Target.Tx_data[9] = '0' +  (angular_el / 10) % 10;
//			  Target.Tx_data[10] = '0' + (angular_el) % 10;
//
//			  if (HAL_UART_Transmit_IT(&huart2, (uint8_t*)Target.Tx_data, Size_Tx_UART) == HAL_OK) Target.transmitting = 1;
//		  };
//		  break;
//	  };
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Work_permit_GPIO_Port, Work_permit_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DIR_AZ_Pin|DIR_EL_Pin|Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ENA_AZ_Pin|ENA_EL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_green_Pin|LD3_orange_Pin|LD5_red_Pin|LD6_blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Trigger_position_Pin */
  GPIO_InitStruct.Pin = Trigger_position_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Trigger_position_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Work_permit_Pin */
  GPIO_InitStruct.Pin = Work_permit_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Work_permit_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_AZ_Pin DIR_EL_Pin ENA_AZ_Pin ENA_EL_Pin
                           Trigger_Pin */
  GPIO_InitStruct.Pin = DIR_AZ_Pin|DIR_EL_Pin|ENA_AZ_Pin|ENA_EL_Pin
                          |Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_green_Pin LD3_orange_Pin LD5_red_Pin LD6_blue_Pin */
  GPIO_InitStruct.Pin = LD4_green_Pin|LD3_orange_Pin|LD5_red_Pin|LD6_blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Target.Rx_data[Size_Rx_UART - 1] = '\0';

	Target.Azimuth =   (Target.Rx_data[3]  - '0') * 100.0f + (Target.Rx_data[4]  - '0') * 10.0f + (Target.Rx_data[5]  - '0') + (Target.Rx_data[6]  - '0') * 0.1f;
	if (Target.Rx_data[2] == '-') Target.Azimuth = -Target.Azimuth;

	Target.Elevation = (Target.Rx_data[10] - '0') * 100.0f + (Target.Rx_data[11] - '0') * 10.0f + (Target.Rx_data[12] - '0') + (Target.Rx_data[13] - '0') * 0.1f;
	if (Target.Rx_data[9] == '-') Target.Elevation = -Target.Elevation;

	HAL_UART_Receive_IT(&huart2, (uint8_t*)Target.Rx_data, Size_Rx_UART);
};

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Target.transmitting = 0;  // Сброс флага после передачи
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
	//HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
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
