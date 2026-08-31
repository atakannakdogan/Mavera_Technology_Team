/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <math.h>

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

CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
uint32_t TxMailbox;
int i;
uint16_t testTemp_analog;
float testTemp;
uint16_t motorTemp_analog;
float motorTemp;
uint8_t dataReceive[8];
uint8_t Package8_1[8];
uint8_t Package8_2[8];
uint8_t Package8_3[8];
uint8_t Package8_4[8];
int gps_speed;

float Package1[8];
float Package2[8];
float Package3[8];
float Package4[8];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
double Thermistor(int analogValue){

 double temperature;
 temperature = log(((40950000 / analogValue) - 10000));// 4095 ADC çözünürlüğü(12 bit)
 temperature = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temperature * temperature)) * temperature);
 temperature = temperature - 273.15;
 return temperature;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct carData{
	  float voltage1;
	  float voltage2;
	  float voltage3;
	  float voltage4;
	  float voltage5;
	  float voltage6;
	  float voltage7;
	  float voltage8;
	  float voltage9;
	  float voltage10;
	  float voltage11;
	  float voltage12;
	  float voltage13;
	  float voltage14;
	  float voltage15;
	  float voltage16;
	  float voltage17;
	  float voltage18;
	  float voltage19;
	  float voltage20;
	  float totalVoltage;
	  float batteryTemp1;
	  float batteryTemp2;
	  float batteryTemp3;
	  float batteryTemp4;
	  float testTemp;
	  float motorTemp;
	  float speed;
	  float current;
	  float remainEnergy;
  }data1;
CAN_TxHeaderTypeDef TxHeader[4];
CAN_RxHeaderTypeDef RxHeader[4];

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);

  TxHeader[0].DLC = 8;
  TxHeader[0].IDE = CAN_ID_EXT;
  TxHeader[0].RTR = CAN_RTR_DATA;
/*
  TxHeader[1].DLC = 8;
  TxHeader[1].IDE = CAN_ID_EXT;
  TxHeader[1].RTR = CAN_RTR_DATA;
  TxHeader[1].ExtId = 0x1DFFFFFF;

  TxHeader[2].DLC = 8;
  TxHeader[2].IDE = CAN_ID_EXT;
  TxHeader[2].RTR = CAN_RTR_DATA;
  TxHeader[2].ExtId = 0x1EFFFFFF;

  TxHeader[3].DLC = 8;
  TxHeader[3].IDE = CAN_ID_EXT;
  TxHeader[3].RTR = CAN_RTR_DATA;
  TxHeader[3].ExtId = 0x1FFFFFFF;
  */
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  HAL_ADC_Start(&hadc1);
	  testTemp_analog = HAL_ADC_GetValue(&hadc1);
	  testTemp = Thermistor(testTemp_analog);
	  HAL_ADC_Stop(&hadc1);

	  sConfig.Channel = ADC_CHANNEL_1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  HAL_ADC_Start(&hadc1);
	  motorTemp_analog = HAL_ADC_GetValue(&hadc1);
	  motorTemp = Thermistor(motorTemp_analog);
	  HAL_ADC_Stop(&hadc1);

	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	 // HAL_Delay(50);
	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader[0], dataReceive);
	  if(RxHeader[0].ExtId == 0x1CFFFFFF){
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		 // HAL_Delay(50);
		  for(i=0;i<=7;i++){
			  Package8_1[i] = dataReceive[i];
		  }
	  }
	  //HAL_Delay(50);
	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader[0], dataReceive);
	  if(RxHeader[0].ExtId == 0x1DFFFFFF){
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  //HAL_Delay(50);
		  for(i=0;i<=7;i++){
			  Package8_2[i] = dataReceive[i];
		  }
	  }
	  //HAL_Delay(50);
	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader[0], dataReceive);
	  if(RxHeader[0].ExtId == 0x1EFFFFFF){
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	 // HAL_Delay(50);
		  for(i=0;i<=7;i++){
			  Package8_3[i] = dataReceive[i];
		  }
	  }
	  //HAL_Delay(50);
	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader[0], dataReceive);
	  if(RxHeader[0].ExtId == 0x1FFFFFFF){
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  //HAL_Delay(50);
		  for(i=0;i<1;i++){
			  Package8_4[i] = dataReceive[i];
		  }
	  }
	  for(i=0;i<=7;i++){
		  Package1[i] = Thermistor(Package8_1[i]*16);
		  Package2[i] = Thermistor(Package8_2[i]*16);
		  Package3[i] = Thermistor(Package8_3[i]*16);
		  Package4[0] = Thermistor(Package8_4[0]*16);

	  }


	  gps_speed=15;

	  data1.voltage1 = Package1[0];
	  data1.voltage2 = Package1[1];
	  data1.voltage3 = Package1[2];
	  data1.voltage4 = Package1[3];
	  data1.voltage5 = Package1[4];
	  data1.voltage6 = Package1[5];
	  data1.voltage7 = Package1[6];
	  data1.voltage8 = Package1[7];
	  data1.voltage9 = Package2[0];
	  data1.voltage10 = Package2[1];
	  data1.voltage11 = Package2[2];
	  data1.voltage12 = Package2[3];
	  data1.voltage13 = Package2[4];
	  data1.voltage14 = Package2[5];
	  data1.voltage15 = Package2[6];
	  data1.voltage16 = Package2[7];
	  data1.voltage17 = Package3[0];
	  data1.voltage18 = Package3[1];
	  data1.voltage19 = Package3[2];
	  data1.voltage20 = Package3[3];
	  data1.totalVoltage = Package8_3[4]*84/255;
      data1.batteryTemp1 = Package3[5];
	  data1.batteryTemp2 = Package3[6];
	  data1.batteryTemp3 = Package3[7];
	  data1.batteryTemp4 = Package4[0];
	  data1.testTemp = testTemp;
	  Package8_4[1] = testTemp_analog/16;
	  data1.motorTemp = motorTemp;
	  Package8_4[2] = motorTemp_analog/16;
	  data1.speed = gps_speed;
	  Package8_4[3] = (uint8_t)gps_speed;
	  Package8_4[4] = 1028/16;
	  data1.current = Package8_4[3];
	  data1.remainEnergy = (int)(data1.totalVoltage/84.00)*100;
	  Package8_4[5] = (uint8_t)data1.remainEnergy;

	  TxHeader[0].ExtId = 0x0CFFFFFF;
	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader[0], Package8_1, &TxMailbox);
	  HAL_Delay(10);
	  TxHeader[0].ExtId = 0x0DFFFFFF;
	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader[0], Package8_2, &TxMailbox);
	  HAL_Delay(10);
	  TxHeader[0].ExtId = 0x0EFFFFFF;
	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader[0], Package8_3, &TxMailbox);
	  HAL_Delay(10);
	  TxHeader[0].ExtId = 0x0FFFFFFF;
	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader[0], Package8_4, &TxMailbox);
	  HAL_Delay(10);





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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x000<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x000<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
