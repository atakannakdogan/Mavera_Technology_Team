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
#include<stdio.h>
#include<string.h>
#include<stdlib.h>

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
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
typedef struct fixedStransmission
{
	uint8_t ADDH;
	uint8_t ADDL;
	uint8_t CHAN;
	unsigned char message[];
}FixedStransmission;
FixedStransmission *init_stack(int m){
	FixedStransmission *st = (FixedStransmission *)malloc(sizeof(FixedStransmission)+m*sizeof(int));
    return st;
}
int sizedata;

uint32_t TxMailbox;

uint8_t Package8_1[8];
uint8_t Package8_2[8];
uint8_t Package8_3[8];
uint8_t Package8_4[8];
uint8_t dataReceive[8];

float Package1[8];
float Package2[8];
float Package3[8];
float Package4[8];

float temp=0.00;
int i = 0;
int gps_satallite;
int gps_speed;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void sendFixedMessage( uint8_t ADDH,uint8_t ADDL, uint8_t CHAN, const void *message, const uint8_t size);
double Thermistor(uint8_t value){
 uint16_t analogValue = value*16;
 double temperature;
 temperature = log(((40950000 / analogValue) - 10000));
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

CAN_RxHeaderTypeDef RxHeader[4];
CAN_TxHeaderTypeDef TxHeader[1];

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
  MX_CAN_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);

  TxHeader[0].DLC = 8;
  TxHeader[0].IDE = CAN_ID_EXT;
  TxHeader[0].RTR = CAN_RTR_DATA;
  TxHeader[0].ExtId = 0x12345678;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader[0], dataReceive);
	  HAL_Delay(10);
	  if(RxHeader[0].ExtId == 0x0CFFFFFF){
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		  HAL_Delay(50);
		  for(i=0;i<=7;i++){
			  Package8_1[i] = dataReceive[i];
		  }
	  }
	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader[1], dataReceive);
	  HAL_Delay(10);
	  if(RxHeader[1].ExtId == 0x0DFFFFFF){
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		  for(i=0;i<=7;i++){
			  Package8_2[i] = dataReceive[i];
		  }
	  }
	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader[2], dataReceive);
	  HAL_Delay(10);
	  if(RxHeader[2].ExtId == 0x0EFFFFFF){
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  //HAL_Delay(50);
		  for(i=0;i<=7;i++){
			  Package8_3[i] = dataReceive[i];
		  }
	  }
	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader[3], dataReceive);
	  HAL_Delay(10);
	  if(RxHeader[3].ExtId == 0x0FFFFFFF){
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  //HAL_Delay(50);
		  for(i=0;i<=7;i++){
			  Package8_4[i] = dataReceive[i];
		  }
	  }
	  //temp = Thermistor(2100);

	  for(i=0;i<=7;i++){
	  		  Package1[i] = Thermistor(Package8_1[i]);
	  		  Package2[i] = Thermistor(Package8_2[i]);
	  		  Package3[i] = Thermistor(Package8_3[i]);

	  		  //Package4[i] = Thermistor(Package8_4[i]*16);
	  	  }
	  	  Package3[4] = Package8_3[4]*84/255;
	  	  Package4[0] = Thermistor(Package8_4[0]);
	  	  Package4[1] = Thermistor(Package8_4[1]);
	  	  Package4[2] = Thermistor(Package8_4[2]);
	  	  Package4[3] = Package8_4[3];
	  	  Package4[4] = Thermistor(Package8_4[4]);
	  	  Package4[5] = Package8_4[5];

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
		  data1.totalVoltage = Package3[4];
	      data1.batteryTemp1 = Package3[5];
		  data1.batteryTemp2 = Package3[6];
		  data1.batteryTemp3 = Package3[7];
		  data1.batteryTemp4 = Package4[0];
		  data1.testTemp = Package4[1];
		  data1.motorTemp = Package4[2];
		  data1.speed = Package4[3];
		  data1.current = Package4[4];
		  data1.remainEnergy = Package4[5];

	  gps_satallite = gps_satallite;

	  sizedata = sizeof(data1);
	  sendFixedMessage(0, 2, 25, &data1, sizedata);
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 5;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
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
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
void sendFixedMessage( uint8_t ADDH,uint8_t ADDL, uint8_t CHAN, const void *message, const uint8_t size){

	FixedStransmission *fixedStransmission = init_stack(size);

	fixedStransmission->ADDH = ADDH;
	fixedStransmission->ADDL = ADDL;
	fixedStransmission->CHAN = CHAN;

	memcpy(fixedStransmission->message,(unsigned char*)message,size);

	HAL_UART_Transmit(&huart3, (uint8_t *)fixedStransmission, size+3, 1000);
	HAL_Delay(200);

	free(fixedStransmission);
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
