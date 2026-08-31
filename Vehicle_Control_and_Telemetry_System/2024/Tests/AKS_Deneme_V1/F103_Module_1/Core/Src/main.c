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
#include <stdlib.h>

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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char datatxt[5];
float data2;
double data3;
int virgulOncesi;
int virgulSonrasi;
uint8_t Package8_1[8];
uint8_t Package8_2[8];
uint8_t Package8_3[8];
uint8_t Package8_4[8];

float Package81_1[8];
float Package81_2[8];
float Package81_3[8];
float Package81_4[8];

uint8_t dataReceive[8];
char sdata[20];
float Package1[8];
float Package2[8];
float Package3[8];
float Package4[8];
int i;
int satallite = 8;
uint8_t nextionEnd[3] = {0xFF,0xFF,0xFF};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void sendNextion();
uint8_t def_msb(float value);
uint8_t def_lsb(float value);
double Thermistor(uint8_t value);
/*
float getVoltage(uint8_t value){
	value = value*3.3/255;

}*/

void txtNextion(char *location, float data);
void numNextion(char *location, int data);
void picNextion(char *location, int data);
void ppicNextion(char *location, int data);
void progbarNextion(char *location, int data);
void inttxtNextion(char *location, int data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef RxHeader[0];


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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	  //HAL_Delay(50);
	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader[0], dataReceive);
	  if(RxHeader[0].ExtId == 0x0CFFFFFF){
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		  HAL_Delay(50);
		  for(i=0;i<=7;i++){
			  Package8_1[i] = dataReceive[i];
		  }
	  }
	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader[0], dataReceive);
	  if(RxHeader[0].ExtId == 0x0DFFFFFF){
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  //HAL_Delay(50);
		  for(i=0;i<=7;i++){
			  Package8_2[i] = dataReceive[i];
		  }
	  }
	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader[0], dataReceive);
	  if(RxHeader[0].ExtId == 0x0EFFFFFF){
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  //HAL_Delay(50);
		  for(i=0;i<=7;i++){
			  Package8_3[i] = dataReceive[i];
		  }
	  }
	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader[0], dataReceive);
	  if(RxHeader[0].ExtId == 0x0FFFFFFF){
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	 // HAL_Delay(50);
		  for(i=0;i<=7;i++){
			  Package8_4[i] = dataReceive[i];
		  }
	  }
	  //HAL_Delay(50);
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
/*
	  while(Package1[0] && Package2[0] && Package2[0] && Package2[0] == "nan"){

	  }
	  */
	  /*
		  for(int i = 0;i<8;i++){
			  int x = (int)def_msb(Package81_1[i]);
			  int y = (int)def_lsb(Package81_1[i]);
			  Package1[i] = (float)x+(float)(y/100.00);
			  x=0;
			  y=0;

			  x = (int)def_msb(Package81_2[i]);
			  y = (int)def_lsb(Package81_2[i]);
			  Package2[i] = (float)x+(float)(y/100.00);
			  x=0;
			  y=0;

			  x = (int)def_msb(Package81_3[i]);
			  y = (int)def_lsb(Package81_3[i]);
			  Package3[i] = (float)x+(float)(y/100.00);
			  x=0;
			  y=0;

			  x = (int)def_msb(Package81_4[i]);
			  y = (int)def_lsb(Package81_4[i]);
			  Package4[i] = (float)x+(float)(y/100.00);
			  x=0;
			  y=0;
		  }
		  */
	  sendNextion();

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
  huart1.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
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
void sendNextion(){
 HAL_Delay(10);
 picNextion("p1",(int)Package4[3]);
 numNextion("n0",(int)Package4[3]);
 txtNextion("t1",Package3[5]);
 txtNextion("t2",Package3[6]);
 txtNextion("t3",Package3[7]);
 txtNextion("t4",Package4[0]);
 txtNextion("t5",Package4[1]);
 if (Package3[5] >= 50 || Package3[6] >= 50 || Package3[7] >= 50 || Package4[0] >= 50 || Package4[1] >= 50) {
     int alert_pic = 68;
     picNextion("p2",alert_pic);
   }
 else {
     int alert_pic = 67;
     picNextion("p2",alert_pic);
   }
 txtNextion("t6",Package4[2]);
 txtNextion("t7",Package1[0]);
 txtNextion("t8",Package1[1]);
 txtNextion("t9",Package1[2]);
 txtNextion("t10",Package1[3]);
 txtNextion("t11",Package1[4]);
 txtNextion("t12",Package1[5]);
 txtNextion("t13",Package1[6]);
 txtNextion("t14",Package1[7]);
 txtNextion("t15",Package2[0]);
 txtNextion("t16",Package2[1]);
 txtNextion("t17",Package2[2]);
 txtNextion("t18",Package2[3]);
 txtNextion("t19",Package2[4]);
 txtNextion("t20",Package2[5]);
 txtNextion("t21",Package2[6]);
 txtNextion("t22",Package2[7]);
 txtNextion("t23",Package3[0]);
 txtNextion("t24",Package3[1]);
 txtNextion("t25",Package3[2]);
 txtNextion("t26",Package3[3]);
 txtNextion("t27",(float)Package3[4]);
 txtNextion("t28",Package4[4]);
 inttxtNextion("t29",(int)Package4[5]);
 if (Package4[5] <= 100 && Package4[5] > 50) {
     int ppic = 66;
     ppicNextion("j0", ppic);
   }
 else if (Package4[5] <= 60 && Package4[5] > 40) {
     int ppic = 65;
     ppicNextion("j0", ppic);
   }
 else if (Package4[5] <= 40 && Package4[5] > 0) {
     int ppic = 64;
     ppicNextion("j0", ppic);
   }
 progbarNextion("j0", (int)Package4[5]);
 inttxtNextion("t30", (int)satallite);
 picNextion("p3",70); // SD kart
 HAL_Delay(10);
 //----------------------------NextionEnd---------------------------------

}
uint8_t def_msb(float value){
	virgulOncesi = (int)value; // Virgül öncesi kısmı almak için sayının tam kısmını elde ediyoruz
	return virgulOncesi;
}
uint8_t def_lsb(float value){
	virgulOncesi = (int)value; // Virgül öncesi kısmı almak için sayının tam kısmını elde ediyoruz
	virgulSonrasi = (int)((value - virgulOncesi) * 100); // Virgül sonrası kısmını elde etmek için float değerinden virgül öncesi kısmını çıkarıp 1000 ile çarpmak yeterlidir
	return virgulSonrasi;
}
double Thermistor(uint8_t value){
 uint16_t analogValue = value*16;
 double temperature;
 temperature = log(((40950000 / analogValue) - 10000));
 temperature = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temperature * temperature)) * temperature);
 temperature = temperature - 273.15;
 return temperature;
}
/*
float getVoltage(uint8_t value){
	value = value*3.3/255;

}*/

void txtNextion(char *location, float data){

	char text[50];
	//char sdata[8];
	//sprintf(sdata,"%f",data);
	virgulOncesi = def_msb(data); // Virgül öncesi kısmı almak için sayının tam kısmını elde ediyoruz
	virgulSonrasi = def_lsb(data);
	//data2 = virgulOncesi + (virgulSonrasi/100.0);
/*
	data3 = round(data * 100) / 100;
	sprintf(datatxt,"%.2f",data3);
	*/
/*
	data3 = (int)(data*100);
	data2 = (float)(data3/100);
*/
	if(virgulSonrasi <10){
		sprintf(datatxt,"%d.0%d",virgulOncesi,virgulSonrasi);
	}
	else{
		sprintf(datatxt,"%d.%d",virgulOncesi,virgulSonrasi);
	}

	int length = sprintf(text,"%s.txt=\"%s\"",location, datatxt);
	HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 1000);
	HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 100);
	//memset(text, '\0', sizeof(text));
}
void numNextion(char *location, int data){

	char text[50];

	int length = sprintf(text,"%s.val=%d",location,data);
	HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 100);
	HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 100);
	//memset(text, '\0', sizeof(text));
}
void picNextion(char *location, int data){
	char text[50];

	int length = sprintf(text,"%s.pic=%d",location,data);
	HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 100);
	HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 100);
	//memset(text, '\0', sizeof(text));

}
void ppicNextion(char *location, int data){
	char text[50];

	int length = sprintf(text,"%s.ppic=%d",location,data);
	HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 100);
	HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 100);
	//memset(text, '\0', sizeof(text));
}
void progbarNextion(char *location, int data){
	char text[50];

	int length = sprintf(text,"%s.val=%d",location,data);
	HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 100);
	HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 100);
	//memset(text, '\0', sizeof(text));
}
void inttxtNextion(char *location, int data){

	char text[50];
	//sprintf(sdata,"%f",data);
	int length = sprintf(text,"%s.txt=\"%d\"",location,data);
	HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 100);
	HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 100);
	//memset(text, '\0', sizeof(text));
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
