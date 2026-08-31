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
#include "main.h"
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

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
uint16_t adc1, adc2;
int count=0;
int counter=0;
int i=0;
int flag=0;
int j=0;
int b_con;
float temp1;
float temp2;
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
float maxvalue;
float minvalue;
int maxindex;
int minindex;
float voltages[10]={};

uint32_t TxMailbox;
float sendData[2];
float voltages[10];
float temps[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
double Thermistor(uint16_t analogValue){
    // Sıfıra bölme hatasını önlemek için kontrol ekleyin
    if (analogValue == 0) {
        return -273.15; // Hata durumunda düşük bir sıcaklık döndürün
    }

    // ADC çözünürlüğü ve direnç hesaplaması
    double resistance = ((40950000.0 / analogValue) - 10000);
    double temperature;

    // Direnç logaritmasını alın
    temperature = log(resistance);

    // Sıcaklık hesaplaması (Steinhart-Hart denklemi)
    temperature = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temperature * temperature)) * temperature);
    temperature = temperature - 273.15;

    return temperature;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef ret;
  ret = HAL_ADCEx_Calibration_Start(&hadc1);
  if(ret != HAL_OK){
	  Error_Handler();
  }
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

  HAL_CAN_Start(&hcan);
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.RTR = CAN_RTR_DATA;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);

	  // Voltage Measurement
	 	  switch(count){

	 	  	  case 0:
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	//S3
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);	//S2
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);	//S1
	 	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); //S0
	 	  		  HAL_Delay(100);
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		  HAL_ADC_Start(&hadc1);
	 	  		  HAL_ADC_PollForConversion(&hadc1, 100);
	 	  		  adc1= HAL_ADC_GetValue(&hadc1);
	 	  		  voltage1= ((adc1*3.282)/4095.0)*(5.012);
	 	  		  HAL_Delay(500);
	 	  		/*  HAL_ADC_Stop(&hadc1);
	 	  		  HAL_Delay(50);
	 	  		  Temperature_Read();
	 	  		  */
	 	  		  break;

	 	  	  case 1:
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	//S3
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);	//S2
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);	//S1
	 	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1); //S0
	 	  		  HAL_Delay(10);
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		  HAL_ADC_Start(&hadc1);
	 	   		  HAL_ADC_PollForConversion(&hadc1, 100);
	 	  		  adc1= HAL_ADC_GetValue(&hadc1);
	 	  		  voltage2= ((adc1*3.282)/4095.0)*(4.981);
	 	  		  HAL_Delay(500);
	 	  		 /* HAL_ADC_Stop(&hadc1);
	 	  		  HAL_Delay(50);
	 	  		  Temperature_Read();
	 	  		  */
	 	  		  break;

	 	  	  case 2:
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	//S3
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);	//S2
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);	//S1
	 	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); //S0
	 	  		  HAL_Delay(10);
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		  HAL_ADC_Start(&hadc1);
	 	   		  HAL_ADC_PollForConversion(&hadc1, 100);
	 	  		  adc1= HAL_ADC_GetValue(&hadc1);
	 	  		  voltage3= ((adc1*3.282)/4095.0)*(4.981);
	 	  		  HAL_Delay(500);
	 	  		/*  HAL_ADC_Stop(&hadc1);
	 	  		  HAL_Delay(50);
	 	  		  Temperature_Read();
	 	  		  */
	 	  		  break;

	 	  	  case 3:
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	//S3
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);	//S2
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);	//S1
	 	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1); //S0
	 	  		  HAL_Delay(10);
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		  HAL_ADC_Start(&hadc1);
	 	   		  HAL_ADC_PollForConversion(&hadc1, 100);
	 	  		  adc1= HAL_ADC_GetValue(&hadc1);
	 	  		  voltage4= ((adc1*3.282)/4095.0)*(4.952);
	 	  		  HAL_Delay(500);
	 	  		/*  HAL_ADC_Stop(&hadc1);
	 	  		  HAL_Delay(50);
	 	  		  Temperature_Read();
	 	  		  */
	 	  		  break;

	 	  	  case 4:
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	//S3
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);	//S2
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);	//S1
	 	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); //S0
	 	  		  HAL_Delay(10);
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		  HAL_ADC_Start(&hadc1);
	 	   		  HAL_ADC_PollForConversion(&hadc1, 100);
	 	  		  adc1= HAL_ADC_GetValue(&hadc1);
	 	  		  voltage5= ((adc1*3.282)/4095.0)*(4.993);
	 	  		  HAL_Delay(500);
	 	  		/*  HAL_ADC_Stop(&hadc1);
	 	  		  HAL_Delay(50);
	 	  		  Temperature_Read();
	 	  		  */
	 	  		  break;

	 	  	  case 5:
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	//S3
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);	//S2
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);	//S1
	 	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1); //S0
	 	  		  HAL_Delay(10);
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		  HAL_ADC_Start(&hadc1);
	 	   		  HAL_ADC_PollForConversion(&hadc1, 100);
	 	  		  adc1= HAL_ADC_GetValue(&hadc1);
	 	  		  voltage6= ((adc1*3.282)/4095.0)*(4.946);
	 	  		  HAL_Delay(500);
	 	  		/*  HAL_ADC_Stop(&hadc1);
	 	  		  HAL_Delay(50);
	 	  		  Temperature_Read();
	 	  		  */
	 	  		  break;

	 	  	  case 6:
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	//S3
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);	//S2
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);	//S1
	 	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); //S0
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		  HAL_Delay(10);
	 	  		  HAL_ADC_Start(&hadc1);
	 	   		  HAL_ADC_PollForConversion(&hadc1, 100);
	 	  		  adc1= HAL_ADC_GetValue(&hadc1);
	 	  		  voltage7= ((adc1*3.282)/4095.0)*(9.656);
	 	  		  HAL_Delay(500);
	 	  		/*  HAL_ADC_Stop(&hadc1);
	 	  		  HAL_Delay(50);
	 	  		  Temperature_Read();
	 	  		  */
	 	  		  break;

	 	  	  case 7:
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	//S3
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);	//S2
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);	//S1
	 	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1); //S0
	 	  		  HAL_Delay(10);
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		  HAL_ADC_Start(&hadc1);
	 	   		  HAL_ADC_PollForConversion(&hadc1, 100);
	 	  		  adc1= HAL_ADC_GetValue(&hadc1);
	 	  		  voltage8= ((adc1*3.282)/4095.0)*(9.974);
	 	  		  HAL_Delay(500);
	 	  		/*  HAL_ADC_Stop(&hadc1);
	 	  		  HAL_Delay(50);
	 	  		  Temperature_Read();
	 	  		  */
	 	  		  break;

	 	  	  case 8:
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);	//S3
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);	//S2
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);	//S1
	 	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); //S0
	 	  		  HAL_Delay(10);
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		  HAL_ADC_Start(&hadc1);
	 	   		  HAL_ADC_PollForConversion(&hadc1, 100);
	 	  		  adc1= HAL_ADC_GetValue(&hadc1);
	 	  		  voltage9= ((adc1*3.282)/4095.0)*(9.875);
	 	  		  HAL_Delay(500);
	 	  		/*  HAL_ADC_Stop(&hadc1);
	 	  		  HAL_Delay(50);
	 	  		  Temperature_Read();
	 	  		  */
	 	  		  break;

	 	  	  case 9:
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);	//S3
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);	//S2
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);	//S1
	 	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1); //S0
	 	  		  HAL_Delay(10);
	 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		  HAL_ADC_Start(&hadc1);
	 	   		  HAL_ADC_PollForConversion(&hadc1, 100);
	 	  		  adc1= HAL_ADC_GetValue(&hadc1);
	 	  		  voltage10= ((adc1*3.282)/4095.0)*(9.875);
	 	  		  HAL_Delay(500);
	 	  		/*  HAL_ADC_Stop(&hadc1);
	 	  		  HAL_Delay(50);
	 	  		  Temperature_Read();
	 	  		  */
	 	  		  break;

	 	  	 case 10:
	 			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);	//S3
	 			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);	//S2
	 			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);	//S1
	 			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); //S0
	 			  HAL_Delay(10);
	 			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 			  HAL_ADC_Start(&hadc1);
	 			  adc1 = HAL_ADC_GetValue(&hadc1);
	 			  temp1 = Thermistor(adc1);
	 			  HAL_Delay(500);
	 		 /*  HAL_ADC_Stop(&hadc1);
	 			 HAL_Delay(50);
	 	   	  Temperature_Read();
	 							  */
	 			 break;

	 	  	 case 11:
	 	  		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);	//S3
	 	  		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);	//S2
	 	  		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);	//S1
	 	  		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1); //S0
	 	  		 HAL_Delay(10);
	 	  		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);	//E
	 	  		 HAL_ADC_Start(&hadc1);
	 	  		 adc1 = HAL_ADC_GetValue(&hadc1);
	 	  		 temp2 = Thermistor(adc1);
	 	  		  		  HAL_Delay(500);
	 	  		  		/*  HAL_ADC_Stop(&hadc1);
	 	  		  		  HAL_Delay(50);
	 	  		  		  Temperature_Read();
	 	  		  		  */
	 	  		  		  break;
	 	  	  }

	 	 float voltages[10]= {voltage1,voltage2,voltage3,voltage4,voltage5,voltage6,voltage7,voltage8,voltage9,voltage10};
	 	 	  	  	 maxvalue=0;
	 	 	  	  	 maxindex=0;
	 	 	  	  	 minvalue=5;
	 	 	  	  	 minindex=0;
	 	 	  	  	  for(i=0; i<10; i++){
	 	 	  	  		  if(maxvalue < voltages[i]){
	 	 	  	  			  maxvalue = voltages[i];
	 	 	  	  			  maxindex = i+1;
	 	 	  	  		  }
	 	 	  	  		  if(minvalue > voltages[i]){
	 	 	  	  		  	  minvalue = voltages[i];
	 	 	  	  		  	  minindex = i+1;
	 	 	  	   		  }
	 	 	  	  	  	  	  	  }

	 	 if(maxvalue-minvalue > 0.1 && maxvalue > 3.5 && b_con==0){
	 	 	  	  		  flag = maxindex -1;
	 	 	  	  	  }

	 	 // CANBUS
	 	 TxHeader.ExtId = 0x0FFFFFF0;
	 	 sendData[0] = voltage1;
	 	 sendData[1] = voltage2;
	 	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, sendData, &TxMailbox);
	 	 HAL_Delay(10);

	 	 TxHeader.ExtId = 0x0FFFFFF1;
	 	 sendData[0] = voltage3;
	 	 sendData[1] = voltage4;
	 	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, sendData, &TxMailbox);
	 	 HAL_Delay(10);

	 	 TxHeader.ExtId = 0x0FFFFFF2;
	 	 sendData[0] = voltage5;
	 	 sendData[1] = voltage6;
	 	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, sendData, &TxMailbox);
	 	 HAL_Delay(10);

	 	 TxHeader.ExtId = 0x0FFFFFF3;
	 	 sendData[0] = voltage7;
	 	 sendData[1] = voltage8;
	 	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, sendData, &TxMailbox);
	 	 HAL_Delay(10);

	 	 TxHeader.ExtId = 0x0FFFFFF4;
	 	 sendData[0] = voltage9;
	 	 sendData[1] = voltage10;
	 	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, sendData, &TxMailbox);
	 	 HAL_Delay(10);

	 	 TxHeader.ExtId = 0x0FFFFFF5;
	 	 sendData[0] = temp1;
	 	 sendData[1] = temp2;
	 	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, sendData, &TxMailbox);
	 	 HAL_Delay(10);


	 	 // Balancing Mux
	 	switch(flag){

	 	 	  	  	  case 0:
	 	 	  	  	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0); //B3
	 	 	  	  	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);	//B2
	 	 	  	  	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);	//B1
	 	 	  	  	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);	//B0
	 	 	  	  	   HAL_Delay(10);
	 	 	  	  	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); //E
	 	 	  	  	   break;

	 	 	  	  	  	  		  		  case 1:
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);	//B3
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);	//B2
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);	//B1
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);	//B0
	 	 	  	  	  	  		  			  HAL_Delay(10);
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);	//E
	 	 	  	  	  	  		  			  break;

	 	 	  	  	  	  		  		  case 2:
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);	//B3
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);	//B2
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);	//B1
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);	//B0
	 	 	  	  	  	  		  			  HAL_Delay(10);
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);	//E
	 	 	  	  	  	  		  			  break;

	 	 	  	  	  	  		  		  case 3:
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);	//B3
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);	//B2
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);	//B1
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);	//B0
	 	 	  	  	  	  		  			  HAL_Delay(10);
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);	//E
	 	 	  	  	  	  		  			  break;

	 	 	  	  	  	  		  		  case 4:
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);	//B3
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);	//B2
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);	//B1
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);	//B0
	 	 	  	  	  	  		  			  HAL_Delay(10);
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);	//E
	 	 	  	  	  	  		  			  break;

	 	 	  	  	  	  		  		  case 5:
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);	//B3
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);	//B2
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);	//B1
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);	//B0
	 	 	  	  	  	  		  			  HAL_Delay(10);
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);	//E
	 	 	  	  	  	  		  			  break;

	 	 	  	  	  	  		  		  case 6:
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);	//B3
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);	//B2
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);	//B1
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);	//B0
	 	 	  	  	  	  		  			  HAL_Delay(10);
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);	//E
	 	 	  	  	  	  		  			  break;

	 	 	  	  	  	  		  		  case 7:
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);	//B3
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);	//B2
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);	//B1
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);	//B0
	 	 	  	  	  	  		  			  HAL_Delay(10);
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);	//E
	 	 	  	  	  	  		  			  break;

	 	 	  	  	  	  		  		  case 8:
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);	//B3
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);	//B2
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);	//B1
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);	//B0
	 	 	  	  	  	  		  			  HAL_Delay(10);
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);	//E
	 	 	  	  	  	  		  			  break;

	 	 	  	  	  	  		  		  case 9:
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);	//B3
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);	//B2
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);	//B1
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);	//B0
	 	 	  	  	  	  		  			  HAL_Delay(10);
	 	 	  	  	  	  		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);	//E
	 	 	  	  	  	  		  			  break;
	 	 	  	  	  	  		  		  }

	 	 	  	HAL_Delay(100);
	 	 	  	count++;
	 	 	  	counter++;
	 	 	  	b_con = counter % 100;
	 	 	  	if(count>11){
	 	 	  	count=0;
	 	 	  	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB3 PB4 PB5 PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
