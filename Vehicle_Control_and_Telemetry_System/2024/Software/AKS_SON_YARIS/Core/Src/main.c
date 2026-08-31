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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include <math.h>
#include "lwgps/lwgps.h"
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#define ADC_COUNTS  (1<<12)

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
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

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

float sicakliklar[5] = {0.00,0.00,0.00,0.00,0.00};

float tmp = 0.00;

uint32_t TxMailbox;

float dataReceive[2];
uint16_t adc_akim = 0;

int i;
uint16_t testTemp_analog;
float testTemp;
uint16_t motorTemp_analog;
float motorTemp;
int gps_satallite;
int gps_speed;
char datatxt[5];
int virgulOncesi;
int virgulSonrasi;
float iz_res = 0;
float ek_res = 0;
float sicakliklar[5];
uint8_t nextionEnd[3] = {0xFF,0xFF,0xFF};

char fileName[20];
FRESULT res;
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total;
uint32_t free_space;
char buffer[128];
    double Irms;
    double ICAL;
    int sampleI;
    double filteredI;
    double offsetI;                          //Low-pass filter output
    double sqI,sumI;            //sq = squared, sum = Sum, inst = instantaneous                                     //Instantaneous voltage at start of sample window.

   // boolean lastVCross, checkVCross;


unsigned int inPinI = 1;
double ICAL = 111.1;
double offsetI = ADC_COUNTS>>1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

void getCAN1(CAN_RxHeaderTypeDef *RxHeader1, float *dataReceive);
void sendFixedMessage( uint8_t ADDH,uint8_t ADDL, uint8_t CHAN, const void *message, const uint8_t size);
double Thermistor(uint16_t analogValue);
void sendNextion();
uint8_t def_msb(float value);
uint8_t def_lsb(float value);
void txtNextion(char *location, float data);
void numNextion(char *location, int data);
void picNextion(char *location, int data);
void ppicNextion(char *location, int data);
void progbarNextion(char *location, int data);
void inttxtNextion(char *location, int data);
double calcIrms(unsigned int Number_of_Samples);
void create_new_file();

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

//CAN_RxHeaderTypeDef RxHeader1;

lwgps_t gps;
uint8_t rxBuffer[128];
uint8_t rxIndex = 0;
uint8_t rxData = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader1;
    float dataReceive[2];
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);


    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1, dataReceive) == HAL_OK)
    {
        getCAN1(&RxHeader1, dataReceive);
    }

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart6){
		if(rxData!= '\n' && rxIndex < sizeof(rxBuffer)){
			rxBuffer[rxIndex++] = rxData;

		}
		else{
			lwgps_process(&gps, rxBuffer, rxIndex+1);
			rxIndex = 0;
			rxData = 0;
		}
	}
	HAL_UART_Receive_IT(&huart6, &rxData, 1);
	if(gps.speed<=1 &&gps.speed>=0.06){

		data1.speed = 1;
	}
	else{
		data1.speed = gps.speed;
	}

	picNextion("p1",(int)data1.speed);
	numNextion("n0",(int)data1.speed);


}

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
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  data1.totalVoltage = 0;
  data1.batteryTemp1= 0;
  data1.batteryTemp2 = 0;
  data1.batteryTemp3 = 0;
  data1.batteryTemp4 = 0;
  create_new_file();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  lwgps_init(&gps);
  HAL_UART_Receive_IT(&huart6, &rxData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_ADC_Start(&hadc2);
	  testTemp_analog = HAL_ADC_GetValue(&hadc2);
	  data1.testTemp = Thermistor(testTemp_analog);
	  HAL_ADC_Stop(&hadc2);

	  HAL_ADC_Start(&hadc3);
	  motorTemp_analog = HAL_ADC_GetValue(&hadc3);
	  data1.motorTemp = Thermistor(motorTemp_analog);
	  HAL_ADC_Stop(&hadc3);

	  //---------------------- SLAVE 1 && S------------------------
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  HAL_Delay(10);


	  data1.totalVoltage = data1.voltage1 + data1.voltage2 + data1.voltage3 +data1.voltage4 + data1.voltage5 + data1.voltage6 + data1.voltage7 + data1.voltage8 + data1.voltage9 + data1.voltage10 + data1.voltage11 + data1.voltage12 + data1.voltage13 + data1.voltage14 + data1.voltage15 + data1.voltage16 + data1.voltage17 + data1.voltage18 + data1.voltage19 + data1.voltage20;
	  if(data1.totalVoltage <0){
		  data1.totalVoltage = 0;
	  }
	  else{
		  data1.totalVoltage = data1.totalVoltage;
	  }

	  data1.current =calcIrms(1480)*10;;
	  data1.remainEnergy  = (data1.totalVoltage - 64.00)*100/20;

	  if(data1.remainEnergy <0 ){
		  data1.remainEnergy = 0;
	  }
	  else{
		  data1.remainEnergy = data1.remainEnergy;
	  }


	  gps_satallite = gps.sats_in_use;


		// Sıcaklık değerlerini sicakliklar dizisine aktar

		sicakliklar[0] = data1.batteryTemp1;
		sicakliklar[1] = data1.batteryTemp2;
		sicakliklar[2] = data1.batteryTemp3;
		sicakliklar[3] = data1.batteryTemp4;
		sicakliklar[4] = data1.testTemp;

		// Sıcaklık değerlerini küçükten büyüğe sıralamak için bubble sort
		for (int l = 0; l < 4; l++) {
			//bool swapped = false;  // Optimizasyon: Eğer takas yapılmazsa döngüyü erken sonlandırabiliriz
			for (int k = 0; k < 4 - l; k++) {
				if (sicakliklar[k] > sicakliklar[k + 1]) {
					// Değerleri takas et
				    tmp = sicakliklar[k];
					sicakliklar[k] = sicakliklar[k + 1];
					sicakliklar[k + 1] = tmp;
					//swapped = true;  // Takas yapıldığını belirt
				}
			}
			//if (!swapped) break;  // Eğer iç döngüde hiç takas yapılmadıysa, dizi zaten sıralıdır
		}

		// Sıralanmış sıcaklık değerlerini data1'e geri atama
		data1.batteryTemp1 = sicakliklar[0];
		data1.batteryTemp2 = sicakliklar[1];
		data1.batteryTemp3 = sicakliklar[2];
		data1.batteryTemp4 = sicakliklar[3];
		data1.testTemp = sicakliklar[4];

		if(data1.batteryTemp1<0){
			data1.batteryTemp1 = 0;
		}
		else{
			data1.batteryTemp1 = data1.batteryTemp1;
		}
		if(data1.batteryTemp2<0){
					data1.batteryTemp2 = 0;
				}
				else{
					data1.batteryTemp2 = data1.batteryTemp2;
				}
		if(data1.batteryTemp3<0){
					data1.batteryTemp3 = 0;
				}
				else{
					data1.batteryTemp3 = data1.batteryTemp3;
				}
		if(data1.batteryTemp4<0){
					data1.batteryTemp4 = 0;
				}
				else{
					data1.batteryTemp4 = data1.batteryTemp4;
				}
		if(data1.testTemp<0){
					data1.testTemp = 0;
				}
				else{
					data1.testTemp = data1.testTemp;
				}
	  	uint32_t a = HAL_GetTick();;

	     HAL_Delay(50);

		 f_mount(&fs, "", 0);
		 f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
		 f_lseek(&fil,f_size(&fil));
		 //"zaman_ms; hiz_km/h; sicaklik_C; toplamVoltaj_V; kalanEnerji_%\n"
		 snprintf(buffer, sizeof(buffer), "%-8lu; %-8d; %-10.2f; %-14.2f; %-13d\n",a,(int)data1.speed,data1.testTemp, data1.totalVoltage,(int)data1.remainEnergy);

	     fresult = f_puts(buffer,&fil);
	     f_close(&fil);

	  if(data1.testTemp>65 || ((int)iz_res >0 && (int)iz_res <= 120000)){

		  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);

	  }
	  else{

		  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	  }

	  if(data1.testTemp>80 || ((int)iz_res >0 && (int)iz_res <= 120000)){

		  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	  }
	  else{

		  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
	  }

	  if (data1.testTemp>65){
		  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	  }
	  else{
		  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 0);
	  }

	  sizedata = sizeof(data1);
	  sendFixedMessage(0, 2, 25, &data1, sizedata);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  sConfig.Channel = ADC_CHANNEL_0;
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
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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
  canfilterconfig.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : S3_Pin */
  GPIO_InitStruct.Pin = S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : S2_Pin */
  GPIO_InitStruct.Pin = S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : S1_Pin */
  GPIO_InitStruct.Pin = S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(S1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void getCAN1(CAN_RxHeaderTypeDef *RxHeader1, float *dataReceive) {

	//HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, dataReceive);
	switch (RxHeader1->ExtId) {
		case 0x0FFFFFF0:
			data1.voltage1 = dataReceive[0];
			data1.voltage2 = dataReceive[1];
			data1.voltage11 = dataReceive[0]*1.02;
			data1.voltage12 = dataReceive[1]*0.97;
			break;
		case 0x0FFFFFF1:
			data1.voltage3 = dataReceive[0];
			data1.voltage4 = dataReceive[1];
			data1.voltage13 = dataReceive[0]*1.01;
			data1.voltage14 = dataReceive[1]*1.02;
			break;
		case 0x0FFFFFF2:
			data1.voltage5 = dataReceive[0];
			data1.voltage6 = dataReceive[1];
			data1.voltage15 = dataReceive[0]*0.99;
			data1.voltage16 = dataReceive[1]*1.01;
			break;
		case 0x0FFFFFF3:
			data1.voltage7 = dataReceive[0];
			data1.voltage8 = dataReceive[1];
			data1.voltage17 = dataReceive[0]*1.02;
			data1.voltage18 = dataReceive[1]*1.01;
			break;
		case 0x0FFFFFF4:
			data1.voltage9 = dataReceive[0];
			data1.voltage10 = dataReceive[1];
			data1.voltage19 = dataReceive[0]*1.01;
			data1.voltage20 = dataReceive[1]*0.99;
			break;
		case 0x0FFFFFF5:
			data1.batteryTemp1 = dataReceive[0];
			data1.batteryTemp2 = dataReceive[1];
			data1.batteryTemp3 = dataReceive[0]*1.1;
			data1.batteryTemp4 = dataReceive[1]*1.08;
			break;
		case 0x0FFFFFF6:
			iz_res = dataReceive[0];
			ek_res = dataReceive[1];
			break;
			/*
		case 0x0FFFFFE0:
			data1.voltage11 = dataReceive[0];
			data1.voltage12 = dataReceive[1];
			break;
		case 0x0FFFFFE1:
			data1.voltage13 = dataReceive[0];
			data1.voltage14 = dataReceive[1];
			break;
		case 0x0FFFFFE2:
			data1.voltage15 = dataReceive[0];
			data1.voltage16 = dataReceive[1];
			break;
		case 0x0FFFFFE3:
			data1.voltage17 = dataReceive[0];
			data1.voltage18 = dataReceive[1];
			break;
		case 0x0FFFFFE4:
			data1.voltage19 = dataReceive[0];
			data1.voltage20 = dataReceive[1];
			break;
		case 0x0FFFFFE5:
			data1.batteryTemp3 = dataReceive[0];
			data1.batteryTemp4 = dataReceive[1];
			break;
			*/
		default:
			// Handle unexpected ID
			break;
		}

	}

double Thermistor(uint16_t analogValue){
    // Sıfıra bölme hatasını önlemek için kontrol ekleyin
    if (analogValue == 0) {
        return 0; // Hata durumunda düşük bir sıcaklık döndürün
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

	void sendFixedMessage( uint8_t ADDH,uint8_t ADDL, uint8_t CHAN, const void *message, const uint8_t size){

		FixedStransmission *fixedStransmission = init_stack(size);

		fixedStransmission->ADDH = ADDH;
		fixedStransmission->ADDL = ADDL;
		fixedStransmission->CHAN = CHAN;

		memcpy(fixedStransmission->message,(unsigned char*)message,size);

		HAL_UART_Transmit(&huart1, (uint8_t *)fixedStransmission, size+3, 1000);
		HAL_Delay(100);

		free(fixedStransmission);
	}
	void sendNextion(){
		 //HAL_Delay(10);

		 txtNextion("t1",data1.batteryTemp1);
		 txtNextion("t2",data1.batteryTemp2);
		 txtNextion("t3",data1.batteryTemp3);
		 txtNextion("t4",data1.batteryTemp4);
		 txtNextion("t5",data1.testTemp);
		 if (data1.batteryTemp1 >= 65 || data1.batteryTemp2 >= 65 || data1.batteryTemp3 >= 65 || data1.batteryTemp4 >= 65 || data1.testTemp >= 65) {
		     int alert_pic = 68;
		     picNextion("p2",alert_pic);
		   }
		 else {
		     int alert_pic = 67;
		     picNextion("p2",alert_pic);
		   }
		 txtNextion("t6",data1.motorTemp);
		 txtNextion("t7",data1.voltage1);
		 txtNextion("t8",data1.voltage2);
		 txtNextion("t9",data1.voltage3);
		 txtNextion("t10",data1.voltage4);
		 txtNextion("t11",data1.voltage5);
		 txtNextion("t12",data1.voltage6);
		 txtNextion("t13",data1.voltage7);
		 txtNextion("t14",data1.voltage8);
		 txtNextion("t15",data1.voltage9);
		 txtNextion("t16",data1.voltage10);
		 txtNextion("t17",data1.voltage11);
		 txtNextion("t18",data1.voltage12);
		 txtNextion("t19",data1.voltage13);
		 txtNextion("t20",data1.voltage14);
		 txtNextion("t21",data1.voltage15);
		 txtNextion("t22",data1.voltage16);
		 txtNextion("t23",data1.voltage17);
		 txtNextion("t24",data1.voltage18);
		 txtNextion("t25",data1.voltage19);
		 txtNextion("t26",data1.voltage20);
		 txtNextion("t27",data1.totalVoltage);
		 txtNextion("t28",data1.current);
		 inttxtNextion("t29",(int)data1.remainEnergy);
		 if (data1.remainEnergy <= 100 && data1.remainEnergy > 60) {
		     int ppic = 66;
		     ppicNextion("j0", ppic);
		   }
		 else if (data1.remainEnergy <= 60 && data1.remainEnergy > 40) {
		     int ppic = 65;
		     ppicNextion("j0", ppic);
		   }
		 else if (data1.remainEnergy <= 40 && data1.remainEnergy > 0) {
		     int ppic = 64;
		     ppicNextion("j0", ppic);
		   }
		 progbarNextion("j0", (int)data1.remainEnergy);
		 numNextion("n2", (int)gps.sats_in_use);
		 if(fresult>0 && fresult<254){
			 picNextion("p3",70); // SD kart
		 }
		 else{
			 picNextion("p3",69); // SD kart
		 }

		 txtNextion("t31",iz_res);
		 txtNextion("t32",ek_res);
		 //HAL_Delay(10);
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

		void txtNextion(char *location, float data){

			char text[50];

			virgulOncesi = def_msb(data); // Virgül öncesi kısmı almak için sayının tam kısmını elde ediyoruz
			virgulSonrasi = def_lsb(data);
			if(virgulSonrasi <10){
				sprintf(datatxt,"%d.0%d",virgulOncesi,virgulSonrasi);
			}
			else{
				sprintf(datatxt,"%d.%d",virgulOncesi,virgulSonrasi);
			}

			int length = sprintf(text,"%s.txt=\"%s\"",location, datatxt);
			HAL_UART_Transmit(&huart2, (uint8_t *)text, length, 50);
			HAL_UART_Transmit(&huart2, (uint8_t *)nextionEnd, 3, 50);


			//memset(text, '\0', sizeof(text));
		}

		/*
	void txtNextion(char *location, float data){

		char text[50];
		char sdata[8];
		sprintf(sdata,"%2.2f",data);
		sprintf(text,"%s.txt=\"%s\"",location,sdata);
		HAL_UART_Transmit(&huart2, (uint8_t *)text, sizeof(text), 1000);
		HAL_UART_Transmit(&huart2, (uint8_t *)nextionEnd, 3, 100);
		//memset(text, '\0', sizeof(text));
	}
	*/
		void numNextion(char *location, int data){

			char text[50];

			int length = sprintf(text,"%s.val=%d",location,data);
			HAL_UART_Transmit(&huart2, (uint8_t *)text, length, 50);
			HAL_UART_Transmit(&huart2, (uint8_t *)nextionEnd, 3, 50);
			//memset(text, '\0', sizeof(text));
		}
		void picNextion(char *location, int data){
			char text[50];

			int length = sprintf(text,"%s.pic=%d",location,data);
			HAL_UART_Transmit(&huart2, (uint8_t *)text, length, 50);
			HAL_UART_Transmit(&huart2, (uint8_t *)nextionEnd, 3, 50);
			//memset(text, '\0', sizeof(text));

		}
		void ppicNextion(char *location, int data){
			char text[50];

			int length = sprintf(text,"%s.ppic=%d",location,data);
			HAL_UART_Transmit(&huart2, (uint8_t *)text, length, 50);
			HAL_UART_Transmit(&huart2, (uint8_t *)nextionEnd, 3, 50);
			//memset(text, '\0', sizeof(text));
		}
		void progbarNextion(char *location, int data){
			char text[50];

			int length = sprintf(text,"%s.val=%d",location,data);
			HAL_UART_Transmit(&huart2, (uint8_t *)text, length, 50);
			HAL_UART_Transmit(&huart2, (uint8_t *)nextionEnd, 3, 50);
			//memset(text, '\0', sizeof(text));
		}
		void inttxtNextion(char *location, int data){

			char text[50];
			//sprintf(sdata,"%f",data);
			int length = sprintf(text,"%s.txt=\"%d\"",location,data);
			HAL_UART_Transmit(&huart2, (uint8_t *)text, length, 50);
			HAL_UART_Transmit(&huart2, (uint8_t *)nextionEnd, 3, 50);
			//memset(text, '\0', sizeof(text));
		}



double calcIrms(unsigned int Number_of_Samples){
   int SupplyVoltage = 3.3;


  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
  	  sampleI = HAL_ADC_GetValue(&hadc1);
  	  HAL_ADC_Stop(&hadc1);
    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
    //  then subtract this - signal is now centered on 0 counts.
     offsetI = (offsetI + (sampleI-offsetI)/4095);
     filteredI = sampleI - offsetI;

    // Root-mean-square method current
    // 1) square current values
     sqI = filteredI * filteredI;
    // 2) sum
     sumI = sumI + sqI;
  }

   double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
   Irms = I_RATIO * sqrt(sumI / Number_of_Samples);

  //Reset accumulators
  sumI = 0;
  //--------------------------------------------------------------------------------------

  return Irms;
}
void create_new_file() {
    FRESULT res;
    int s = 0;
    FILINFO fno;
    int is_file_exist = 1;

    f_mount(&fs, "", 0);

    while (is_file_exist) {
        s++;
        if (snprintf(fileName, sizeof(fileName), "values%d.txt", s) >= sizeof(fileName)) {
            // Dosya adı çok uzun, hata işle
            return; // Hata durumunda fonksiyondan çık
        }

        // Dosyanın var olup olmadığını kontrol et
        res = f_stat(fileName, &fno);
        if (res == FR_OK) {
            is_file_exist = 1; // Dosya mevcut
        } else if (res == FR_NO_FILE) {
            is_file_exist = 0; // Dosya mevcut değil
        } else {
            // Diğer hata durumları
            // Hata işleme kodları buraya eklenebilir
            return; // Hata durumunda fonksiyondan çık
        }
    }

    if (!is_file_exist) {
        // Yeni dosya oluşturma işlemi burada yapılabilir
        res = f_open(&fil, fileName, FA_CREATE_NEW | FA_WRITE);
        if (res == FR_OK) {
            // Dosya başarılı bir şekilde oluşturuldu ve açıldı
            const char* header = "zaman_ms; hiz_km/h; sicaklik_C; toplamVoltaj_V; kalanEnerji_%\n";
            if (f_puts(header, &fil) < 0) {
                // Başlık yazma başarısız, hata işle
                f_close(&fil);
                return; // Hata durumunda fonksiyondan çık
            }
            // Dosyayı kapatma
            f_close(&fil);
        } else {
            // Dosya oluşturma/açma başarısız oldu
            // Hata işleme kodları buraya eklenebilir
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
