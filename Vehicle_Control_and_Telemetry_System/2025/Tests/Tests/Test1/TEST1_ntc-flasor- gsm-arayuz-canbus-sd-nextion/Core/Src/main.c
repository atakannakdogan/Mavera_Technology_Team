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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include <time.h>
//#include "fatfs_sd.h"
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

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t testTemp_analog;
float testTemp=0;

int sizedata;
int virgulOncesi;
int virgulSonrasi;
char datatxt[5];
//***************************-HALL Variables-***************************************
uint32_t last_time = 0;        // Önceki Hall sensörü tetiklenme zamanı
uint32_t current_time = 0;     // Şu anki Hall sensörü tetiklenme zamanı
float time_diff = 0;           // İki tetikleme arası zaman farkı (saniye)
float frequency = 0;           // Tekerleğin dönüş frekansı (Hz)
float rpm = 0;                 // RPM (dakikadaki devir sayısı)
float hall_speed = 0;               // Hız (km/saat)
//**********************************************************************************
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

uint32_t TxMailbox;

uint16_t motorTemp_analog;
float motorTemp;

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
static void MX_UART5_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
double Thermistor(uint16_t analogValue);

void send_at_command(char *command, uint16_t delay);
void send_data_to_server(float voltages[20], float temperatures[6], float current, float speed, float energy, float voltage_sum);
void send_data_cont(float voltages[20], float temperatures[6], float current, float speed, float energy, float voltage_sum);

void getCAN1(CAN_RxHeaderTypeDef *RxHeader1, float *dataReceive);
void sendFixedMessage( uint8_t ADDH,uint8_t ADDL, uint8_t CHAN, const void *message, const uint8_t size);

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

uint8_t rxBuffer[128];
uint8_t rxIndex = 0;
uint8_t rxData = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader1;
    float dataReceive[2];
    HAL_GPIO_WritePin(prg_led_GPIO_Port, prg_led_Pin, 1);


    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1, dataReceive) == HAL_OK)
    {
        getCAN1(&RxHeader1, dataReceive);
    }

}
/*
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
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == Hall_Pin) {  // Hall sensörü GPIO_PIN_0'da bağlı

        // Zaman damgasını al (milisaniye cinsinden)
        current_time = HAL_GetTick();  // Timer'dan geçen süreyi al (ms cinsinden)

        // Eğer daha önce bir tetikleme olduysa, zaman farkını hesapla
        if (last_time != 0) {
            // Zaman farkını milisaniyeden saniyeye çevir
            time_diff = (current_time - last_time) / 1000.0;  // ms -> saniye

            // Frekansı hesapla (frekans = 1 / zaman farkı)
            frequency = 1.0 / time_diff;

            // RPM'yi hesapla (frekans * 60)
            rpm = frequency * 60.0;

            // Hızı hesapla (hız = çevre * frekans) ve km/saat'e çevir
            hall_speed = (1.35 * frequency) * 3.6;

            // Hız ve RPM değerlerini terminale yazdır (isteğe bağlı)
            //printf("Hız: %.2f km/h, RPM: %.2f\r\n", speed, rpm);
        }

        // Şu anki zamanı "last_time" olarak güncelle
        last_time = current_time;
    }
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
  MX_UART5_Init();
  MX_CAN1_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  data1.testTemp = 0;
  data1.totalVoltage = 0;
    data1.batteryTemp1= 0;
    data1.batteryTemp2 = 0;
    data1.batteryTemp3 = 0;
    data1.batteryTemp4 = 0;

    create_new_file();
      HAL_CAN_Start(&hcan1);
      HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
     // lwgps_init(&gps);
      HAL_UART_Receive_IT(&huart5, &rxData, 1);

  float voltages[20] = {
        	             99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99,
        	             99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99};

        	  float temperatures[6] = {99.99, 99.99, 99.99, 99.99, 99.99, 99.99};
        	  float current = 99.99;
        	  float speed = 99.99;
        	  float energy = 99.99;
        	  float voltage_sum = 99.99;

    srand(HAL_GetTick());
    send_data_to_server(voltages, temperatures, current, speed, energy, voltage_sum);
    HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_ADC_Start(&hadc1);
	 	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	 	  	  testTemp_analog = HAL_ADC_GetValue(&hadc1);
	 	  	  data1.testTemp = Thermistor(testTemp_analog);
	 	  	  HAL_ADC_Stop(&hadc1);

	 	  	  HAL_GPIO_WritePin(prg_led_GPIO_Port, prg_led_Pin, 1);
	 	  	  HAL_Delay(100);

	 	  	if(data1.testTemp<0){
	 	  						data1.testTemp = 0;
	 	  					}
	 	  					else{
	 	  						data1.testTemp = data1.testTemp;
	 	  					}

	 	  	if(data1.testTemp>65){

	 	  			  HAL_GPIO_WritePin(flasor_GPIO_Port, flasor_Pin, 1);

	 	  		  }
	 	  		  else{

	 	  			 HAL_GPIO_WritePin(flasor_GPIO_Port, flasor_Pin, 0);
	 	  		  }

	 	  	data1.speed=hall_speed;
	 	  	float voltages[20] = {
	 	  			((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)),
	 	  						 ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40)), ((float)(rand()%40))};

	 	  	    	  float temperatures[6] = {22.3, 23.0, 21.8, 22.5, data1.testTemp, 22.9};
	 	  	    	  float current = (5+(float)(rand()%30));
	 	  	    	  float speed = (data1.speed);
	 	  	    	  float energy = (60+(float)(rand()%102));
	 	  	    	  float voltage_sum = 250.0;

	 	  	    	  send_data_cont(voltages, temperatures, current, speed, energy, voltage_sum);
	 	  	    	  //HAL_Delay(10);

	 	  	    	uint32_t a = HAL_GetTick();;
	 	  	    	HAL_Delay(50);

	 	 f_mount(&fs, "", 0);
	 	  f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	 	  f_lseek(&fil,f_size(&fil));
	 	  //"zaman_ms; hiz_km/h; sicaklik_C; toplamVoltaj_V; kalanEnerji_%\n"
	 	  snprintf(buffer, sizeof(buffer), "%-8lu; %-8d; %-10.2f; %-14.2f; %-13d\n",a,(int)data1.speed,data1.testTemp, data1.totalVoltage,(int)data1.remainEnergy);

	 	  fresult = f_puts(buffer,&fil);
	 	  f_close(&fil);


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
  sConfig.Channel = ADC_CHANNEL_0;
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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(prg_led_GPIO_Port, prg_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(flasor_GPIO_Port, flasor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : prg_led_Pin */
  GPIO_InitStruct.Pin = prg_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(prg_led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : flasor_Pin */
  GPIO_InitStruct.Pin = flasor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(flasor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Hall_Pin */
  GPIO_InitStruct.Pin = Hall_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Hall_GPIO_Port, &GPIO_InitStruct);

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
	//		iz_res = dataReceive[0];
		//	ek_res = dataReceive[1];
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

void send_at_command(char *command, uint16_t delay) {
    HAL_UART_Transmit(&huart5, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);
    HAL_Delay(1000); // SIM808'den cevap beklemek için gecikme
}

void send_data_cont(float voltages[20], float temperatures[6], float current, float speed, float energy, float voltage_sum){
	char http_data[64];
	char json_payload[1024];
	sprintf(json_payload,"{\"v1\": %.2f, \"v2\": %.2f, \"v3\": %.2f, \"v4\": %.2f, "
            "\"v5\": %.2f, \"v6\": %.2f, \"v7\": %.2f, \"v8\": %.2f, "
            "\"v9\": %.2f, \"v10\": %.2f, \"v11\": %.2f, \"v12\": %.2f, "
            "\"v13\": %.2f, \"v14\": %.2f, \"v15\": %.2f, \"v16\": %.2f, "
            "\"v17\": %.2f, \"v18\": %.2f, \"v19\": %.2f, \"v20\": %.2f, "
            "\"vt\": %.2f, \"s1\": %.2f, \"s2\": %.2f, "
            "\"s3\": %.2f, \"s4\": %.2f, \"s5\": %.2f, \"s6\": %.2f, "
            "\"a\": %.2f, \"h\": %.2f, \"kE\": %.2f}",
            voltages[0], voltages[1], voltages[2], voltages[3], voltages[4],
            voltages[5], voltages[6], voltages[7], voltages[8], voltages[9],
            voltages[10], voltages[11], voltages[12], voltages[13], voltages[14],
            voltages[15], voltages[16], voltages[17], voltages[18], voltages[19],
            voltage_sum, temperatures[0], temperatures[1], temperatures[2],
            temperatures[3], temperatures[4], temperatures[5], current, speed, energy);
	sprintf(http_data, "AT+HTTPDATA=%d,5000\r\n", strlen(json_payload));
    send_at_command(http_data, 100);
    send_at_command(json_payload, 100);

    // POST isteği gönder
    send_at_command("AT+HTTPACTION=1\r\n", 100); // 1: POST
    HAL_Delay(1000); // Veriyi göndermek için biraz zaman tanı

}

void send_data_to_server(float voltages[20], float temperatures[6], float current, float speed, float energy, float voltage_sum) {
    char apn_setup[64];
    char http_init[32];
    char http_start[128];
    char http_data[64];
    char json_payload[2048];

    // JSON payload
    sprintf(json_payload,"{\"v1\": %.2f, \"v2\": %.2f, \"v3\": %.2f, \"v4\": %.2f, "
            "\"v5\": %.2f, \"v6\": %.2f, \"v7\": %.2f, \"v8\": %.2f, "
            "\"v9\": %.2f, \"v10\": %.2f, \"v11\": %.2f, \"v12\": %.2f, "
            "\"v13\": %.2f, \"v14\": %.2f, \"v15\": %.2f, \"v16\": %.2f, "
            "\"v17\": %.2f, \"v18\": %.2f, \"v19\": %.2f, \"v20\": %.2f, "
            "\"vt\": %.2f, \"s1\": %.2f, \"s2\": %.2f, "
            "\"s3\": %.2f, \"s4\": %.2f, \"s5\": %.2f, \"s6\": %.2f, "
            "\"a\": %.2f, \"h\": %.2f, \"kE\": %.2f}",
            voltages[0], voltages[1], voltages[2], voltages[3], voltages[4],
            voltages[5], voltages[6], voltages[7], voltages[8], voltages[9],
            voltages[10], voltages[11], voltages[12], voltages[13], voltages[14],
            voltages[15], voltages[16], voltages[17], voltages[18], voltages[19],
            voltage_sum, temperatures[0], temperatures[1], temperatures[2],
            temperatures[3], temperatures[4], temperatures[5], current, speed, energy);

    // SIM808 başlatma
    send_at_command("AT\r\n", 1000); // AT komutuyla modülü kontrol et
    send_at_command("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n", 1000); // GPRS yapılandırması
    sprintf(apn_setup, "AT+SAPBR=3,1,\"APN\",\"internet\"\r\n"); // APN'i "internet" olarak ayarla
    send_at_command(apn_setup, 1000);
    send_at_command("AT+SAPBR=1,1\r\n", 1000); // GPRS oturumunu başlat

    // HTTP başlatma
    send_at_command("AT+HTTPINIT\r\n", 1000); // HTTP başlat
    sprintf(http_init, "AT+HTTPPARA=\"CID\",1\r\n");
    send_at_command(http_init, 1000);
    send_at_command("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n", 1000);//FORMATI BELİRTİYOR
    sprintf(http_start, "AT+HTTPPARA=\"URL\",\"http://maveratelemetry.somee.com/api/SensorData\"\r\n"); //post adresi
    send_at_command(http_start, 1000);

    // JSON içeriği gönderme
    sprintf(http_data, "AT+HTTPDATA=%d,10000\r\n", strlen(json_payload));
    send_at_command(http_data, 1000);
    send_at_command(json_payload, 1000);

    // POST isteği gönder
    send_at_command("AT+HTTPACTION=1\r\n", 1000); // 1: POST
    HAL_Delay(5000); // Veriyi göndermek için biraz zaman tanı

    // HTTP sonlandırma
    //send_at_command("AT+HTTPREAD\r\n", 1000);
    //send_at_command("AT+HTTPTERM\r\n", 1000); // HTTP oturumunu kapat



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
	//	 numNextion("n2", (int)gps.sats_in_use);
		 if(fresult>0 && fresult<254){
			 picNextion("p3",70); // SD kart
		 }
		 else{
			 picNextion("p3",69); // SD kart
		 }

	//	 txtNextion("t31",iz_res);
	//	 txtNextion("t32",ek_res);
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
				HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 50);
				HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 50);


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
				HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 50);
				HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 50);
				//memset(text, '\0', sizeof(text));
			}
			void picNextion(char *location, int data){
				char text[50];

				int length = sprintf(text,"%s.pic=%d",location,data);
				HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 50);
				HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 50);
				//memset(text, '\0', sizeof(text));

			}
			void ppicNextion(char *location, int data){
				char text[50];

				int length = sprintf(text,"%s.ppic=%d",location,data);
				HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 50);
				HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 50);
				//memset(text, '\0', sizeof(text));
			}
			void progbarNextion(char *location, int data){
				char text[50];

				int length = sprintf(text,"%s.val=%d",location,data);
				HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 50);
				HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 50);
				//memset(text, '\0', sizeof(text));
			}
			void inttxtNextion(char *location, int data){

				char text[50];
				//sprintf(sdata,"%f",data);
				int length = sprintf(text,"%s.txt=\"%d\"",location,data);
				HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 50);
				HAL_UART_Transmit(&huart1, (uint8_t *)nextionEnd, 3, 50);
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
