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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define ADC_COUNTS  (1<<12)

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_EFFICIENCY        0.932f    // Ansys simülasyonu motor verimi (%93.2)
#define RADPS_TO_RPM_CONST      9.5488f   // 60 / (2 * PI) birim dönüşüm katsayısı
#define MIN_RPM_THRESHOLD       5.0f      // Sıfıra bölme hatasını engelleyen devir eşiği (RPM)
#define MOTOR_KT_NM_A           1.41f     // 0-5 RPM arası durma torku sabiti (Nm/A)

#define TIMER_FREQ          100000.0f
#define WHEEL_CIRCUMFERENCE 1.2768f  // 16 inç (0.4064m çap) tekerlek çevresi
#define MAGNET_COUNT 4// Mıknatıs sayısı 8'e çıkarıldı
#define DISTANCE_PER_PULSE (WHEEL_CIRCUMFERENCE / MAGNET_COUNT)
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

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t nextion_rx_data[1];          // Nextion'dan gelecek 1 byte'lık veri
volatile uint8_t animation_flag = 0;

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
float sicakliklar[5];
uint8_t nextionEnd[3] = {0xFF,0xFF,0xFF};
float maxTemp;
uint8_t modbusTxData[8];  // Sorgu göndermek için
uint8_t modbusRxData[9]; // Gelen cevabı almak için

int isCharging = 0;

float iz_res = 0.0f; // Kodunda zaten var, bunu güncelleyeceğiz.
float ek_res = 0;
static float aracin_gercek_direnci = 1200.0f;

// İnotel IMD100 Varsayılan Modbus ID'si genelde 1'dir.
#define IMD_SLAVE_ID 4
#define IMD_REG_IZOLASYON 0x04
//***************************-HALL Variables (YENİ)-**************************
extern TIM_HandleTypeDef htim4; // Artık htim2 değil htim4

volatile uint32_t last_capture_val = 0;
volatile uint32_t current_capture_val = 0;
volatile uint8_t is_first_capture = 1;
volatile uint32_t last_pulse_tick = 0;

volatile uint32_t overflow_count = 0;
volatile uint32_t last_overflow_count = 0;

volatile float hall_speed = 0.0f;
volatile float filtered_rpm = 0.0f;
volatile float rpm = 0.0f;


volatile uint32_t tick_buffer[4] = {0}; // Maksimum 8 mıknatısa kadar tampon
volatile uint8_t tick_index = 0;
volatile uint8_t valid_pulses = 0;

int count =0;

//****************************************************************************

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

int cntt=0;


CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
volatile uint8_t RxData[8];
uint32_t TxMailbox;

float batarya_voltaji = 0;
float batarya_akimi = 0;
float batarya_yuzdesi = 0;
uint16_t max_hucre_mV = 0;
int8_t max_sicaklik_C = 0;

uint16_t ham_akim;

float anlik_guc_W = 0.0f;
float toplam_harcanan_Wh = 0.0f;
uint32_t son_zaman_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Modbus_CRC16(uint8_t *buf, int len);
float Modbus_Request_Izolasyon(void);

void BMS_Request_Data(uint32_t data_id);
void BMS_Charge_Control(uint8_t start_charge);

void getCAN1(CAN_RxHeaderTypeDef *RxHeader1, uint8_t *dataReceive);

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

void txtNextionFormatted(char *location, float data);

float kalibre_et(float imd_degeri);
// GSM fonksiyon prototipleri
double Thermistor(uint16_t analogValue);
void gsm_init();
void gsm_send_data(float voltages[20], float temperatures[6], float current, float speed, float energy, float voltage_sum);
void send_at_command(char *command, uint16_t delay);

void CalculateSpeed(TIM_HandleTypeDef *htim,uint32_t channel);

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
	  float mech_power;
	  float torque;
}data1;

//CAN_RxHeaderTypeDef RxHeader1;



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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_UART5_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  data1.totalVoltage = 0;
  data1.batteryTemp1= 0;
  data1.batteryTemp2 = 0;
  data1.batteryTemp3 = 0;
  data1.batteryTemp4 = 0;

  sendNextion();

  create_new_file();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);



  // lwgps_init(&gps); ve HAL_UART_Receive_IT(&huart6, &rxData, 1); sadece GPS içinse kaldırıldı

  // GSM başlatıcı
  gsm_init();

  float voltages[20], temperatures[6], current, speed, energy, voltage_sum;
  for(int i=0;i<20;i++) voltages[i] = 99.99;
  for(int i=0;i<6;i++) temperatures[i] = 99.99;
  current = 99.99; speed = 99.99; energy = 99.99; voltage_sum = 99.99;
  gsm_send_data(voltages, temperatures, current, speed, energy, voltage_sum);
  HAL_Delay(1000);

  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim4);


  HAL_UART_Receive_IT(&huart2, nextion_rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_gsm_send = HAL_GetTick();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
	      __HAL_UART_CLEAR_OREFLAG(&huart2);
	      HAL_UART_Receive_IT(&huart2, nextion_rx_data, 1); // Dinlemeyi yeniden başlat
	  }




	  if (HAL_GetTick() - last_pulse_tick > 1000) {
	            hall_speed = 0.0f;
	            filtered_rpm = 0.0f;

	            rpm = 0.0f;
	            data1.speed = 0.0f;
	            is_first_capture = 1; // Araba tekrar kalktığında ilk okumayı pas geç


	            valid_pulses = 0;
	        }


	  HAL_ADC_Start(&hadc3);
	  // Çevrimin bitmesi için max 10ms bekle
	  HAL_ADC_PollForConversion(&hadc3, 10);
	  testTemp_analog = HAL_ADC_GetValue(&hadc3);
	  data1.testTemp = Thermistor(testTemp_analog);
	  HAL_ADC_Stop(&hadc3);

	  HAL_ADC_Start(&hadc2);
	  // Aynı bekleme motorTemp için de gerekli
	  HAL_ADC_PollForConversion(&hadc2, 10);
	  motorTemp_analog = HAL_ADC_GetValue(&hadc2);
	  data1.motorTemp = Thermistor(motorTemp_analog);
	  HAL_ADC_Stop(&hadc2);

	  //---------------------- SLAVE 1 && S------------------------
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  HAL_Delay(10);

	  BMS_Request_Data(0x90);
	  HAL_Delay(10);
	  BMS_Request_Data(0x95);
	  HAL_Delay(10);
	  BMS_Request_Data(0x96);

	  iz_res = Modbus_Request_Izolasyon();

	  // 2. Direnç takıldı mı takılmadı mı kontrolü yap
	  float iz_res_kohm = iz_res / 1000.0f;

	  	  // 2. Direnç takıldı mı takılmadı mı kontrolü yap (Sınırımız 900 kOhm)
	  	  if (iz_res_kohm > 1000.0f) {
	  	      // 900k'nın üstündeyse soket BOŞTUR, araç normal çalışıyordur.
	  	      aracin_gercek_direnci = iz_res_kohm; // Arabanın o anki yalıtımını kOhm olarak kaydet (örn: 1200.0)
	  	      ek_res = 0.0f;                       // Takılı direnç yok
	  	  } else {
	  	      // 900k'nın altına düştüyse HAKEM DİRENÇ TAKMIŞTIR.
	  	      // Gelen düşmüş değeri kOhm cinsinden kalibre edip "ek_res" yapıyoruz:
	  	      ek_res = kalibre_et(iz_res_kohm);
	  	  }

	  data1.totalVoltage = data1.voltage1 + data1.voltage2 + data1.voltage3 +data1.voltage4 + data1.voltage5 + data1.voltage6 + data1.voltage7 + data1.voltage8 + data1.voltage9 + data1.voltage10 + data1.voltage11 + data1.voltage12 + data1.voltage13 + data1.voltage14 + data1.voltage15 + data1.voltage16 + data1.voltage17 + data1.voltage18 + data1.voltage19 + data1.voltage20;
	  if(data1.totalVoltage <0){
		  data1.totalVoltage = 0;
	  }
	  else{
		  data1.totalVoltage = data1.totalVoltage;
	  }

	  //data1.current =calcIrms(1480)*10;;
	  data1.remainEnergy  = (data1.totalVoltage - 64.00)*100/20;

	  if(data1.remainEnergy <0 ){
		  data1.remainEnergy = 0;
	  }


	  uint32_t su_an_ms = HAL_GetTick();
	  // Döngü başından beri geçen süreyi saniye olarak hesapla
	  float gecen_sure_sn = (su_an_ms - son_zaman_ms) / 1000.0f;
	  son_zaman_ms = su_an_ms;

	  // Gelen akım ve voltaj geçerliyse hesapla (Araç şarj oluyorsa akım eksi olabilir, harcamayı ölçmek için pozitifi al)
	  if(data1.totalVoltage > 0 && data1.current > 0) {


	      anlik_guc_W = data1.totalVoltage * data1.current; // P = V * I

	      // Tüketilen enerjiyi Watt-Saat cinsinden kümülatif olarak ekle
	      // (Güç * Süre) / 3600
	      toplam_harcanan_Wh += (anlik_guc_W * gecen_sure_sn) / 3600.0f;

	      // ----- YENİ EKLENEN TORK VE GÜÇ HESABI -----
	      data1.mech_power = anlik_guc_W * MOTOR_EFFICIENCY; // Mekanik Güç

	      	      if (filtered_rpm >= MIN_RPM_THRESHOLD) {
	                    // Gerçek motor torku
	      	          data1.torque = (data1.mech_power / filtered_rpm) * RADPS_TO_RPM_CONST;

	                    // EĞER TEKERLEK TORKUNU EKRANDA GÖRMEK İSTİYORSAN (Opsiyonel)
	                    // data1.torque = data1.torque * 6.0f;
	      	      } else {
	      	          data1.torque = data1.current * MOTOR_KT_NM_A;
	      	      }
	                // --------------------------------------------
	  }
	  //gps_satallite = gps.sats_in_use; ve gps ile ilgili diğer satırlar kaldırıldı

	  sendNextion();
		// Sıcaklık değerlerini sicakliklar dizisine aktar


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

		// --- SD KART YAZMA BLOĞU ---
		    uint32_t a = HAL_GetTick();

		    fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
		    if (fresult == FR_OK){
		    	f_lseek(&fil, f_size(&fil));

		    			    		        // Verileri başlık sırasına göre CSV formatında yaz
		    			    		        snprintf(buffer, sizeof(buffer), "%lu;%.1f;%.1f;%.2f;%.2f;%.2f;%.1f;%.1f\n",
		    			    		                 a,
		    			    		                 data1.speed,
		    			    		                 filtered_rpm,
		    			    		                 data1.torque,
		    			    		                 data1.current,
		    			    		                 data1.totalVoltage,
		    			    		                 data1.mech_power,
		    			    		                 data1.motorTemp);

		    			    		        f_puts(buffer, &fil);
		    			    		        f_close(&fil);
		    			    // ----------------------------
		    }


	  if(data1.motorTemp>55 || ((int)iz_res >0 && (int)iz_res <= 120000)){

		  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);


	  }
	  else{

		  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);

	  }

	  if(isCharging == 1 || data1.motorTemp>70 || ((int)iz_res >0 && (int)iz_res <= 120000)){

		  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	  }
	  else{

		  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
	  }


	  if (data1.motorTemp>55){
		  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	  }
	  else{
		  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 0);
	  }

	  sizedata = sizeof(data1);



	  // GSM için veri güncelleme ve gönderim (her 1 saniyede bir)
	  if (HAL_GetTick() - last_gsm_send > 1000) {
		  voltages[0]  = data1.voltage1;
		  voltages[1]  = data1.voltage2;
		  voltages[2]  = data1.voltage3;
		  voltages[3]  = data1.voltage4;
		  voltages[4]  = data1.voltage5;
		  voltages[5]  = data1.voltage6;
		  voltages[6]  = data1.voltage7;
		  voltages[7]  = data1.voltage8;
		  voltages[8]  = data1.voltage9;
		  voltages[9]  = data1.voltage10;
		  voltages[10] = data1.voltage11;
		  voltages[11] = data1.voltage12;
		  voltages[12] = data1.voltage13;
		  voltages[13] = data1.voltage14;
		  voltages[14] = data1.voltage15;
		  voltages[15] = data1.voltage16;
		  voltages[16] = data1.voltage17;
		  voltages[17] = data1.voltage18;
		  voltages[18] = data1.voltage19;
		  voltages[19] = data1.voltage20;

	    temperatures[0] = data1.batteryTemp1;
	    temperatures[1] = data1.batteryTemp2;
	    temperatures[2] = data1.batteryTemp3;
	    temperatures[3] = data1.batteryTemp4;
	    temperatures[4] = data1.testTemp;
	    temperatures[5] = data1.motorTemp;
	    current = data1.current;
	    speed = data1.speed;
	    energy = data1.remainEnergy;
	    voltage_sum = data1.totalVoltage;
	    gsm_send_data(voltages, temperatures, current, speed, energy, voltage_sum);
	    last_gsm_send = HAL_GetTick();

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
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  CAN_FilterTypeDef sFilterConfig;

  	sFilterConfig.FilterBank = 0;
  	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  	sFilterConfig.FilterIdHigh = 0x0000;
  	sFilterConfig.FilterIdLow = 0x0000;
  	sFilterConfig.FilterMaskIdHigh = 0x0000;
  	sFilterConfig.FilterMaskIdLow = 0x0000;
  	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  	sFilterConfig.FilterActivation = ENABLE;
  	sFilterConfig.SlaveStartFilterBank = 14;

  	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
  		Error_Handler();
  	}
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 840-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, S4_Pin|S3_Pin|S2_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : S4_Pin S3_Pin S2_Pin S1_Pin */
  GPIO_InitStruct.Pin = S4_Pin|S3_Pin|S2_Pin|S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Veri Nextion'un bağlı olduğu USART2'den mi geldi?
    if (huart->Instance == USART2)
    {
        // Gelen veri 02 mi? (ŞARJI BAŞLAT)
        if (nextion_rx_data[0] == 0x02)
        {
            isCharging = 1;
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
            BMS_Charge_Control(1); // Şarj MOSFET'ini AÇ
        }
        // Gelen veri 03 mü? (ŞARJI DURDUR)
        else if (nextion_rx_data[0] == 0x03)
        {
            isCharging = 0;
            BMS_Charge_Control(0); // Şarj MOSFET'ini KAPAT
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
        }

        // Tekrar dinlemeye devam et
        HAL_UART_Receive_IT(&huart2, nextion_rx_data, 1);
    }
}




void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    // PB9 pini TIM4_CH4'e bağlıdır
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        if (htim->Instance == TIM4) {
        	if(count%2==0){
        		CalculateSpeed(htim,TIM_CHANNEL_4);
        	}



        }

    }
}
// --- MODBUS CRC16 HESAPLAMA ---
uint16_t Modbus_CRC16(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// --- MODBUS SORGUSU GÖNDERME ---
float Modbus_Request_Izolasyon(void) {
    // 1. Paketi Hazırla (Read Input Registers - Function Code 0x04)
    modbusTxData[0] = IMD_SLAVE_ID; // Slave ID
    modbusTxData[1] = 0x04;         // Fonksiyon Kodu (0x04 Olmalı)
    modbusTxData[2] = 0x00;         // Register Adresi High
    modbusTxData[3] = 0x00;         // Register Adresi Low (Adres 0)
    modbusTxData[4] = 0x00;         // Okunacak Register Sayısı High
    modbusTxData[5] = 0x02;         // Okunacak Register Sayısı Low (Float veri 4 byte yani 2 register'dır)

    // 2. CRC Hesapla ve Pakete Ekle
    uint16_t crc = Modbus_CRC16(modbusTxData, 6);
    modbusTxData[6] = crc & 0xFF;        // CRC Low Byte
    modbusTxData[7] = crc >> 8; // CRC High Byte

    __HAL_UART_FLUSH_DRREGISTER(&huart6);

    // 3. RS485 Yönünü TX (Gönderme) Yap (PC5 = HIGH)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    // 4. Veriyi USART6 üzerinden gönder
    HAL_UART_Transmit(&huart6, modbusTxData, 8, 100);

    HAL_Delay(2);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    if(HAL_UART_Receive(&huart6, modbusRxData, 9, 700) == HAL_OK){
    	uint16_t received_crc = ((modbusRxData[8] << 8)| modbusRxData[7]);
    	if(Modbus_CRC16(modbusRxData,7) == received_crc){
    		union {
    			float value;
    			uint8_t bytes[4];
    		}float_converter;

    		float_converter.bytes[3] = modbusRxData[3];
			float_converter.bytes[2] = modbusRxData[4];
			float_converter.bytes[1] = modbusRxData[5];
			float_converter.bytes[0] = modbusRxData[6];

			return float_converter.value;
    	}

    }
    return -1.0f;
}
void txtNextionFormatted(char *location, float data) {
    char text[50];
    char formatted_data[20];
    char temp_str[20];

    // 1. Tam sayı ve ondalık kısmı ayır
    int tam_kisim = (int)data;
    int ondalik_kisim = (int)((data - tam_kisim) * 10.0f); // Sadece 1 ondalık hane alıyoruz

    // Negatif kontrolü
    if (ondalik_kisim < 0) {
        ondalik_kisim = -ondalik_kisim;
    }

    // 2. Tam kısmı normal stringe çevir
    sprintf(temp_str, "%d", tam_kisim);
    int len = strlen(temp_str);

    // 3. Binlik ayraçları (virgül) ekleyerek formatla
    int i = 0, j = 0;
    if (temp_str[0] == '-') {
        formatted_data[j++] = '-';
        i++;
    }

    for (; i < len; i++) {
        formatted_data[j++] = temp_str[i];
        if ((len - i - 1) > 0 && (len - i - 1) % 3 == 0) {
            formatted_data[j++] = ',';
        }
    }

    // 4. Ondalık kısmı sonuna ekle
    sprintf(&formatted_data[j], ".%d", ondalik_kisim);

    // 5. Nextion'a gönder
    int length = sprintf(text, "%s.txt=\"%s\"", location, formatted_data);
    HAL_UART_Transmit(&huart2, (uint8_t *)text, length, 50);
    HAL_UART_Transmit(&huart2, (uint8_t *)nextionEnd, 3, 50);
}

void BMS_Request_Data(uint32_t data_id) {
    // 1. ÇÖZÜM: Global yerine sadece bu fonksiyona özel değişkenler oluşturuyoruz
    CAN_TxHeaderTypeDef TxHeaderLocal;
    uint8_t TxDataLocal[8] = {0};
    uint32_t TxMailboxLocal;

    // 2. ÇÖZÜM: CAN hattı hataya düşmüşse (kilitlenmişse) sistemi otomatik canlandır
    if (hcan1.State == HAL_CAN_STATE_ERROR) {
        HAL_CAN_ResetError(&hcan1);
        HAL_CAN_Stop(&hcan1);
        HAL_CAN_Start(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

    TxHeaderLocal.StdId = 0x00;
    TxHeaderLocal.ExtId = 0x18000140 | (data_id << 16);
    TxHeaderLocal.RTR = CAN_RTR_DATA;
    TxHeaderLocal.IDE = CAN_ID_EXT;
    TxHeaderLocal.DLC = 8;

    // CAN1 üzerinden mesajı hatta bırak
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &TxHeaderLocal, TxDataLocal, &TxMailboxLocal);
    }
}

/*
 * Daly BMS Şarj MOSFET Kontrolü
 * Parametre: 1 (Şarjı Başlat), 0 (Şarjı Durdur)
 */
void BMS_Charge_Control(uint8_t start_charge) {
    // Çakışmayı önlemek için yine sadece buraya özel değişkenler kullanıyoruz
    CAN_TxHeaderTypeDef TxHeaderLocal;
    uint8_t TxDataLocal[8] = {0};
    uint32_t TxMailboxLocal;

    TxHeaderLocal.StdId = 0x00;
    TxHeaderLocal.ExtId = 0x18DA0140;      // 0xDA: Şarj MOS Kontrol CAN ID'si
    TxHeaderLocal.RTR = CAN_RTR_DATA;
    TxHeaderLocal.IDE = CAN_ID_EXT;
    TxHeaderLocal.DLC = 8;

    TxDataLocal[0] = 0xDA; // MOSFET kontrol komut kodu

    if (start_charge == 1) {
        TxDataLocal[1] = 0x00; // Şarjı Başlat (MOSFET AÇIK)
    } else {
        TxDataLocal[1] = 0x01; // Şarjı Durdur (MOSFET KAPALI)
    }

    TxDataLocal[2] = 0x00; // Deşarjı her zaman açık bırak

    // CAN hattına mesajı bırak
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &TxHeaderLocal, TxDataLocal, &TxMailboxLocal);
    }
}

/*
 * CAN RX Kesmesi (Interrupt) Callback Fonksiyonu
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {

		// ÇÖZÜM BURADA: FIFO'da mesaj olduğu sürece (kutu boşalana kadar) dön ve hepsini al!
		while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
        {
			if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, (uint8_t*) RxData) == HAL_OK)
            {
				// Mesajın Extended Frame olup olmadığını kontrol et
				if (RxHeader.IDE == CAN_ID_EXT)
                {
					uint32_t received_data_id = (RxHeader.ExtId >> 16) & 0xFF;

					if (received_data_id >= 0x90 && received_data_id <= 0x98)
                    {
						// 0x90 Paketi: Voltaj, Akım, SOC
						if (received_data_id == 0x90) {
							uint16_t ham_voltaj = (RxData[0] << 8) | RxData[1];
							batarya_voltaji = ham_voltaj / 10.0f;

							uint16_t ham_akim = (RxData[4] << 8) | RxData[5];
							batarya_akimi = (30000 - ham_akim) / 10.0f; // Pozitif tüketim için ters çevirmiştik

							uint16_t ham_soc = (RxData[6] << 8) | RxData[7];
							batarya_yuzdesi = ham_soc / 10.0f;
						}
						// 0x91 Paketi: Hücre Voltajları Ekstrem
						else if (received_data_id == 0x91) {
							max_hucre_mV = (RxData[1] << 8) | RxData[2];
						}
						// 0x92 Paketi: Sıcaklık
						else if (received_data_id == 0x92) {
							max_sicaklik_C = RxData[1] - 40;
						}
						// 0x95 Paketi: BİREYSEL HÜCRE VOLTAJLARI
						else if (received_data_id == 0x95) {

							uint8_t frame_no = RxData[0];

							float cell_a = ((RxData[1] << 8) | RxData[2]) / 1000.0f;
							float cell_b = ((RxData[3] << 8) | RxData[4]) / 1000.0f;
							float cell_c = ((RxData[5] << 8) | RxData[6]) / 1000.0f;

							if (frame_no == 1) {
								data1.voltage1 = cell_a;
								data1.voltage2 = cell_b;
								data1.voltage3 = cell_c;
							}
							else if (frame_no == 2) {
								data1.voltage4 = cell_a;
								data1.voltage5 = cell_b;
								data1.voltage6 = cell_c;
							}
							else if (frame_no == 3) {
								data1.voltage7 = cell_a;
								data1.voltage8 = cell_b;
								data1.voltage9 = cell_c;
							}
							else if (frame_no == 4) {
								data1.voltage10 = cell_a;
								data1.voltage11 = cell_b;
								data1.voltage12 = cell_c;
							}
							else if (frame_no == 5) {
								data1.voltage13 = cell_a;
								data1.voltage14 = cell_b;
								data1.voltage15 = cell_c;
							}
							else if (frame_no == 6) {
								data1.voltage16 = cell_a;
								data1.voltage17 = cell_b;
								data1.voltage18 = cell_c;
							}
							else if (frame_no == 7) {
								data1.voltage19 = cell_a;
								data1.voltage20 = cell_b;
							}
						}
						else if (received_data_id == 0x96) {

							uint8_t frame_no = RxData[0]; // Paket numarası

													// İlk pakette 1'den 7'ye kadar olan NTC sensörlerinin verisi gelir
							if (frame_no == 1) {
							// Gerçek sıcaklığı bulmak için gelen veriden 40 çıkarıyoruz
							data1.batteryTemp1 = RxData[1] - 40.0f;
							data1.batteryTemp2 = RxData[2] - 40.0f;
							data1.batteryTemp3 = RxData[3] - 40.0f;
							data1.batteryTemp4 = RxData[4] - 40.0f;
						                                // Not: Eğer sisteminde 5., 6. ve 7. sensörler de varsa
						                                // RxData[5], RxData[6], RxData[7] şeklinde okumaya devam edebilirsin.
													}
												}
					}
				}
			}
		} // While Döngüsü Sonu

		// Döngü bittiğinde global verileri ana sisteme aktar
		data1.totalVoltage = batarya_voltaji;
		data1.current = batarya_akimi;
	}
}



/*void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader,
				(uint8_t*) RxData) == HAL_OK) {
			// Mesajın Extended Frame olup olmadığını kontrol et
			if (RxHeader.IDE == CAN_ID_EXT) {
				uint32_t received_data_id = (RxHeader.ExtId >> 16) & 0xFF;
				// Değişkenleri global olarak veya fonksiyon üstünde tanımlayabilirsin

				// ... [CAN RX KESME İÇERİSİ] ...

				if (received_data_id >= 0x90 && received_data_id <= 0x98) {
					// 0x90 Paketi: Voltaj, Akım, SOC
					if (received_data_id == 0x90) {
						// 1. Toplam Voltaj (Byte 0 ve 1) -> 0.1V çarpanlı
						uint16_t ham_voltaj = (RxData[0] << 8) | RxData[1];
						batarya_voltaji = ham_voltaj / 10.0f;

						// 2. Akım (Byte 4 ve 5) -> 30000 ofsetli, 0.1A çarpanlı
						 ham_akim = (RxData[4] << 8) | RxData[5];
						batarya_akimi = (30000.0 - ham_akim) / 10.0f;

						// 3. SOC (Byte 6 ve 7) -> 0.1% çarpanlı
						uint16_t ham_soc = (RxData[6] << 8) | RxData[7];

						batarya_yuzdesi = ham_soc / 10.0f;
					}

					// 0x91 Paketi: Hücre Voltajları
					else if (received_data_id == 0x91) {
						// En yüksek hücre voltajı (Byte 1 ve 2) -> Doğrudan milivolt
						max_hucre_mV = (RxData[1] << 8) | RxData[2];
					}

					// 0x92 Paketi: Sıcaklık
					else if (received_data_id == 0x92) {
						// Maksimum Sıcaklık (Byte 1) -> 40 ofsetli
						max_sicaklik_C = RxData[1] - 40;
					}// BİREYSEL HÜCRE VOLTAJLARI (0x95)
					else if (received_data_id == 0x95) {

						uint8_t frame_no = RxData[0]; // Paketin kaçıncı hücre grubu olduğunu gösterir (1, 2, 3...)

						// Hücre voltajları milivolt (mV) gelir. Volt'a (V) çevirmek için 1000'e bölüyoruz.
						float cell_a = ((RxData[1] << 8) | RxData[2]) / 1000.0f;
						float cell_b = ((RxData[3] << 8) | RxData[4]) / 1000.0f;
						float cell_c = ((RxData[5] << 8) | RxData[6]) / 1000.0f;

						if (frame_no == 1) {
							data1.voltage1 = cell_a;
							data1.voltage2 = cell_b;
							data1.voltage3 = cell_c;
						}
						else if (frame_no == 2) {
							data1.voltage4 = cell_a;
							data1.voltage5 = cell_b;
							data1.voltage6 = cell_c;
						}
						else if (frame_no == 3) {
							data1.voltage7 = cell_a;
							data1.voltage8 = cell_b;
							data1.voltage9 = cell_c;
						}
						else if (frame_no == 4) {
							data1.voltage10 = cell_a;
							data1.voltage11 = cell_b;
							data1.voltage12 = cell_c;
						}
						else if (frame_no == 5) {
							data1.voltage13 = cell_a;
							data1.voltage14 = cell_b;
							data1.voltage15 = cell_c;
						}
						else if (frame_no == 6) {
							data1.voltage16 = cell_a;
							data1.voltage17 = cell_b;
							data1.voltage18 = cell_c;
						}
						else if (frame_no == 7) {
							data1.voltage19 = cell_a;
							data1.voltage20 = cell_b;
							// 7. Paketteki 3. hücre (cell_c) 20 serili bataryada boştur, işleme almıyoruz.
						}
				}
			}
		}
		data1.totalVoltage = batarya_voltaji;
				data1.current = batarya_akimi;
				txtNextion("t37",data1.current);

	}
}
}*/

void getCAN1(CAN_RxHeaderTypeDef *RxHeader1, uint8_t *dataReceive) {

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
		/*case 0x0FFFFFF6:
			iz_res = dataReceive[0];
			ek_res = dataReceive[1];
			break;*/

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

void sendNextion(){
	 //HAL_Delay(10);
	maxTemp = data1.batteryTemp1;
	if (data1.batteryTemp2 > maxTemp) maxTemp = data1.batteryTemp2;
	    if (data1.batteryTemp3 > maxTemp) maxTemp = data1.batteryTemp3;
	    if (data1.batteryTemp4 > maxTemp) maxTemp = data1.batteryTemp4;

	    // --- AKILLI "WAIT" KONTROLÜ ---
	    // Eğer BMS henüz veri göndermediyse (Voltaj ve Akım 0 ise)
	    if (data1.totalVoltage == 0.0f && data1.current == 0.0f) {
	        char waitMsg[30];
	        const char* waitFields[] = {"t34", "t35", "t36", "t37", "t38"};

	        for (int k = 0; k < 5; k++) {
	            // Nextion formatına uygun (boşluksuz) string oluştur
	            int len = sprintf(waitMsg, "%s.txt=\"Wait\"", waitFields[k]);
	            HAL_UART_Transmit(&huart2, (uint8_t *)waitMsg, len, 50);
	            HAL_UART_Transmit(&huart2, (uint8_t *)nextionEnd, 3, 50);
	        }
	    }
	    else {
	        // BMS verisi geldiyse normal sayısal verileri bas
	        inttxtNextion("t34", data1.speed);

	        txtNextion("t35", maxTemp);
	        txtNextion("t36", data1.motorTemp);
	        txtNextion("t37", data1.current);
	        inttxtNextion("t38", data1.torque
	        		);
	    }


	    numNextion("n99", data1.current);
	    numNextion("n98", (int)data1.totalVoltage);
	    numNextion("n97", (int)rpm);
	    numNextion("n96", (int)data1.torque);
	    numNextion("n95", (int)data1.speed);
	    numNextion("n94", (int)filtered_rpm);
	    numNextion("n93", (int)data1.mech_power);




	   	    txtNextion("t23", data1.testTemp);
	    txtNextion("t24", data1.batteryTemp1);
	    txtNextion("t25", data1.batteryTemp2);
	    txtNextion("t26", data1.batteryTemp3);
	    txtNextion("t27", data1.batteryTemp4);

	    if (data1.batteryTemp1 >= 50 || data1.batteryTemp2 >= 50 || data1.batteryTemp3 >= 50 || data1.batteryTemp4 >= 50 || data1.testTemp >= 50) {
	        // Sıcaklık uyarısı

	    	HAL_UART_Transmit(&huart2, (uint8_t *)"p6.pic=", 8, 50);
	    	HAL_UART_Transmit(&huart2, (uint8_t *)nextionEnd, 3, 50);
	    }
	    else {
	        int alert_pic = 4;
	        picNextion("p1", alert_pic);
	    }

	    txtNextion("t22", data1.motorTemp);
	    txtNextion("t0", data1.voltage1);
	    txtNextion("t1", data1.voltage2);
	    txtNextion("t2", data1.voltage3);
	    txtNextion("t3", data1.voltage4);
	    txtNextion("t4", data1.voltage5);
	    txtNextion("t5", data1.voltage6);
	    txtNextion("t6", data1.voltage7);
	    txtNextion("t7", data1.voltage8);
	    txtNextion("t8", data1.voltage9);
	    txtNextion("t9", data1.voltage10);
	    txtNextion("t10", data1.voltage11);
	    txtNextion("t11", data1.voltage12);
	    txtNextion("t12", data1.voltage13);
	    txtNextion("t13", data1.voltage14);
	    txtNextion("t14", data1.voltage15);
	    txtNextion("t15", data1.voltage16);
	    txtNextion("t16", data1.voltage17);
	    txtNextion("t17", data1.voltage18);
	    txtNextion("t18", data1.voltage19);
	    txtNextion("t19", data1.voltage20);
	    txtNextion("t30", data1.totalVoltage);
	    txtNextion("t33", data1.totalVoltage);


	    inttxtNextion("t29", (int)batarya_yuzdesi);
	    inttxtNextion("t32", (int)batarya_yuzdesi);
	    txtNextionFormatted("t31", toplam_harcanan_Wh);
	    txtNextionFormatted("t28", toplam_harcanan_Wh);

	    if (fresult > 0) {
	        picNextion("p0", 2); // SD kart
	    }
	    else {
	        picNextion("p0", 3); // SD kart
	    }

	    txtNextionFormatted("t20", aracin_gercek_direnci);
	    if (ek_res > 0.0f) {
	    		ek_res = ek_res * 1000;
	    	        txtNextionFormatted("t21", ek_res);
	    } else {
	    	        txtNextionFormatted("t21", 0.0f);
	    	    }
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

float kalibre_et(float imd_degeri) {
		    // x: İnotel'den gelen değerler, y: Hakemin taktığı (ek_res) değerler
		    float x[] = {420.0f, 560.0f, 660.0f, 730.0f, 790.0f};
		    float y[] = {100.0f, 200.0f, 300.0f, 400.0f, 500.0f};
		    int nokta_sayisi = 5;

		    // Hakem 100k'dan küçük takarsa
		    if (imd_degeri <= x[0]) {
		        float egim = (y[1] - y[0]) / (x[1] - x[0]);
		        float sonuc = y[0] - egim * (x[0] - imd_degeri);
		        if (sonuc < 0) return 0.0f;
		        return sonuc;
		    }

		    // Hakem 100k ile 500k arası takarsa
		    for (int i = 0; i < nokta_sayisi - 1; i++) {
		        if (imd_degeri >= x[i] && imd_degeri <= x[i+1]) {
		            float oran = (imd_degeri - x[i]) / (x[i+1] - x[i]);
		            return y[i] + oran * (y[i+1] - y[i]);
		        }
		    }

		    // Hakem 500k'dan biraz daha büyük takarsa (matematiksel tahmin)
		    if(imd_degeri > x[nokta_sayisi-1]) {
		        float egim = (y[nokta_sayisi-1] - y[nokta_sayisi-2]) / (x[nokta_sayisi-1] - x[nokta_sayisi-2]);
		        return y[nokta_sayisi-1] + egim * (imd_degeri - x[nokta_sayisi-1]);
		    }

		    return 0.0f;
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

    HAL_Delay(10);
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
            // Sütun başlıkları: Zaman; Hız; RPM; Tork; Akım; Voltaj; Güç; Sıcaklık
            const char* header = "Zaman(ms);Hiz(km/h);RPM;Tork(Nm);Akim(A);Voltaj(V);Guc(W);Sicaklik(C)\n";

            if (f_puts(header, &fil) < 0) {
                f_close(&fil);
                return;
            }
            f_close(&fil);
        } else {
            // Dosya oluşturma/açma başarısız oldu
            // Hata işleme kodları buraya eklenebilir
        }
    }
}

// GSM fonksiyonları (optimize)
void send_at_command(char *command, uint16_t delay) {
    HAL_UART_Transmit(&huart5, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);
    HAL_Delay(delay);
}

void gsm_init() {
    char apn_setup[64];
    char http_init[32];
    char http_start[128];
    send_at_command("AT\r\n", 1000);
    send_at_command("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n", 1000);
    sprintf(apn_setup, "AT+SAPBR=3,1,\"APN\",\"internet\"\r\n");
    send_at_command(apn_setup, 1000);
    send_at_command("AT+SAPBR=1,1\r\n", 2000);
    send_at_command("AT+HTTPINIT\r\n", 1000);
    sprintf(http_init, "AT+HTTPPARA=\"CID\",1\r\n");
    send_at_command(http_init, 1000);
    send_at_command("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n", 1000);
    sprintf(http_start, "AT+HTTPPARA=\"URL\",\"http://maveratelemetry.somee.com/api/SensorData\"\r\n");
    send_at_command(http_start, 1000);
}

void gsm_send_data(float voltages[20], float temperatures[6], float current, float speed, float energy, float voltage_sum) {
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
    sprintf(http_data, "AT+HTTPDATA=%d,2000\r\n", strlen(json_payload));
    send_at_command(http_data, 100);
    send_at_command(json_payload, 100);
    send_at_command("AT+HTTPACTION=1\r\n", 100);
    HAL_Delay(500); // Daha kısa bekleme
}

void CalculateSpeed(TIM_HandleTypeDef *htimPin, uint32_t channel) {
    last_pulse_tick = HAL_GetTick();

    if (is_first_capture) {
        last_capture_val = HAL_TIM_ReadCapturedValue(htimPin, channel);
        last_overflow_count = overflow_count;
        data1.speed = 0.0f;
        is_first_capture = 0;
    } else {
        current_capture_val = HAL_TIM_ReadCapturedValue(htimPin, channel);
        uint32_t current_overflow_count = overflow_count;

        if (__HAL_TIM_GET_FLAG(htimPin, TIM_FLAG_UPDATE) != RESET && current_capture_val < 32768) {
            current_overflow_count++;
        }

        uint32_t current_total_ticks = (current_overflow_count * 65536UL) + current_capture_val;
        uint32_t last_total_ticks = (last_overflow_count * 65536UL) + last_capture_val;
        uint32_t diff_ticks = current_total_ticks - last_total_ticks;

        // Gürültü Filtresi: 300 tick altını kesinlikle yoksay
        if (diff_ticks > 300) {

            // 1. Yeni süreyi tampona (array) kaydet
            tick_buffer[tick_index] = diff_ticks;
            tick_index = (tick_index + 1) % MAGNET_COUNT;

            if (valid_pulses < MAGNET_COUNT) {
                valid_pulses++;
            }

            // 2. Sadece 1 tam tur (4 mıknatıs) tamamlandığında hızı hesapla
            if (valid_pulses == MAGNET_COUNT) {
                uint32_t sum_ticks = 0;
                for (int i = 0; i < MAGNET_COUNT; i++) {
                    sum_ticks += tick_buffer[i];
                }

                // 3. Bir tam turun süresiyle kusursuz SAF MOTOR DEVRİ (Mıknatısa bölmeye gerek kalmaz)
                float anlik_motor_rpm = (60.0f * TIMER_FREQ) / sum_ticks;

                // 4. Anlık devri filtreye at (Tepki hızlandırıldı: 0.4 eski, 0.6 yeni)
                filtered_rpm = 0.4f * filtered_rpm + 0.6f * anlik_motor_rpm;

                // 5. TEKERLEK DEVRİ (Redüktör için 6'ya bölüyoruz, senin istediğin ~490 değeri)
                float tekerlek_rpm = filtered_rpm / 6.0f;
                rpm = tekerlek_rpm; // Takometre verin ve Ekrana gidecek olan RPM

                // 6. Araç Hızı
                float speed_ms = (tekerlek_rpm / 60.0f) * WHEEL_CIRCUMFERENCE;
                hall_speed = speed_ms * 3.6f;
                data1.speed = hall_speed;
            }

            last_capture_val = current_capture_val;
            last_overflow_count = current_overflow_count;
        }
    }
}

/*void CalculateSpeed(TIM_HandleTypeDef *htimPin, uint32_t channel) {
    last_pulse_tick = HAL_GetTick();

    if (is_first_capture) {
        last_capture_val = HAL_TIM_ReadCapturedValue(htimPin, channel);
        last_overflow_count = overflow_count;
        data1.speed = 0.0f; // 1 yerine 0 olmalı, aksi takdirde kalkışta 1km/h sıçrama yapar
        is_first_capture = 0;
    } else {
        current_capture_val = HAL_TIM_ReadCapturedValue(htimPin, channel);
        uint32_t current_overflow_count = overflow_count;


        if (__HAL_TIM_GET_FLAG(htimPin, TIM_FLAG_UPDATE) != RESET && current_capture_val < 32768) {
            current_overflow_count++;
        }


        // 32-bit uzaya taşıyarak if-else karmaşasından ve wrap-around hatalarından kurtuluyoruz
        uint32_t current_total_ticks = (current_overflow_count * 65536UL) + current_capture_val;
        uint32_t last_total_ticks = (last_overflow_count * 65536UL) + last_capture_val;
        uint32_t diff_ticks = current_total_ticks - last_total_ticks;

        // Gürültü Filtresi: 100 tick'ten büyükse geçerli say
        if (diff_ticks > 100) {

        	rpm = (60.0f * TIMER_FREQ) / (MAGNET_COUNT * diff_ticks);

        	rpm = rpm / 6;

            filtered_rpm = 0.8f * filtered_rpm + 0.2f * rpm;



            float speed_ms = (filtered_rpm / 60.0f) * WHEEL_CIRCUMFERENCE;
            hall_speed = speed_ms * 3.6f;
            data1.speed = hall_speed;

            // SADECE geçerli bir okuma yapıldığında referans noktasını güncelle
            last_capture_val = current_capture_val;
            last_overflow_count = current_overflow_count;
        }


    }
}*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        overflow_count++;
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
#ifdef USE_FULL_ASSERT
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
