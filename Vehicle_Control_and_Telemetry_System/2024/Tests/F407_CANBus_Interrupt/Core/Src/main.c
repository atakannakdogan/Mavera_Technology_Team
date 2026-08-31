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
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
float iz_res, ek_res;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void getCAN1(CAN_RxHeaderTypeDef *RxHeader1, float *dataReceive);
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  iz_res = 5;
	  ek_res = iz_res + ek_res;
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  // CAN filtresi yapılandırması
  CAN_FilterTypeDef canFilterConfig;
  canFilterConfig.FilterBank = 18;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x000<<5;
  canFilterConfig.FilterIdLow = 0;
  canFilterConfig.FilterMaskIdHigh = 0x000<<5;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.SlaveStartFilterBank = 20;
  if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }


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

void getCAN1(CAN_RxHeaderTypeDef *RxHeader1, float *dataReceive) {

	//HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, dataReceive);
	switch (RxHeader1->ExtId) {
		case 0x0FFFFFF0:
			data1.voltage1 = dataReceive[0];
			data1.voltage2 = dataReceive[1];
			break;
		case 0x0FFFFFF1:
			data1.voltage3 = dataReceive[0];
			data1.voltage4 = dataReceive[1];
			break;
		case 0x0FFFFFF2:
			data1.voltage5 = dataReceive[0];
			data1.voltage6 = dataReceive[1];
			break;
		case 0x0FFFFFF3:
			data1.voltage7 = dataReceive[0];
			data1.voltage8 = dataReceive[1];
			break;
		case 0x0FFFFFF4:
			data1.voltage9 = dataReceive[0];
			data1.voltage10 = dataReceive[1];
			break;
		case 0x0FFFFFF5:
			data1.batteryTemp1 = dataReceive[0];
			data1.batteryTemp2 = dataReceive[1];
			break;
		case 0x0FFFFFF6:
			iz_res = dataReceive[0];
			ek_res = dataReceive[1];
			break;
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
		default:
			// Handle unexpected ID
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
