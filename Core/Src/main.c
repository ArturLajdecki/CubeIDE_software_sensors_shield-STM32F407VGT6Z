/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "i2c-lcd.h"
#include "i2c-eeprom.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 4095 // Maximal value of ADCread
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFF7A2A)) //Raw data acquired at temperature of 30 °C VDDA = 3.3 V
															  //ST did a measurement for us when manufacturing the chip,
															  //using a precise external reference voltage on the VREF+ pin,
				 	 	 	 	 	 	 	 	 	 	 	  //and stored the resulting ADC reading into the system memory

#define VCAL 3.3	//The voltage used as external reference at calibration. Put as 330 instead 3.3V to avoid float, vmeasured would be in 10mV units

#define _NTC_R_SERIES         10000.0f
#define _NTC_R_NOMINAL        10000.0f
#define _NTC_TEMP_NOMINAL     25.0f
#define _NTC_ADC_MAX          4096 //  12bit
#define _NTC_BETA             3950

#define NTC_QUANTITY 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
	uint8_t error_msg[] = "COM error.\n\r";
	uint8_t Received[13];
	uint16_t ADCread[5];

	float NTC_temp[4];
	float MAX_temp[4];
	_Bool flag_temp[4];
	_Bool flag_eeprom[4];

	uint8_t data[4][20];


	float Rntc = 0; 		//Resistance of the thermistor
	float temp = 0; 		//Calculated temperature in Celcius
	float vrefint = 0; 	//Reference internal voltage
	float vrefext = 0; 	//External voltage calculated based on vrefint
	float vmeas = 0; 	//Measured voltage in volts [V]

	 static uint8_t counter = 1;

	 uint8_t data_write = 90;
	 char data_read0[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM10)
 { // Jeżeli przerwanie pochodzi od timera 10

	 //TEMP1
	 if(flag_temp[0] == 1)
	 {

		 if(NTC_temp[0] >= MAX_temp[0])
		 {
			 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_RESET);
			 flag_temp[0] = 0;
		 }
		 else
		 {
			 sprintf(data[0], "01010101%3.2f\r\n", NTC_temp[0]);
		 }

	 }

	 //TEMP2
	 if(flag_temp[1] == 1)
	 	 {

	 		 if(NTC_temp[1] >= MAX_temp[1])
	 		 {
	 			 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,GPIO_PIN_RESET);
	 			 flag_temp[1] = 0;
	 		 }
	 		 else
	 		 {
	 			 sprintf(data[1], "01010102%3.2f\r\n", NTC_temp[1]);
	 		 }

	 	 }

	 //TEMP3
	 if(flag_temp[2] == 1)
	 	 	 {

	 	 		 if(NTC_temp[2] >= MAX_temp[2])
	 	 		 {
	 	 			 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,GPIO_PIN_RESET);
	 	 			 flag_temp[2] = 0;
	 	 		 }
	 	 		 else
	 	 		 {
	 	 			 sprintf(data[2], "01010103%3.2f\r\n", NTC_temp[2]);
	 	 		 }

	 	 	 }
	 //TEMP4
	 if(flag_temp[3] == 1)
	 	 	 {

	 	 		 if(NTC_temp[3] >= MAX_temp[3])
	 	 		 {
	 	 			 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,GPIO_PIN_RESET);
	 	 			 flag_temp[3] = 0;
	 	 		 }
	 	 		 else
	 	 		 {
	 	 			 sprintf(data[3], "01010104%3.2f\r\n", NTC_temp[3]);
	 	 		 }

	 	 	 }


	if(counter>4)
	{
		counter=1;
	}
	else
	{
		counter++;
	}

	 switch(counter)
	 {
	 	case 1:
	 	 if(flag_temp[0] == 1)
	 	 {
	 		HAL_UART_Transmit_IT(&huart1, (uint8_t*) data[0], strlen(data[0]));

	 		lcd_put_cur(0, 3);
  			lcd_send_float (NTC_temp[0], 5);

	 	 }
		break;

	 	case 2:
	 		if(flag_temp[1] == 1)
	 		{
				HAL_UART_Transmit_IT(&huart1, (uint8_t*) data[1], strlen(data[1]));

				lcd_put_cur(1, 3);
				lcd_send_float (NTC_temp[1], 5);
	 		}
	 	break;

	 	case 3:
	 		if(flag_temp[2] == 1)
	 		{
	 			HAL_UART_Transmit_IT(&huart1, (uint8_t*) data[2], strlen(data[2]));

				lcd_put_cur(2, 3);
				lcd_send_float (NTC_temp[2], 5);
	 		}
	 	break;

	 	case 4:
	 		if(flag_temp[3] == 1)
	 		{
	 			HAL_UART_Transmit_IT(&huart1, (uint8_t*) data[3], strlen(data[3]));

				lcd_put_cur(3, 3);
				lcd_send_float (NTC_temp[3], 5);
	 		}
	 	break;

	 	default:

	 	break;
	 }

 }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static float result = 0;

	 if(Received[0] == '1' && Received[1] == '0' && Received[2] == '1' && Received[3] == '0' && Received[4] == '1' && Received[5] == '0' && Received[6] == '1')
	 {

		 switch(Received[7])
		 {
		 case '1':

			 for(int i=8; i<13; i++){
				 result = result * 10 + ( Received[i] - '0' );
				 }

			 MAX_temp[0] = result/100;
			 result = 0;

			 flag_eeprom[0] = 1;

		     break;

		 case '2':

			 for(int i=8; i<13; i++){
				 result = result * 10 + ( Received[i] - '0' );
				 }

			 MAX_temp[1] = result/100;
			 result = 0;

			 flag_eeprom[1] = 1;

		     break;

		 case '3':

			 for(int i=8; i<13; i++){
				 result = result * 10 + ( Received[i] - '0' );
				 }

			 MAX_temp[2] = result/100;
			 result = 0;

			 flag_eeprom[2] = 1;

		     break;

		 case '4':

			 for(int i=8; i<13; i++){
				 result = result * 10 + ( Received[i] - '0' );
				 }

			 MAX_temp[3] = result/100;
			 result = 0;

			 flag_eeprom[3] = 1;

			 break;

		 case '5':

			 if(flag_temp[0] == 0)
			 {
				 flag_temp[0] = 1;
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_SET);
			 }
			 else if(flag_temp[0] == 1)
			 {
				 flag_temp[0] = 0;
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_RESET);
			 }
			 break;

		 case '6':

			 if(flag_temp[1] == 0)
			 {
				 flag_temp[1] = 1;
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,GPIO_PIN_SET);
			 }
			 else if(flag_temp[1] == 1)
			 {
				 flag_temp[1] = 0;
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,GPIO_PIN_RESET);
			 }
			 break;

		 case '7':

			 if(flag_temp[2] == 0)
			 {
				 flag_temp[2] = 1;
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,GPIO_PIN_SET);
			 }
			 else if(flag_temp[2] == 1)
			 {
				 flag_temp[2] = 0;
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,GPIO_PIN_RESET);
			 }
			 break;

		 case '8':

			 if(flag_temp[3] == 0)
			 {
				 flag_temp[3] = 1;
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,GPIO_PIN_SET);
			 }
			 else if(flag_temp[3] == 1)
			 {
				 flag_temp[3] = 0;
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,GPIO_PIN_RESET);
			 }
			 break;

		 default:

		     break;
		 }
	 }
	 else
	 {
		 HAL_UART_Transmit_IT(&huart1, (uint8_t*)error_msg, 12);
	 }


	 HAL_UART_Receive_DMA(&huart1, Received, 13); // Ponowne włączenie nasłuchiwania
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	// Can be used on integers for more precision allegedly but I don't know how to calculate it without overflowing. (VCAL 330 instead 3.3)
		  //vmeas = (VCAL * ADCread[0] * (*TEMP30_CAL_ADDR)+ (ADCread[1]* ADC_BUF_LEN)/2)/(ADCread[1]*ADC_BUF_LEN); // Calculating voltage in 10mv units

	for(int i=0; i<4; i++){
		vrefint = VCAL * (*TEMP30_CAL_ADDR) / ADC_BUF_LEN;
		vrefext = vrefint * ADC_BUF_LEN / ADCread[4];
		vmeas = vrefext * ADCread[i] / ADC_BUF_LEN;

		// Calculating Resistance of NTC Vmeasured*100kOhms
		Rntc= (vmeas*(float)_NTC_R_NOMINAL)/(vrefext-vmeas);

		// Calculating Temperature
		NTC_temp[i] = Rntc/(float)_NTC_R_NOMINAL;
		NTC_temp[i] = logf(NTC_temp[i]);
		NTC_temp[i] = NTC_temp[i]/(float)_NTC_BETA;
		NTC_temp[i] += 1.0f / ((float)_NTC_TEMP_NOMINAL + 273.15f);
		NTC_temp[i] = 1.0f/NTC_temp[i];
		NTC_temp[i] -= 273.15f;
	}


		  //temp_in_celsius = temp + 273.15; //temperature in Celsius

}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  for(int i = 0; i < NTC_QUANTITY; i++)
  {
	  MAX_temp[i] = eeprom_read(i);
  }

	lcd_init();
	HAL_Delay(100);
	lcd_clear();
	HAL_Delay(100);

	lcd_put_cur(0, 0);
	lcd_send_string("T1:");

	lcd_put_cur(1, 0);
	lcd_send_string("T2:");

	lcd_put_cur(2, 0);
	lcd_send_string("T3:");

	lcd_put_cur(3, 0);
	lcd_send_string("T4:");

	HAL_TIM_Base_Start_IT(&htim10);
	HAL_UART_Receive_DMA(&huart1, Received, 13);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCread, 5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
	  if (flag_eeprom[0] == 1)
	  {
		  eeprom_write(0,MAX_temp[0]);
		  flag_eeprom[0] = 0;
	  }
	  if (flag_eeprom[1] == 1)
	  {
		  eeprom_write(1,MAX_temp[1]);
		  flag_eeprom[1] = 0;
	  }
	  if (flag_eeprom[2] == 1)
	  {
		  eeprom_write(2,MAX_temp[2]);
		  flag_eeprom[2] = 0;
	  }
	  if (flag_eeprom[3] == 1)
	  {
		  eeprom_write(3,MAX_temp[3]);
		  flag_eeprom[3] = 0;
	  }

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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
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
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 5;
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
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 65535;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 320;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
