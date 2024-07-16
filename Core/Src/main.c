/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#include "string.h"
#include "stdio.h"
#include "stdlib.h"


#include "GFX_FUNCTIONS.h"
#include "fonts.h"
#include "ST7735.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
	INICIO,
	GUARDANDO_EN_PING,
	GUARDANDO_EN_PONG,
	PAUSA

}estados_t;

typedef enum{

	//start_adc,
	boton_presionado,
	evt_buffer_lleno,
	trigger_disparado,
	nulo

}eventos_t;


#define OFFSET 1
#define BYTE_START 0xAA


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX 25
#define TAMANO 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

volatile uint8_t buffer_lleno_flag = 0;
volatile uint8_t UART_TX_PING_flag = 0;
volatile uint8_t UART_TX_PONG_flag = 0;

uint8_t bufferPING[TAMANO];
uint8_t bufferPONG[TAMANO];

uint8_t *puntero_escritura;
uint8_t *puntero_lectura;
uint8_t *posicion_final_puntero_escritura;
uint8_t *posicion_final_puntero_lectura;

uint16_t debug;

//Modificaciones 05/07

volatile uint8_t trigger_valor = 0;
uint8_t trigger_flag = 0;
uint16_t trigger_contador = 0;

#define MAX_BUFFER_TRIGGER 6


//Modificaciones 16/07

volatile uint8_t CONFIGURACION_flag = 1;


#define MAX_X 160-1
#define MAX_Y 128-1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void UART_TX_PING();
void UART_TX_PONG();
uint8_t ADC_Conversion();
void fsm(estados_t *estado, eventos_t eventos);
void Swap_Buffer(uint8_t swap_condicion);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
estados_t estado = INICIO;
eventos_t evt = evt_buffer_lleno;

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
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //La maquina de estados comienza desde el estado de configuracion, por tanto se inicializa el ADC que
  //mide el potenciometro del trigger y la pantalla.


  //Inicializacion de la pantalla.
  ST7735_Init(0);
  fillScreen(BLACK);
  //testAll();

  //Para almacenar en formato string el valor medido del trigger.
  char value[4];
  uint8_t valor_anterior;


  ST7735_SetRotation(2);


  //bufferPING[0] = BYTE_START;
  //bufferPONG[0] = BYTE_START;


  while (1)
  {

	  if(UART_TX_PING_flag == 1){

		  UART_TX_PING_flag = 0;
		  UART_TX_PING();
	  }
	  if(UART_TX_PONG_flag == 1){

		  UART_TX_PONG_flag = 0;
		  UART_TX_PONG();
	  }
	  if(CONFIGURACION_flag == 1){
			HAL_ADC_Start(&hadc2);
			if(HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK){

				trigger_valor = (HAL_ADC_GetValue(&hadc2)>>4) & 0XFF;
			}


			if(valor_anterior != trigger_valor){
				fillScreen(WHITE);
				itoa(trigger_valor, value, 10);
				ST7735_WriteString(0, 0, value, Font_11x18, RED, BLACK);

			}
			valor_anterior = trigger_valor;
			HAL_Delay(500);
	  }

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

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart3.Init.BaudRate = 950000;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_TX;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TIMER_INTERRUPT_Pin|DEBUG_BUFFER_LLENO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|DC_Pin|RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TIMER_INTERRUPT_Pin DEBUG_BUFFER_LLENO_Pin */
  GPIO_InitStruct.Pin = TIMER_INTERRUPT_Pin|DEBUG_BUFFER_LLENO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOTON_Pin */
  GPIO_InitStruct.Pin = BOTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin DC_Pin RESET_Pin */
  GPIO_InitStruct.Pin = CS_Pin|DC_Pin|RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if(GPIO_Pin == BOTON_Pin){

		evt = boton_presionado;
		fsm(&estado, evt);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

	if(htim->Instance == TIM2){
		HAL_GPIO_TogglePin(GPIOA, TIMER_INTERRUPT_Pin);

		switch(ADC_Conversion()){
		case 1:
			HAL_GPIO_WritePin(DEBUG_BUFFER_LLENO_GPIO_Port, DEBUG_BUFFER_LLENO_Pin, GPIO_PIN_SET);
			evt = evt_buffer_lleno;
			fsm(&estado, evt);
			HAL_GPIO_WritePin(DEBUG_BUFFER_LLENO_GPIO_Port, DEBUG_BUFFER_LLENO_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			evt = trigger_disparado;
			fsm(&estado, evt);
			break;
		default:
			break;
		}

	}

}

uint8_t ADC_Conversion(){
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK){

		*puntero_escritura = (HAL_ADC_GetValue(&hadc1)>>4) & 0XFF;
		//*(puntero_escritura + 1) = HAL_ADC_GetValue(&hadc1) & 0XFF;

		//if( (flag == 1) && (*puntero_escritura >= trigger_valor)){
		if( *puntero_escritura >= trigger_valor){

			trigger_flag = 1;
			//flag = 0;
		}

		puntero_escritura = puntero_escritura + OFFSET;		//Se incrementa el puntero.



		if(puntero_escritura >= posicion_final_puntero_escritura){

			if(trigger_flag == 1){
				trigger_contador++;

				if(trigger_contador >= MAX_BUFFER_TRIGGER){
					trigger_contador = 0;
					trigger_flag = 0;
					return 2;
				}
			}

			return 1;
		}
		else{
			return 0;
		}

	}



	else{
		return -1;
	}
}

void Swap_Buffer(uint8_t swap_condicion){
	if(swap_condicion == 0){
		posicion_final_puntero_escritura = bufferPING + TAMANO;
		puntero_escritura = bufferPING;
	}
	else{
		posicion_final_puntero_escritura = bufferPONG + TAMANO;
		puntero_escritura = bufferPONG;
	}
}

void UART_TX_PING(){


	//HAL_GPIO_WritePin(GPIOA, BUFF_PING_Pin, GPIO_PIN_SET);

	HAL_UART_Transmit(&huart3, bufferPING, sizeof(bufferPING), 10);

	//HAL_GPIO_WritePin(GPIOA, BUFF_PING_Pin, GPIO_PIN_RESET);
}


void UART_TX_PONG(){

	//HAL_GPIO_WritePin(GPIOA, BUFF_PONG_Pin, GPIO_PIN_SET);

	HAL_UART_Transmit(&huart3, bufferPONG, sizeof(bufferPONG), 10);

	//HAL_GPIO_WritePin(GPIOA, BUFF_PONG_Pin, GPIO_PIN_RESET);
}


void fsm(estados_t *estado, eventos_t eventos){

	estados_t estado_anterior = *estado;

	if(estado_anterior == INICIO){
		switch(eventos){
		case boton_presionado:
			*estado = GUARDANDO_EN_PING;
			break;
		default:
			break;
			}
	}
	if(estado_anterior == GUARDANDO_EN_PING){
		switch(eventos){
			case trigger_disparado:
				*estado = PAUSA;
				break;
			case evt_buffer_lleno:
				*estado = GUARDANDO_EN_PONG;
				break;
			/*case boton_presionado:
				*estado = PAUSA;*/
				break;
			default:
				break;
			}
	}
	if(estado_anterior == GUARDANDO_EN_PONG){
		switch(eventos){
			case trigger_disparado:
				*estado = PAUSA;
				break;
			case evt_buffer_lleno:
				*estado = GUARDANDO_EN_PING;
				break;
			default:
				break;
		}
	}
	if(estado_anterior == PAUSA){
		switch(eventos){
			case boton_presionado:
				*estado = INICIO;
				break;
			default:
				break;
		}
	}



	if(estado_anterior != *estado){
		switch(estado_anterior){
		case INICIO:
			//Ya no me encuentro en el estado de configuracion, pongo el flag en 0.
			CONFIGURACION_flag = 0;
			HAL_ADC_Stop(&hadc2);		//Detengo el ADC que mide el potenciometro para el valor del trigger.

			Swap_Buffer(0);
			HAL_TIM_Base_Start_IT(&htim2);
			HAL_ADC_Start(&hadc1);
			break;
		case GUARDANDO_EN_PING:
			Swap_Buffer(1);
			UART_TX_PING_flag = 1;

			if(*estado == PAUSA){
				HAL_TIM_Base_Stop_IT(&htim2);
				HAL_ADC_Stop(&hadc1);
			}

			break;
		case GUARDANDO_EN_PONG:
			Swap_Buffer(0);
			UART_TX_PONG_flag = 1;

			if(*estado == PAUSA){
				HAL_TIM_Base_Stop_IT(&htim2);
				HAL_ADC_Stop(&hadc1);
			}

			break;
		case PAUSA:
			CONFIGURACION_flag = 1;
			HAL_ADC_Start(&hadc2);		//Inicio el ADC del pot. del trigger.
			break;

		default:
			break;
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
