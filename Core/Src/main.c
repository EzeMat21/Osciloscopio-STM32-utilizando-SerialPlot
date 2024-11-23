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
#include "stm32f1xx_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
	INICIO,
	C_RESOLUCION,
	C_RESOLUCION_8bit,
	C_RESOLUCION_12bit,
	C_TRIGGER,
	C_TRIGGER_ASC,
	C_TRIGGER_DESC,
	C_MUESTRAS,
	C_MUESTRAS_500,
	C_MUESTRAS_1000,
	GUARDANDO_EN_PING,
	GUARDANDO_EN_PONG,
	PAUSA,
	TRIGGER_TERMINADO

}estados_t;

typedef enum{

	//INICIO_adc,
	boton_OK,
	boton_FLECHA,
	boton_ESC,
	evt_buffer_lleno,
	trigger_disparado

}eventos_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TAMANO 100

#define MAX_Y 160-1
#define MAX_X 128-1

#define color(r, g, b) ((r << 11) | (g  << 5) | ( b))


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

volatile uint16_t trigger_valor = 127;
volatile uint8_t trigger_flag = 1;
volatile uint8_t trigger_activado = 0;
volatile uint16_t trigger_contador = 0;
volatile uint8_t max_buff_trigger = 6;

volatile uint8_t trigger_en_condiciones = 0;

volatile uint8_t ADC_TRIGGER_flag = 0;
volatile uint8_t OFFSET = 1;			//Si el offset es 1 entonces estamos en resol de 8bits, si es 2 entonces estamos en resol. de 12bits.
volatile uint8_t flanco = 0;

volatile uint8_t muestras = 0;

volatile uint16_t trigger_valor_09 = 127 - (127>>3);
volatile uint16_t trigger_valor_11 = 127 + (127>>3);

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


void plotConfig();
void plotResolucion();
void plotTrigger();
void plotMuestras();
void plotInicio();
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
  ST7735_Init(2);
  ST7735_SetRotation(2);
  plotConfig();
  fillCircle( 21, 75, 5, BLUE);

  //Para almacenar en formato string el valor medido del trigger.
  char value[4];
  uint16_t valor_anterior = 0;


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
	  if(ADC_TRIGGER_flag == 1){

			HAL_ADC_Start(&hadc2);
			if(HAL_ADC_PollForConversion(&hadc2, 50) == HAL_OK){


				if(OFFSET == 1){
					trigger_valor = (HAL_ADC_GetValue(&hadc2)>>4) & 0XFF;
				}
				else{
					trigger_valor = HAL_ADC_GetValue(&hadc2) & 0XFFF;
				}

				if(valor_anterior != trigger_valor){

					fillRect(45, 50, 45, 15, BLACK);
					itoa(trigger_valor, value, 10);
					ST7735_WriteString(45, 50, value, Font_11x18, color(31,44,0), BLACK);

				}

				valor_anterior = trigger_valor;
				HAL_Delay(300);
			}
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
  htim2.Init.Period = 17-1;
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
  HAL_GPIO_WritePin(GPIOA, DEBUG_TIMER_INTERRUPT_Pin|DEBUG_BUFFER_LLENO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|DC_Pin|RESET_Pin|DEBUG_ADC1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_TIMER_INTERRUPT_Pin DEBUG_BUFFER_LLENO_Pin */
  GPIO_InitStruct.Pin = DEBUG_TIMER_INTERRUPT_Pin|DEBUG_BUFFER_LLENO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BOTON_OK_Pin BOTON_FLECHA_Pin BOTON_ESC_Pin */
  GPIO_InitStruct.Pin = BOTON_OK_Pin|BOTON_FLECHA_Pin|BOTON_ESC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin DC_Pin RESET_Pin DEBUG_ADC1_Pin */
  GPIO_InitStruct.Pin = CS_Pin|DC_Pin|RESET_Pin|DEBUG_ADC1_Pin;
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

void plotConfig(){

	  fillScreen(BLACK);
	  drawRoundRect(0, 0, MAX_X, MAX_Y, 10, BLUE);


	  ST7735_WriteString(18, 10, "OSCILOSCOPIO", Font_7x10, WHITE, BLACK);
	  ST7735_WriteString(18, 25, "CONFIGURACION", Font_7x10, WHITE, BLACK);


	  ST7735_WriteString(35, 70, "Inicio", Font_7x10, BLACK, YELLOW);
	  ST7735_WriteString(35, 90, "Resolucion", Font_7x10, BLACK, YELLOW);
	  ST7735_WriteString(35, 110, "Trigger", Font_7x10, BLACK, YELLOW);
	  ST7735_WriteString(35, 130, "Muestras", Font_7x10, BLACK, YELLOW);
}

void plotInicio(){

	  fillScreen(BLACK);
	  drawRoundRect(0, 0, MAX_X, MAX_Y, 10, color(3,47,3));

	  ST7735_WriteString(40, 40, "MODO: ", Font_11x18, WHITE, BLACK);
	  ST7735_WriteString(20, 80, "Transmitiendo", Font_7x10, GREEN, BLACK);
}

void plotResolucion(){

	fillScreen(BLACK);
	drawRoundRect(0, 0, MAX_X, MAX_Y, 10, color(4,47,25));
	ST7735_WriteString(10, 10, "Configuracion", Font_7x10, WHITE, BLACK);
	ST7735_WriteString(10, 25, "de la Resolucion", Font_7x10, WHITE, BLACK);

	ST7735_WriteString(50, 70, "8 bits", Font_7x10, WHITE, BLACK);
	ST7735_WriteString(50, 90, "12 bits", Font_7x10, WHITE, BLACK);

}

void plotTrigger(){

	fillScreen(BLACK);
	drawRoundRect(0, 0, MAX_X, MAX_Y, 10, color(26,34,31));
	ST7735_WriteString(15, 10, "Configuracion", Font_7x10, WHITE, BLACK);
	ST7735_WriteString(15, 25, "del Trigger", Font_7x10, WHITE, BLACK);


	ST7735_WriteString(35, 100, "Ascendente", Font_7x10, WHITE, BLACK);
	ST7735_WriteString(35, 120, "Descendente", Font_7x10, WHITE, BLACK);

}

void plotMuestras(){

	fillScreen(BLACK);
	drawRoundRect(0, 0, MAX_X, MAX_Y, 10, color(0,58,31));
	ST7735_WriteString(15, 10, "Configuracion", Font_7x10, WHITE, BLACK);
	ST7735_WriteString(15, 25, "de las muestras", Font_7x10, WHITE, BLACK);


	ST7735_WriteString(35, 80, "500 muestras", Font_7x10, WHITE, BLACK);
	ST7735_WriteString(35, 100, "1000 muestras", Font_7x10, WHITE, BLACK);

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if(GPIO_Pin == BOTON_OK_Pin){

		evt = boton_OK;
		fsm(&estado, evt);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	if(GPIO_Pin == BOTON_FLECHA_Pin){

		evt = boton_FLECHA;
		fsm(&estado, evt);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	if(GPIO_Pin == BOTON_ESC_Pin){

		evt = boton_ESC;
		fsm(&estado, evt);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

	if(htim->Instance == TIM2){
		HAL_GPIO_TogglePin(GPIOA, DEBUG_TIMER_INTERRUPT_Pin);

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


	uint16_t valor_medido = 0;
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK) {
	    //HAL_GPIO_WritePin(GPIOB, DEBUG_ADC1_Pin, GPIO_PIN_SET);

	    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
	    if (OFFSET == 1) {
	        *puntero_escritura = (adc_value >> 4) & 0xFF;
	        valor_medido = *puntero_escritura;
	    } else { // OFFSET = 2
	        *puntero_escritura = (adc_value >> 8) & 0x0F;
	        *(puntero_escritura + 1) = adc_value & 0xFF;
	        //valor_medido = ((*puntero_escritura) << 8) + *(puntero_escritura + 1);
	        valor_medido = adc_value;
	    }


	    if (trigger_flag == 1) {
	        if ((flanco == 0 && (valor_medido < trigger_valor_09)) || (flanco == 1 && (valor_medido > trigger_valor_11)) ) {
	            trigger_en_condiciones = 1;
	            trigger_flag = 0;
	        }
	    }

	    if (trigger_en_condiciones == 1) {
	        if ((flanco == 0 && (valor_medido > trigger_valor_11)) || (flanco == 1 && (valor_medido < trigger_valor_09)) ) {
	            trigger_activado = 1;
	            trigger_en_condiciones = 0;
	        }
	    }

	    puntero_escritura += OFFSET;

	    if (puntero_escritura >= posicion_final_puntero_escritura) {
	        if (trigger_activado == 1) {
	            if (++trigger_contador >= max_buff_trigger) {
	                trigger_contador = 0;
	                trigger_activado = 0;
	                trigger_flag = 1;

	                return 2;
	            }
	        }
	        return 1;
	    } else {

	        return -1;
	    }
	} else {
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

void fsm(estados_t *estado, eventos_t eventos) {
    estados_t estado_anterior = *estado;

    switch (estado_anterior) {
        case GUARDANDO_EN_PING:
        case GUARDANDO_EN_PONG:
            if (eventos == boton_FLECHA) {
                *estado = PAUSA;
            } else if (eventos == evt_buffer_lleno) {
                *estado = (estado_anterior == GUARDANDO_EN_PING) ? GUARDANDO_EN_PONG : GUARDANDO_EN_PING;
            } else if (eventos == trigger_disparado) {
                *estado = TRIGGER_TERMINADO;
            }
            break;
        case INICIO:
            if (eventos == boton_OK) {
                *estado = GUARDANDO_EN_PING;
            } else if (eventos == boton_FLECHA) {
                *estado = C_RESOLUCION;
            }
            break;
        case PAUSA:
            if (eventos == boton_OK) {
                *estado = GUARDANDO_EN_PING;
            } else if (eventos == boton_ESC) {
                *estado = INICIO;
            }
            break;
        case TRIGGER_TERMINADO:
            if (eventos == boton_ESC) {
                *estado = INICIO;
            }
            break;
        case C_RESOLUCION:
            if (eventos == boton_OK) {
                *estado = (OFFSET == 1) ? C_RESOLUCION_8bit : C_RESOLUCION_12bit;
            } else if (eventos == boton_FLECHA) {
                *estado = C_TRIGGER;
            }
            break;
        case C_RESOLUCION_8bit:
            if (eventos == boton_ESC) {
                *estado = C_RESOLUCION;
            } else if (eventos == boton_FLECHA) {
                *estado = C_RESOLUCION_12bit;
            }
            break;
        case C_RESOLUCION_12bit:
            if (eventos == boton_ESC) {
                *estado = C_RESOLUCION;
            } else if (eventos == boton_FLECHA) {
                *estado = C_RESOLUCION_8bit;
            }
            break;
        case C_TRIGGER:
            if (eventos == boton_OK) {
                *estado = (flanco == 0) ? C_TRIGGER_ASC : C_TRIGGER_DESC;
            } else if (eventos == boton_FLECHA) {
                *estado = C_MUESTRAS;
            }
            break;
        case C_TRIGGER_ASC:
            if (eventos == boton_ESC) {
                *estado = C_TRIGGER;
            } else if (eventos == boton_FLECHA) {
                *estado = C_TRIGGER_DESC;
            }
            break;
        case C_TRIGGER_DESC:
            if (eventos == boton_ESC) {
                *estado = C_TRIGGER;
            } else if (eventos == boton_FLECHA) {
                *estado = C_TRIGGER_ASC;
            }
            break;
        case C_MUESTRAS:
            if (eventos == boton_OK) {
                *estado = (muestras == 0) ? C_MUESTRAS_500 : C_MUESTRAS_1000;
            } else if (eventos == boton_FLECHA) {
                *estado = INICIO;
            }
            break;
        case C_MUESTRAS_500:
            if (eventos == boton_ESC) {
                *estado = C_MUESTRAS;
            } else if (eventos == boton_FLECHA) {
                *estado = C_MUESTRAS_1000;
            }
            break;
        case C_MUESTRAS_1000:
            if (eventos == boton_ESC) {
                *estado = C_MUESTRAS;
            } else if (eventos == boton_FLECHA) {
                *estado = C_MUESTRAS_500;
            }
            break;
        default:
            break;
    }

    if (estado_anterior != *estado) {
        switch (estado_anterior) {
            case GUARDANDO_EN_PING:
                Swap_Buffer(1);
                UART_TX_PING_flag = 1;
                if (*estado == PAUSA || *estado == TRIGGER_TERMINADO) {
                    HAL_TIM_Base_Stop_IT(&htim2);
                    HAL_ADC_Stop(&hadc1);
                    fillRect(20, 80, 100, 15, BLACK);
                    if (*estado == PAUSA) {
                        ST7735_WriteString(40, 80, "Pausado", Font_7x10, RED, BLACK);
                    } else {
                        ST7735_WriteString(40, 80, "Trigger", Font_7x10, color(27, 57, 1), BLACK);
                        ST7735_WriteString(35, 100, "Disparado", Font_7x10, color(27, 57, 1), BLACK);
                    }
                }
                break;
            case GUARDANDO_EN_PONG:
                Swap_Buffer(0);
                UART_TX_PONG_flag = 1;
                if (*estado == PAUSA || *estado == TRIGGER_TERMINADO) {
                    HAL_TIM_Base_Stop_IT(&htim2);
                    HAL_ADC_Stop(&hadc1);
                    fillRect(20, 80, 100, 15, BLACK);
                    if (*estado == PAUSA) {
                        ST7735_WriteString(40, 80, "Pausado", Font_7x10, RED, BLACK);
                    } else {
                        ST7735_WriteString(40, 80, "Trigger", Font_7x10, color(27, 57, 1), BLACK);
                        ST7735_WriteString(35, 100, "Disparado", Font_7x10, color(27, 57, 1), BLACK);
                    }
                }
                break;
            case INICIO:
                if (*estado == GUARDANDO_EN_PING) {
                    plotInicio();
                    max_buff_trigger = (OFFSET == 1) ? ((muestras == 0) ? 6 : 11) : ((muestras == 0) ? 11 : 220);
                    Swap_Buffer(0);
                    HAL_TIM_Base_Start_IT(&htim2);
                    HAL_ADC_Start(&hadc1);
                } else {
                    fillCircle(21, 75, 5, BLACK);
                    fillCircle(21, 94, 5, BLUE);
                }
                break;
            case PAUSA:
                if (*estado == GUARDANDO_EN_PING) {
                    fillRect(20, 80, 100, 15, BLACK);
                    ST7735_WriteString(20, 80, "Transmitiendo", Font_7x10, GREEN, BLACK);
                    Swap_Buffer(0);
                    HAL_TIM_Base_Start_IT(&htim2);
                    HAL_ADC_Start(&hadc1);
                } else if (*estado == INICIO) {
                    plotConfig();
                    fillCircle(21, 75, 5, BLUE);
                }
                break;
            case TRIGGER_TERMINADO:

                plotConfig();
                fillCircle(21, 75, 5, BLUE);
                break;
            case C_RESOLUCION:
                if (*estado == C_TRIGGER) {
                    fillCircle(21, 94, 5, BLACK);
                    fillCircle(21, 115, 5, BLUE);
                } else if (*estado == C_RESOLUCION_8bit) {
                    plotResolucion();
                    fillCircle(35, 74, 5, BLUE);
                } else {
                    plotResolucion();
                    fillCircle(35, 94, 5, BLUE);
                }
                break;
            case C_RESOLUCION_8bit:
                if (*estado == C_RESOLUCION_12bit) {
                    OFFSET = 2;
                    //htim2.Init.Period = 30-1;

                    TIM2->ARR = 29-1;


                    fillCircle(35, 74, 5, BLACK);
                    fillCircle(35, 94, 5, BLUE);
                } else {
                    plotConfig();
                    fillCircle(21, 94, 5, BLUE);
                }
                break;
            case C_RESOLUCION_12bit:
                if (*estado == C_RESOLUCION_8bit) {
                    OFFSET = 1;

                    TIM2->ARR = 17-1;

                    fillCircle(35, 94, 5, BLACK);
                    fillCircle(35, 74, 5, BLUE);
                } else {
                    plotConfig();
                    fillCircle(21, 94, 5, BLUE);
                }
                break;
            case C_TRIGGER:
                if (*estado == C_MUESTRAS) {
                    fillCircle(21, 115, 5, BLACK);
                    fillCircle(21, 134, 5, BLUE);
                } else {
                    ADC_TRIGGER_flag = 1;
                    HAL_ADC_Start(&hadc2);
                    plotTrigger();
                    fillCircle(23, (*estado == C_TRIGGER_ASC) ? 105 : 123, 5, BLUE);
                }
                break;
            case C_TRIGGER_ASC:
                if (*estado == C_TRIGGER) {

                	ADC_TRIGGER_flag = 0;
    				trigger_valor_09 = trigger_valor - (trigger_valor>>2);
    				trigger_valor_11 = trigger_valor + (trigger_valor>>2);

                    HAL_ADC_Stop(&hadc2);
                    plotConfig();
                    fillCircle(21, 115, 5, BLUE);
                } else if (*estado == C_TRIGGER_DESC) {
                    flanco = 1;
                    fillCircle(23, 105, 5, BLACK);
                    fillCircle(23, 123, 5, BLUE);
                }
                break;
            case C_TRIGGER_DESC:
                if (*estado == C_TRIGGER) {

                	ADC_TRIGGER_flag = 0;
    				trigger_valor_09 = trigger_valor - (trigger_valor>>2);
    				trigger_valor_11 = trigger_valor + (trigger_valor>>2);

                    HAL_ADC_Stop(&hadc2);
                    plotConfig();
                    fillCircle(21, 115, 5, BLUE);
                } else if (*estado == C_TRIGGER_ASC) {
                    flanco = 0;
                    fillCircle(23, 105, 5, BLUE);
                    fillCircle(23, 123, 5, BLACK);
                }
                break;
            case C_MUESTRAS:
                if (*estado == INICIO) {
                    fillCircle(21, 134, 5, BLACK);
                    fillCircle(21, 75, 5, BLUE);
                } else if (*estado == C_MUESTRAS_500) {
                    plotMuestras();
                    fillCircle(21, 84, 5, BLUE);
                } else {
                    plotMuestras();
                    fillCircle(21, 104, 5, BLUE);
                }
                break;
            case C_MUESTRAS_500:
                if (*estado == C_MUESTRAS) {
                    plotConfig();
                    fillCircle(21, 134, 5, BLUE);
                } else if (*estado == C_MUESTRAS_1000) {
                    muestras = 1;
                    fillCircle(21, 84, 5, BLACK);
                    fillCircle(21, 104, 5, BLUE);
                }
                break;
            case C_MUESTRAS_1000:
                if (*estado == C_MUESTRAS) {
                    plotConfig();
                    fillCircle(21, 134, 5, BLUE);
                } else if (*estado == C_MUESTRAS_500) {
                    fillCircle(21, 84, 5, BLUE);
                    fillCircle(21, 104, 5, BLACK);
                }
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
