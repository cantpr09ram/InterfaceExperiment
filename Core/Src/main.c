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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "keypad.h"
#include "stdio.h"
#include "stdbool.h"
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

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ADC_HandleTypeDef hadc1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//for photoresistor
//UART buffer
char str[64];
//for motor
//
// Light threshold constants
#define LDR_OPEN_THRESHOLD   100U
#define LDR_CLOSE_THRESHOLD   60U
// Servo pulse constants (250–1250 counts for 0.5–2.5 ms)
#define SERVO_STEP_DEGREE    10U
/* USER CODE END PD */

/* USER CODE BEGIN PV */
// Motor open/close state
static bool motorOpen = false;
// Current servo angle
static uint8_t servoAngle = 0;
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle){
	//angle(0-180)to pulse width(250-1250 counts)
	//250 for 0.5ms(0 degree) and 1250 for 2.5ms(180 degree)
	uint32_t pulse_length = 250 +(angle *(1250-250)/180);
	__HAL_TIM_SET_COMPARE(htim, channel, pulse_length);
}
//Timer
volatile uint32_t countdown_ms = 0;
static bool timer_started = false;

//Sound
#define SOUND_GPIO_PORT GPIOC
#define SOUND_PIN       GPIO_PIN_6

static GPIO_PinState prevSoundState = GPIO_PIN_RESET;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //keypad
  //
  //
  GPIO_TypeDef* rowPorts[ROW_NUM] = {GPIOD, GPIOD, GPIOD, GPIOD};
  uint16_t rowPins[ROW_NUM] = {row1_Pin, row2_Pin, row3_Pin, row4_Pin};

  GPIO_TypeDef* colPorts[COL_NUM] = {GPIOC, GPIOC, GPIOC, GPIOC};
  uint16_t colPins[COL_NUM] = {col1_Pin, col2_Pin, col3_Pin, col4_Pin};

  Keypad_SetRowPins(rowPorts, rowPins);
  Keypad_SetColPins(colPorts, colPins);
  Keypad_Init();

  typedef enum { //mode
        MODE_A,
        MODE_B,
		MODE_C
  } Mode;

  static Mode currentMode = MODE_A;
  static Mode previousMode = MODE_C;
  //photoresistor
  //
  //
  HAL_ADC_Start(&hadc1);
  uint32_t adc_value=0;
  float voltage=0;
  //motor
  //
  //
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  //timer
  //
  //


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  char key = Keypad_Scan();
	      if (key == 'A')      currentMode = MODE_A;
	      else if (key == 'B') currentMode = MODE_B;
	      else if (key == 'C') currentMode = MODE_C;
          char state = "close";

          if (currentMode != previousMode) {
        	  switch (currentMode) {
              	  case MODE_A:
              		  HAL_UART_Transmit(&huart3, (uint8_t*)"Enter Mode A\r\n", 13, 100);
                      break;
              	  case MODE_B:
              		  HAL_UART_Transmit(&huart3, (uint8_t*)"Enter Mode B\r\n", 13, 100);
              		  break;
                  case MODE_C:
                      HAL_UART_Transmit(&huart3, (uint8_t*)"Enter Mode C\r\n", 13, 100);
                      break;
                  }
                  previousMode = currentMode;
              }

	      switch (currentMode) {
			  case MODE_A: {
				  // Start ADC in continuous conversion mode once
				  HAL_ADC_Start(&hadc1);

				  // Stay in Mode A until changed
				  while (currentMode == MODE_A) {
					  // 1) Poll ADC conversion
					  // Start ADC in continuous conversion mode once
						 HAL_ADC_Start(&hadc1);
					  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
						  uint32_t raw = HAL_ADC_GetValue(&hadc1);

						  float mv = raw * 3300.0f / 4095.0f;

						  // Transmit raw LDR value and voltage for testing
						  sprintf(str, "LDR raw=%4lu, %6.1f mV\r\n", raw, mv);
						  HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);
						  // 2) Threshold check: open if above open threshold
						  if (!motorOpen && raw >= LDR_OPEN_THRESHOLD) {
							  motorOpen = true;
						  }
						  //    close if below close threshold
						  else if (motorOpen && raw <= LDR_CLOSE_THRESHOLD) {
							  motorOpen = false;
						  }

						  // 3) Move servo one step toward target
						  if (motorOpen && servoAngle < 180) {
							  servoAngle += SERVO_STEP_DEGREE;
						  } else if (!motorOpen && servoAngle > 0) {
							  servoAngle -= SERVO_STEP_DEGREE;
						  }
						  Set_Servo_Angle(&htim2, TIM_CHANNEL_1, servoAngle);

						  // 4) UART feedback
						  sprintf(str,"LDR=%3lu, Motor=%s, Angle=%3u\r\n", raw, motorOpen ? "OPEN" : "CLOSE", servoAngle);
						  HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);
						  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
					  }

					  // 5) Check for exit key
					  char exitKey = Keypad_Scan();
					  if (exitKey == 'B') {
						  currentMode = MODE_B;
					  } else if (exitKey == 'C') {
						  currentMode = MODE_C;
					  }

					  HAL_Delay(500);
				  }
				  break;
			  } // end case MODE_A

			  case MODE_B: {
			      // use the 'key' already read at top of while()
			      if (key >= '0' && key <= '9') {
			          // 1) convert input to seconds
			          uint32_t seconds = key - '0';
			          countdown_ms = seconds * 1000U;
			          timer_started = true;

			          // 2) UART feedback
			          sprintf(str, "Start countdown: %lu s\r\n", seconds);
			          HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);

			          // 3) reset and start TIM4 interrupt
			          __HAL_TIM_SET_COUNTER(&htim4, 0);
			          HAL_TIM_Base_Start_IT(&htim4);
			      }

			      // 4) non-blocking check: countdown finished?
			      if (timer_started && countdown_ms == 0) {
			          timer_started = false;

			          // toggle motor here (move servo)
			          if (motorOpen) {
			              // close motor: sweep 180→0
			              for (uint8_t a = 180; a > 0; a -= SERVO_STEP_DEGREE) {
			                  Set_Servo_Angle(&htim2, TIM_CHANNEL_1, a);
			                  HAL_Delay(50);
			              }
			              motorOpen = false;
			              sprintf(str, "Motor CLOSED\r\n");
			          } else {
			              // open motor: sweep 0→180
			              for (uint8_t a = 0; a <= 180; a += SERVO_STEP_DEGREE) {
			                  Set_Servo_Angle(&htim2, TIM_CHANNEL_1, a);
			                  HAL_Delay(50);
			              }
			              motorOpen = true;
			              sprintf(str, "Motor OPENED\r\n");
			          }
			          HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);
			      }
			      break;
			  }
			  case MODE_C: {
			      // 1) Read digital output from sound sensor
			      GPIO_PinState curSound = HAL_GPIO_ReadPin(SOUND_GPIO_PORT, SOUND_PIN);

			      // --- DEBUG: print current & previous sound states ---
			      //  curSound==1 means HIGH (sound detected), 0 means LOW
			      sprintf(str, "Sound State: prev=%d, cur=%d\r\n", prevSoundState == GPIO_PIN_SET, curSound == GPIO_PIN_SET);
			      HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);

			      // 2) Detect rising edge: LOW -> HIGH
			      if (curSound == GPIO_PIN_SET && prevSoundState == GPIO_PIN_RESET) {
			          // 3) Toggle motor
			          if (motorOpen) {
			              // Close servo: sweep 180 → 0
			              for (uint8_t a = 180; a > 0; a -= SERVO_STEP_DEGREE) {
			                  Set_Servo_Angle(&htim2, TIM_CHANNEL_1, a);
			                  HAL_Delay(50);
			              }
			              motorOpen = false;
			              sprintf(str, "Mode C: Motor CLOSED\r\n");
			          } else {
			              // Open servo: sweep 0 → 180
			              for (uint8_t a = 0; a <= 180; a += SERVO_STEP_DEGREE) {
			                  Set_Servo_Angle(&htim2, TIM_CHANNEL_1, a);
			                  HAL_Delay(50);
			              }
			              motorOpen = true;
			              sprintf(str, "Mode C: Motor OPENED\r\n");
			          }
			          HAL_UART_Transmit(&huart3,
			                            (uint8_t*)str,
			                            strlen(str),
			                            100);
			      }

			      // 4) Update previous state for next loop
			      prevSoundState = curSound;
			      break;
			  }
	          default:
	               break;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  sConfig.Channel = ADC_CHANNEL_10;
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
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, row1_Pin|row2_Pin|row3_Pin|row4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 col1_Pin col2_Pin col3_Pin
                           col4_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|col1_Pin|col2_Pin|col3_Pin
                          |col4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : row1_Pin row2_Pin row3_Pin row4_Pin */
  GPIO_InitStruct.Pin = row1_Pin|row2_Pin|row3_Pin|row4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// This callback runs every 1 ms when TIM4 update interrupt fires
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4 && countdown_ms > 0) {
        countdown_ms--;
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
