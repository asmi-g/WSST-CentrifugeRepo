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
#include "cmsis_os.h"

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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId readSensorsHandle;
osThreadId bangBangControlHandle;
osThreadId communicationTaHandle;
osThreadId motorTaskHandle;
osThreadId messageHanldlerTaskHandle;
osMessageQId comQueueHandle;
/* USER CODE BEGIN PV */
uint32_t temp_setpoint;
enum HeaterState heater_state[HEATER_BANK_COUNT];
int active_heater_bank = HEATER_BANK_0;
uint16_t active_heater_bank_pin;

uint32_t IR_RPM_interrupt_count = 0;
float centrifuge_RPM[10];
uint32_t rpm_time = 0;
uint32_t prev_rpm_time = 0;
float global_rpm_avg;

float MS_TO_S = 1000;

uint16_t adc_values[HEATER_COUNT];
float temp_values[HEATER_COUNT];
int heater_bank_thermistor_0 = 0;
int heater_bank_thermistor_1 = 1;

static uint8_t rx_data;
static char command_queue[COMMAND_QUEUE_SIZE][COMMAND_BUFFER_SIZE];
static uint8_t command_head = 0;
static uint8_t command_tail = 0;
static uint8_t rx_buffer[COMMAND_BUFFER_SIZE];
static uint8_t rx_index = 0;

int bruh = 0;
int other_bruh = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void startReadSensors(void const * argument);
void StartBangBangControl(void const * argument);
void StartComTask(void const * argument);
void startMotorTask(void const * argument);
void StartMessageHandlerTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t is_command_queue_full()
{
    return ((command_head + 1) % COMMAND_QUEUE_SIZE) == command_tail;
}

void enqueue_command(const char *command)
{
    strncpy(command_queue[command_head], command, COMMAND_BUFFER_SIZE - 1);
    command_queue[command_head][COMMAND_BUFFER_SIZE - 1] = '\0';
    command_head = (command_head + 1) % COMMAND_QUEUE_SIZE;
}

char *dequeue_command(void)
{
    char *command = command_queue[command_tail];
    command_tail = (command_tail + 1) % COMMAND_QUEUE_SIZE;
    return command;
}

void set_setpoint(uint32_t new_setpoint)
{
	temp_setpoint = new_setpoint;
}

void reset_setpoint(void)
{
	temp_setpoint = 0;
}

float thermistor_adc_to_temp_c(uint16_t thermistor_adc_value)
{
	float thermistor_resistance_ohm = (float) THERMISTOR_RESISTOR * ((float) thermistor_adc_value / ((float) 4096 - (float) thermistor_adc_value));

	float thermistor_temp = -(30.21*logf(thermistor_resistance_ohm)) + 137.57;
	return thermistor_temp;
}

void cycle_heater_state(int active_heater)
{
  switch(heater_state[active_heater])
  {
	case OFF:
		heater_state[active_heater] = PRE_HEAT;
		set_setpoint(PRE_HEAT_SETPOINT);
		break;
	case PRE_HEAT:
		heater_state[active_heater] = FULL_HEAT;
		set_setpoint(FULL_HEAT_STOPPOINT);
		break;
	case FULL_HEAT:
		heater_state[active_heater] = OFF;
		reset_setpoint();
		break;
  }
}

void select_active_heater_bank(int active_heater_bank_in)
{
	active_heater_bank = active_heater_bank_in;
	switch(active_heater_bank_in)
	{
		case HEATER_BANK_0:
			active_heater_bank_pin = HEATER_BANK_0_Pin;
			heater_bank_thermistor_0 = 0;
			heater_bank_thermistor_1 = 1;
			break;
		case HEATER_BANK_1:
			active_heater_bank_pin = HEATER_BANK_1_Pin;
			heater_bank_thermistor_0 = 2;
			heater_bank_thermistor_1 = 3;
			break;
		case HEATER_BANK_2:
			active_heater_bank_pin = HEATER_BANK_2_Pin;
			heater_bank_thermistor_0 = 4;
			heater_bank_thermistor_1 = 5;
			break;
		case HEATER_BANK_3:
			active_heater_bank_pin = HEATER_BANK_3_Pin;
			heater_bank_thermistor_0 = 6;
			heater_bank_thermistor_1 = 7;
			break;
	}
}

void update_heater_state(int heater_bank_number, int heater_state_in)
{
	heater_state[heater_bank_number] = heater_state_in;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13)
  {
	  cycle_heater_state(active_heater_bank);
  }
  else if(GPIO_Pin == GPIO_PIN_7)
  {
//	  rpm_time = HAL_GetTick();
	  IR_RPM_interrupt_count++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (rx_data == '\n')
        {
            rx_buffer[rx_index] = '\0';
            if (!is_command_queue_full())
            {
                enqueue_command((char *)rx_buffer);
            }
            rx_index = 0;
        }
        else
        {
            if (rx_index < COMMAND_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = rx_data;
            }
        }

        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

void read_and_accumulate_adc_channels()
{
	for(int i = 0; i < HEATER_COUNT; i++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adc_values[i] += HAL_ADC_GetValue(&hadc1);
	}

	HAL_ADC_Stop(&hadc1);
}

void handle_uart_messages(char *command)
{
	  if (strcmp(command, "CMD1") == 0)
	  {
		select_active_heater_bank(HEATER_BANK_0);
//		for(int i = 0; i != active_heater_bank && i < HEATER_BANK_COUNT; i++)
//		{
//			update_heater_state(i, OFF);
//		}
		update_heater_state(active_heater_bank, PRE_HEAT);
	  }
	  else if (strcmp(command, "CMD2") == 0)
	  {
			select_active_heater_bank(HEATER_BANK_1);
//			for(int i = 0; i != active_heater_bank && i < HEATER_BANK_COUNT; i++)
//			{
//				update_heater_state(i, OFF);
//			}
			update_heater_state(active_heater_bank, PRE_HEAT);
	  }
	  else if (strcmp(command, "CMD3") == 0)
	  {
			select_active_heater_bank(HEATER_BANK_2);
//			for(int i = 0; i != active_heater_bank && i < HEATER_BANK_COUNT; i++)
//			{
//				update_heater_state(i, OFF);
//			}
			update_heater_state(active_heater_bank, PRE_HEAT);
	  }
	  else if (strcmp(command, "CMD4") == 0)
	  {
			select_active_heater_bank(HEATER_BANK_3);
//			for(int i = 0; i != active_heater_bank && i < HEATER_BANK_COUNT; i++)
//			{
//				update_heater_state(i, OFF);
//			}
			update_heater_state(active_heater_bank, PRE_HEAT);
	  }
	  else if (strcmp(command, "CMD5") == 0)
	  {
			select_active_heater_bank(HEATER_BANK_0);
//			for(int i = 0; i != active_heater_bank && i < HEATER_BANK_COUNT; i++)
//			{
//				update_heater_state(i, OFF);
//			}
			if (heater_state[active_heater_bank] == FULL_HEAT)
				update_heater_state(active_heater_bank, OFF);
			else
				update_heater_state(active_heater_bank, FULL_HEAT);
	  }
	  else if (strcmp(command, "CMD6") == 0)
	  {
			select_active_heater_bank(HEATER_BANK_1);
//			for(int i = 0; i != active_heater_bank && i < HEATER_BANK_COUNT; i++)
//			{
//				update_heater_state(i, OFF);
//			}
			if (heater_state[active_heater_bank] == FULL_HEAT)
				update_heater_state(active_heater_bank, OFF);
			else
				update_heater_state(active_heater_bank, FULL_HEAT);
	  }
	  else if (strcmp(command, "CMD7") == 0)
	  {
			select_active_heater_bank(HEATER_BANK_2);
//			for(int i = 0; i != active_heater_bank && i < HEATER_BANK_COUNT; i++)
//			{
//				update_heater_state(i, OFF);
//			}
			if (heater_state[active_heater_bank] == FULL_HEAT)
				update_heater_state(active_heater_bank, OFF);
			else
				update_heater_state(active_heater_bank, FULL_HEAT);
	  }
	  else if (strcmp(command, "CMD8") == 0)
	  {
			select_active_heater_bank(HEATER_BANK_3);
//			for(int i = 0; i != active_heater_bank && i < HEATER_BANK_COUNT; i++)
//			{
//				update_heater_state(i, OFF);
//			}
			if (heater_state[active_heater_bank] == FULL_HEAT)
				update_heater_state(active_heater_bank, OFF);
			else
				update_heater_state(active_heater_bank, FULL_HEAT);
	  }
	  // Add more command handlers as needed
}

uint8_t is_command_queue_empty()
{
    return command_head == command_tail;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of comQueue */
  osMessageQDef(comQueue, 16, char*);
  comQueueHandle = osMessageCreate(osMessageQ(comQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of readSensors */
  osThreadDef(readSensors, startReadSensors, osPriorityNormal, 0, 128);
  readSensorsHandle = osThreadCreate(osThread(readSensors), NULL);

  /* definition and creation of bangBangControl */
  osThreadDef(bangBangControl, StartBangBangControl, osPriorityRealtime, 0, 128);
  bangBangControlHandle = osThreadCreate(osThread(bangBangControl), NULL);

  /* definition and creation of communicationTa */
  osThreadDef(communicationTa, StartComTask, osPriorityNormal, 0, 1024);
  communicationTaHandle = osThreadCreate(osThread(communicationTa), NULL);

  /* definition and creation of motorTask */
  osThreadDef(motorTask, startMotorTask, osPriorityNormal, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* definition and creation of messageHanldlerTask */
  osThreadDef(messageHanldlerTask, StartMessageHandlerTask, osPriorityNormal, 0, 256);
  messageHanldlerTaskHandle = osThreadCreate(osThread(messageHanldlerTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HEATER_BANK_3_Pin|HEATER_BANK_0_Pin|HEATER_BANK_1_Pin|HEATER_BANK_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Blue_Button_Interrupt_Pin */
  GPIO_InitStruct.Pin = Blue_Button_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Blue_Button_Interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_Input_Interrupt_Pin */
  GPIO_InitStruct.Pin = IR_Input_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_Input_Interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HEATER_BANK_3_Pin HEATER_BANK_0_Pin HEATER_BANK_1_Pin HEATER_BANK_2_Pin */
  GPIO_InitStruct.Pin = HEATER_BANK_3_Pin|HEATER_BANK_0_Pin|HEATER_BANK_1_Pin|HEATER_BANK_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  reset_setpoint();
  select_active_heater_bank(active_heater_bank);
  /* Infinite loop */
  for(;;)
  {
    osDelay(10000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startReadSensors */
/**
* @brief Function implementing the readSensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startReadSensors */
void startReadSensors(void const * argument)
{
  /* USER CODE BEGIN startReadSensors */
  int oversample_count_max = 10;
  int oversample_count = 0;

  /* Infinite loop */
  // add switch case here for reading ADC channels
  for(;;)
  {
	read_and_accumulate_adc_channels();
	oversample_count++;

	if(oversample_count == oversample_count_max)
	{
		oversample_count = 0;
		for(int i = 0; i < HEATER_COUNT; i++)
		{
			adc_values[i] = adc_values[i]/oversample_count_max;
			temp_values[i] = thermistor_adc_to_temp_c(adc_values[i]);
			adc_values[i] = 0;
		}
	}
    osDelay(1);
  }
  /* USER CODE END startReadSensors */
}

/* USER CODE BEGIN Header_StartBangBangControl */
/**
* @brief Function implementing the bangBangControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBangBangControl */
void StartBangBangControl(void const * argument)
{
  /* USER CODE BEGIN StartBangBangControl */
  TIM2->CCR1 = 500; // Divide by 1000 to get PWM Duty Cycle
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // D7 on board

  /* Infinite loop */
  for(;;)
  {
	if(heater_state[active_heater_bank] == PRE_HEAT)
	{
		if(temp_values[heater_bank_thermistor_0] < PRE_HEAT_SETPOINT
			&& temp_values[heater_bank_thermistor_1] < PRE_HEAT_SETPOINT)
		{
			bruh++;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, active_heater_bank_pin, GPIO_PIN_SET);
		}
		else
		{
			other_bruh++;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, active_heater_bank_pin, GPIO_PIN_RESET);
		}
	}
	else if(heater_state[active_heater_bank] == FULL_HEAT)
	{
		if(temp_values[heater_bank_thermistor_0] < FULL_HEAT_STOPPOINT
			|| temp_values[heater_bank_thermistor_1] < FULL_HEAT_STOPPOINT)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, active_heater_bank_pin, GPIO_PIN_SET); // D12 on board
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, active_heater_bank_pin, GPIO_PIN_RESET);
			heater_state[active_heater_bank] = OFF;
		}
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, active_heater_bank_pin, GPIO_PIN_RESET);
	}
	osDelay(100);
  }
  /* USER CODE END StartBangBangControl */
}

/* USER CODE BEGIN Header_StartComTask */
/**
* @brief Function implementing the communicationTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartComTask */
void StartComTask(void const * argument)
{
  /* USER CODE BEGIN StartComTask */

  /* Infinite loop */
  for(;;)
  {
	if (!is_command_queue_empty())
	{
		char *command = dequeue_command();
		handle_uart_messages(command);
	}
	char buf[512];
	sprintf(buf, "%f, %f, %f, %f, %f, %f, %f, %f, %i, %i, %i, %i, %i, %i\n", temp_values[0], temp_values[1],
			temp_values[2], temp_values[3], temp_values[4], temp_values[5], temp_values[6],
			temp_values[7], heater_state[active_heater_bank], bruh, heater_state[0],
			heater_state[1], heater_state[2], heater_state[3]);
	HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);

    osDelay(200);
  }
  /* USER CODE END StartComTask */
}

/* USER CODE BEGIN Header_startMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMotorTask */
void startMotorTask(void const * argument)
{
  /* USER CODE BEGIN startMotorTask */
  int i = 0;
  float rpm_avg = 0;
  /* Infinite loop */
  for(;;)
  {
	rpm_time = HAL_GetTick();
	float time_delta = ((float) rpm_time - (float) prev_rpm_time)/MS_TO_S;
	centrifuge_RPM[i] = ((IR_RPM_interrupt_count/time_delta)*60)/8;
	IR_RPM_interrupt_count = 0;
	prev_rpm_time = rpm_time;

	rpm_avg = 0;

	for(int index = 0; index < 10; index++)
	{
		rpm_avg += centrifuge_RPM[index];
	}

	global_rpm_avg = (rpm_avg/10);

	i = (i+1) % 10;

    osDelay(100);
  }
  /* USER CODE END startMotorTask */
}

/* USER CODE BEGIN Header_StartMessageHandlerTask */
/**
* @brief Function implementing the messageHanldlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMessageHandlerTask */
void StartMessageHandlerTask(void const * argument)
{
  /* USER CODE BEGIN StartMessageHandlerTask */
  /* Infinite loop */
  for(;;)
  {
	osDelay(10000);
  }
  /* USER CODE END StartMessageHandlerTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  char buf[64];
  sprintf(buf, "BRUH MOMENT");

  HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
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
