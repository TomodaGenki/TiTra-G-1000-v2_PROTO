/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "conf.h"
#include "tbt.h"
#include "lift.h"
#include "turn.h"
#include "nuc.h"
#include "Cntrl_Proc.h"
#include "battery.h"
#include "charge.h"
#include "wheel.h"
#include "task_charger_link.h"
#include "sound.h"
#include "lidar.h"
#include "syncturn.h"
#include "led.h"
#include "wheel_test.h"
#include "load.h"
#include "can.h"

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

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim9;

IRDA_HandleTypeDef hirda2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
static uint32_t tim5_cnt = 0;
static uint16_t	tim6_cnt = 0;
static uint16_t	tim7_cnt = 0;
static uint16_t	r_wheel_enc = 0;
static uint16_t	l_wheel_enc = 0;
static uint16_t turn_encoder = 0;
const uint8_t f_version[ver_length] = {DEST_HIGH,DEST_LOW, MAJOR_HIGH,MAJOR_LOW, MINOR_HIGH,MINOR_LOW, BATCH_HIGH,BATCH_LOW};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART2_IRDA_Init(void);
static void MX_I2C2_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void enc_pulse_tim_start(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t get_fw_version(uint8_t num) {

	uint8_t ret = 0;

	if (num < ver_length) {
		ret = f_version[num];
	}
	return ret;
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
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_USART2_IRDA_Init();
  MX_I2C2_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  nuc_init();
  chagerlink_init();
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

  load_init();
  turn_init();
  charge_proc_init();
  lift_init();
  init_timer();											// initiate timer
  read_HardwareID();									// read microcomputer board ID
  init_lidar_wait_time();
  enc_pulse_tim_start();
  battery_init();
  can_filter_setting();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
#if BATTERY_TYPE == MURATA_BATTERY
  CAN_filter_setting();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (check_timer(CNT_GO4) == TIME_UP)
	  {
		  battery_monitor();
		  set_utimer(CNT_GO4, CNT_PERIOD4);				// 20sec
	  }

	  if (check_timer(CNT_GO3) == TIME_UP)
	  {
		  transmit_charge_cmd();
		  set_utimer(CNT_GO3, CNT_PERIOD3);				// 300msec
	  }

	  if (check_timer(CNT_GO2) == TIME_UP)
	  {
		  ck_com_error();
		  sound_cntrol();
		  cmnd_proc();
		  set_utimer(CNT_GO2, CNT_PERIOD2);				// 100msec
	  }

	  if (check_timer(CNT_GO1) == TIME_UP)
	  {
		  scan_proc1();
		  load_measure();
		  set_utimer(CNT_GO1, CNT_PERIOD1);				// 20msec
	  }

	  if (check_timer(CNT_WHEEL_ENCODER) == TIME_UP) {	// エンコーダー取得のための関数
		  set_utimer(CNT_WHEEL_ENCODER, CNT_PROCPERIOD);
	  }

	  if (check_timer(CNT_PROCESS) == TIME_UP)
	  {
		  if (!Is_SyncTurnning()) {
			  lift_cntrl();
			  charge_proc();	// Charge Sequence
			  wheel_log_update();
		  }
		  led_proc();
		  set_utimer(CNT_PROCESS, CNT_PROCPERIOD);		// 10msec
	  }

	  if (check_timer(CNT_GO7) == TIME_UP)
	  {
		  check_nuc_receive_data();
		  int16_t left_speed = get_LeftWheel_Speed();
		  int16_t right_speed = get_RightWheel_Speed();
		  wheel_cntrl(left_speed, right_speed);
		  //ck_motor_receive_data();
		  set_utimer(CNT_GO7, CNT_1MSEC);				// 1msec
	  }

	  if (get_i2c2_init() != 0) {
		  MX_I2C2_Init();
		  reset_i2c2_init();
	  }

	  if (check_timer(CNT_TURN) == TIME_UP)
	  {
		  if (Is_SyncTurnning() == 0) {
			  turn_cntrl();
		  }
		  set_utimer(CNT_TURN, CNT_TURNPERIOD);		// 1msec
	  }

	  if (check_timer(CNT_SYNCTURN) == TIME_UP) {
		  syncturn_ctrl();
		  set_utimer(CNT_SYNCTURN, CNT_SYNCTURN_VAL);
	  }

	  // log output(for development)
	  if((get_sync_dump_req() == 1) && (get_sync_dump_comp() != 1)){
		  syncturn_log_dump();
	  }
	  else if((get_turn_dump_req() == 1) && (get_turn_dump_comp() != 1)){
		  turn_log_dump();
	  }
	  else if((get_wheel_dump_req() == 1) && (get_wheel_dump_comp() != 1)){
		  wheel_log_dump();
	  }
	  else if((get_wheel_test_dump_req() == 1) && (get_wheel_test_dump_comp() != 1)){
		  wheel_test_log_dump();
	  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim4.Init.Prescaler = 32;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 2000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 168-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 0;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_IRDA_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  hirda2.Instance = USART2;
  hirda2.Init.BaudRate = 9600;
  hirda2.Init.WordLength = IRDA_WORDLENGTH_8B;
  hirda2.Init.Parity = IRDA_PARITY_NONE;
  hirda2.Init.Mode = IRDA_MODE_TX_RX;
  hirda2.Init.Prescaler = 1;
  hirda2.Init.IrDAMode = IRDA_POWERMODE_NORMAL;
  if (HAL_IRDA_Init(&hirda2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_9B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_EVEN;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, O_OptIn4_Pin|O_OptIn3_Pin|O_OptIn2_Pin|O_OptIn1_Pin 
                          |O_OptSel_Pin|O_OptMode_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, O_BlueLed_R_Pin|O_USART6_TR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, O_Wheel1Reset_Pin|O_WheelDCLock_Pin|O_Wheel2CW_Pin|O_Wheel2CCW_Pin 
                          |O_Wheel2Reset_Pin|O_Wheel1Load_Pin|O_BlueLed_L_Pin|O_RedLed_L_Pin 
                          |O_GreenLed_L_Pin|O_Lift1AlarmReset_Pin|O_LiftStopMode_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, O_AlarmCh1_Pin|O_Wheel_STO_Pin|O_AlarmCh3_Pin|O_AlarmCh4_Pin 
                          |O_Lidar1In4_Pin|O_Lidar1In5_Pin|O_Lidar1In3_Pin|O_Lidar1In2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, O_TurnStopMode_Pin|O_GreenLed_R_Pin|O_RedLed_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, O_TableLed_Pin|O_LiftBrake_Pin|O_Lift2AlarmReset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, O_Lift1CCW_Pin|O_Lift1CW_Pin|O_BrakeRelay_Pin|O_LiftRelay_Pin 
                          |O_TurnRelay_Pin|O_DriveRelay_Pin|O_ChargeRelay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(O_AddRelay_GPIO_Port, O_AddRelay_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, O_TurnAlarmReset_Pin|O_TurnBrake_Pin|O_TurnCCW_Pin|O_TurnCW_Pin 
                          |O_Lidar1In1_Pin|O_Lidar2In5_Pin|O_Lidar2In4_Pin|O_Lidar2In3_Pin 
                          |O_Lidar2In2_Pin|O_Lidar2In1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, O_Wheel1CCW_Pin|O_Wheel1CW_Pin|O_Lift2CCW_Pin|O_Lift2CW_Pin 
                          |O_Wheel2Load_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : O_OptIn4_Pin O_OptIn3_Pin O_OptIn2_Pin O_OptIn1_Pin 
                           O_OptSel_Pin O_OptMode_Pin */
  GPIO_InitStruct.Pin = O_OptIn4_Pin|O_OptIn3_Pin|O_OptIn2_Pin|O_OptIn1_Pin 
                          |O_OptSel_Pin|O_OptMode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : O_BlueLed_R_Pin O_USART6_TR_Pin */
  GPIO_InitStruct.Pin = O_BlueLed_R_Pin|O_USART6_TR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_Wheel1Pulse_Pin INT_Wheel2Pulse_Pin */
  GPIO_InitStruct.Pin = INT_Wheel1Pulse_Pin|INT_Wheel2Pulse_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : I_Wheel1Dir_Pin I_BrakeReleSw_F_Pin I_ResetSw_F_Pin I_Bumber2_Pin 
                           I_Lift2Alm_Pin I_TurnEncoderBch_Pin */
  GPIO_InitStruct.Pin = I_Wheel1Dir_Pin|I_BrakeReleSw_F_Pin|I_ResetSw_F_Pin|I_Bumber2_Pin 
                          |I_Lift2Alm_Pin|I_TurnEncoderBch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : O_Wheel1Reset_Pin O_WheelDCLock_Pin O_Wheel2CW_Pin O_Wheel2CCW_Pin 
                           O_Wheel2Reset_Pin O_Wheel1Load_Pin O_BlueLed_L_Pin O_RedLed_L_Pin 
                           O_GreenLed_L_Pin */
  GPIO_InitStruct.Pin = O_Wheel1Reset_Pin|O_WheelDCLock_Pin|O_Wheel2CW_Pin|O_Wheel2CCW_Pin 
                          |O_Wheel2Reset_Pin|O_Wheel1Load_Pin|O_BlueLed_L_Pin|O_RedLed_L_Pin 
                          |O_GreenLed_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : I_TurnBrakeSw_F_Pin I_Wheel1Error_Pin I_Wheel2Dir_Pin */
  GPIO_InitStruct.Pin = I_TurnBrakeSw_F_Pin|I_Wheel1Error_Pin|I_Wheel2Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : I_LiftSw_F_Pin I_TurnBrakeSw_R_Pin I_Bumper1_Pin */
  GPIO_InitStruct.Pin = I_LiftSw_F_Pin|I_TurnBrakeSw_R_Pin|I_Bumper1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : O_AlarmCh1_Pin O_Wheel_STO_Pin O_AlarmCh3_Pin O_AlarmCh4_Pin 
                           O_AddRelay_Pin O_Lidar1In4_Pin O_Lidar1In5_Pin O_Lidar1In3_Pin 
                           O_Lidar1In2_Pin */
  GPIO_InitStruct.Pin = O_AlarmCh1_Pin|O_Wheel_STO_Pin|O_AlarmCh3_Pin|O_AlarmCh4_Pin 
                          |O_AddRelay_Pin|O_Lidar1In4_Pin|O_Lidar1In5_Pin|O_Lidar1In3_Pin 
                          |O_Lidar1In2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : O_TurnStopMode_Pin */
  GPIO_InitStruct.Pin = O_TurnStopMode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(O_TurnStopMode_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : O_TableLed_Pin */
  GPIO_InitStruct.Pin = O_TableLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(O_TableLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : O_LiftBrake_Pin O_Lift2AlarmReset_Pin */
  GPIO_InitStruct.Pin = O_LiftBrake_Pin|O_Lift2AlarmReset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : O_Lift1AlarmReset_Pin O_LiftStopMode_Pin */
  GPIO_InitStruct.Pin = O_Lift1AlarmReset_Pin|O_LiftStopMode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : O_Lift1CCW_Pin O_Lift1CW_Pin */
  GPIO_InitStruct.Pin = O_Lift1CCW_Pin|O_Lift1CW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : I_BrakeReleSw_R_Pin */
  GPIO_InitStruct.Pin = I_BrakeReleSw_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I_BrakeReleSw_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I_Turn0degSens_Pin I_Turn90degSens_Pin I_Turn180degSens_Pin I_Turn270degSens_Pin 
                           I_Lift2UpLimitSens_Pin I_Lift2UpSens_Pin I_Lift2DownSens_Pin I_Lift2DownLimitSens_Pin */
  GPIO_InitStruct.Pin = I_Turn0degSens_Pin|I_Turn90degSens_Pin|I_Turn180degSens_Pin|I_Turn270degSens_Pin 
                          |I_Lift2UpLimitSens_Pin|I_Lift2UpSens_Pin|I_Lift2DownSens_Pin|I_Lift2DownLimitSens_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : I_Lift1UpLimitSens_Pin I_Lift1UpSens_Pin I_Lift1DownSens_Pin I_Lift1DownLimitSens_Pin */
  GPIO_InitStruct.Pin = I_Lift1UpLimitSens_Pin|I_Lift1UpSens_Pin|I_Lift1DownSens_Pin|I_Lift1DownLimitSens_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : I_Lift1Alm_Pin I_EmergencyMoni_Pin I_LiftSw_R_Pin I_ResetSw_R_Pin 
                           I_OptGo_Pin I_OptOut4_Pin I_OptOut3_Pin I_OptOut2_Pin 
                           I_OptOut1_Pin */
  GPIO_InitStruct.Pin = I_Lift1Alm_Pin|I_EmergencyMoni_Pin|I_LiftSw_R_Pin|I_ResetSw_R_Pin 
                          |I_OptGo_Pin|I_OptOut4_Pin|I_OptOut3_Pin|I_OptOut2_Pin 
                          |I_OptOut1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_TurnEncorder_Pin */
  GPIO_InitStruct.Pin = INT_TurnEncorder_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_TurnEncorder_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I_Wheel2Error_Pin I_TurnAlm_Pin */
  GPIO_InitStruct.Pin = I_Wheel2Error_Pin|I_TurnAlm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : O_TurnAlarmReset_Pin O_TurnBrake_Pin O_TurnCCW_Pin O_TurnCW_Pin */
  GPIO_InitStruct.Pin = O_TurnAlarmReset_Pin|O_TurnBrake_Pin|O_TurnCCW_Pin|O_TurnCW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : O_BrakeRelay_Pin O_LiftRelay_Pin O_TurnRelay_Pin O_DriveRelay_Pin 
                           O_ChargeRelay_Pin */
  GPIO_InitStruct.Pin = O_BrakeRelay_Pin|O_LiftRelay_Pin|O_TurnRelay_Pin|O_DriveRelay_Pin 
                          |O_ChargeRelay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : I_Lift1Tim_Pin I_Lift2Tim_Pin */
  GPIO_InitStruct.Pin = I_Lift1Tim_Pin|I_Lift2Tim_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : O_GreenLed_R_Pin O_RedLed_R_Pin */
  GPIO_InitStruct.Pin = O_GreenLed_R_Pin|O_RedLed_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : O_Wheel1CCW_Pin O_Wheel1CW_Pin O_Wheel2Load_Pin */
  GPIO_InitStruct.Pin = O_Wheel1CCW_Pin|O_Wheel1CW_Pin|O_Wheel2Load_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : O_Lift2CCW_Pin O_Lift2CW_Pin */
  GPIO_InitStruct.Pin = O_Lift2CCW_Pin|O_Lift2CW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I_Lidar1Trouble_Pin */
  GPIO_InitStruct.Pin = I_Lidar1Trouble_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I_Lidar1Trouble_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I_Lidar1Out3_Pin I_Lidar2Out2_Pin */
  GPIO_InitStruct.Pin = I_Lidar1Out3_Pin|I_Lidar2Out2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : O_Lidar1In1_Pin O_Lidar2In5_Pin O_Lidar2In4_Pin O_Lidar2In3_Pin 
                           O_Lidar2In2_Pin O_Lidar2In1_Pin */
  GPIO_InitStruct.Pin = O_Lidar1In1_Pin|O_Lidar2In5_Pin|O_Lidar2In4_Pin|O_Lidar2In3_Pin 
                          |O_Lidar2In2_Pin|O_Lidar2In1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I_Lidar1Out2_Pin I_Lidar1Out1_Pin I_Lidar2Trouble_Pin I_Lidar2Out3_Pin 
                           I_Lidar2Out1_Pin */
  GPIO_InitStruct.Pin = I_Lidar1Out2_Pin|I_Lidar1Out1_Pin|I_Lidar2Trouble_Pin|I_Lidar2Out3_Pin 
                          |I_Lidar2Out1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void enc_pulse_tim_start(void){
	// TIM6 start (Wheel1 エンコーダパルス間隔計測用)
	TIM6 ->CR1 |= TIM_CR1_UDIS;
	TIM6 ->DIER &= ~TIM_DIER_UDE;
	TIM6 ->DIER &= ~TIM_DIER_UIE;
	TIM6 ->CR1 |= TIM_CR1_CEN;
	// TIM7 start (Wheel2 エンコーダパルス間隔計測用)
	TIM7 ->CR1 |= TIM_CR1_UDIS;
	TIM7 ->DIER &= ~TIM_DIER_UDE;
	TIM7 ->DIER &= ~TIM_DIER_UIE;
	TIM7 ->CR1 |= TIM_CR1_CEN;
	// TIM5 start (ターンテーブル エンコーダパルス間隔計測用)
	TIM5 ->CR1 |= TIM_CR1_UDIS;
	TIM5 ->DIER &= ~TIM_DIER_UDE;
	TIM5 ->DIER &= ~TIM_DIER_UIE;
	TIM5 ->CR1 |= TIM_CR1_CEN;
}

void enc_pulse_tim_cnt_reset(void){
	// タイマのオーバーフロー時に処理速度低下するためオーバーフローを回避する
	if ((TIM6 -> CNT) > 40000){
		TIM6 -> CNT = 0;	// wheel1エンコーダパルス周期計測用(シンクロターン用)
	}
	if ((TIM7 -> CNT) > 40000){
		TIM7 -> CNT = 0;	// wheel2エンコーダパルス周期計測用(シンクロターン用)
	}
	if ((TIM5 -> CNT) > 2500000000){
		TIM5 -> CNT = 0;	// ターンテーブルエンコーダパルス周期計測用(シンクロターン用)
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // direction & pulse count
#if WHEEL_ENC_FILTER == FILTER_DISABLE
	if (GPIO_Pin == INT_Wheel1Pulse_Pin) {		// Wheel1 encoder PC13
		// パルスの更新周期を計測
		tim6_cnt = TIM6 -> CNT;
		TIM6 -> CNT = 0;
	    if (HAL_GPIO_ReadPin(I_Wheel1Dir_GPIO_Port, I_Wheel1Dir_Pin) == GPIO_PIN_SET) {
			r_wheel_enc++;
	    } else {
			r_wheel_enc--;
	    }
	} else if (GPIO_Pin == INT_Wheel2Pulse_Pin) {	// Wheel2 encoder PC15
		// パルスの更新周期を計測
		tim7_cnt = TIM7 -> CNT;
		TIM7 -> CNT = 0;
		if (HAL_GPIO_ReadPin(I_Wheel2Dir_GPIO_Port, I_Wheel2Dir_Pin) == GPIO_PIN_SET) {
			l_wheel_enc--;
		} else {
			l_wheel_enc++;
		}
#else
	if ((GPIO_Pin == INT_Wheel1Pulse_Pin) && ((TIM6 -> CNT) > 20)) {		// Wheel1 encoder PC13
		// ノイズ対策： 2.1kHz(1.7m/s相当)以上のパルスはカウントしない
		tim6_cnt = TIM6 -> CNT;
		TIM6 -> CNT = 0;
	    if (HAL_GPIO_ReadPin(I_Wheel1Dir_GPIO_Port, I_Wheel1Dir_Pin) == GPIO_PIN_SET) {
			r_wheel_enc++;
	    } else {
			r_wheel_enc--;
	    }
	} else if (GPIO_Pin == INT_Wheel2Pulse_Pin && ((TIM7 -> CNT) > 20)) {	// Wheel2 encoder PC15
		// ノイズ対策： 2.1kHz(1.7m/s相当)以上のパルスはカウントしない
		tim7_cnt = TIM7 -> CNT;
		TIM7 -> CNT = 0;
		if (HAL_GPIO_ReadPin(I_Wheel2Dir_GPIO_Port, I_Wheel2Dir_Pin) == GPIO_PIN_SET) {
			l_wheel_enc--;
		} else {
			l_wheel_enc++;
		}
#endif
	} else if (GPIO_Pin == I_Lift1Tim_Pin) {	// PG7 Lift1 encoder
		add_lift1_enc();
	} else if (GPIO_Pin == I_Lift2Tim_Pin) {	// PG8 Lift2 encoder
		add_lift2_enc();
	} else if (GPIO_Pin == INT_TurnEncorder_Pin) {	//PD9 Turn encoder
		// パルスの更新周期を計測
		tim5_cnt = TIM5 -> CNT;
		TIM5 -> CNT = 0;
		if (HAL_GPIO_ReadPin(I_TurnEncoderBch_GPIO_Port, I_TurnEncoderBch_Pin) == GPIO_PIN_RESET){			// phase B fall down
			turn_encoder--;
		} else {
			turn_encoder++;
		}
	}
}

uint16_t get_l_wheel_enc_cnt(void) {
	return tim7_cnt;
}

uint16_t get_r_wheel_enc_cnt(void) {
	return tim6_cnt;
}

uint16_t get_turn_encoder(void) {
	return turn_encoder;
}

uint32_t get_turn_enc_cnt(void) {
	return tim5_cnt;
}

void int_1msCallback(void){
    // ----timer count up-----
	time_run();

	tbt();
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
