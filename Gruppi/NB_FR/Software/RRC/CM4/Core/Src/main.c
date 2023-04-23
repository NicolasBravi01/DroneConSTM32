/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "ReceiverChannel.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define TIMCLOCK   240000000
#define PRESCALAR  240
#define refClock  TIMCLOCK/(PRESCALAR)


#define MAX_ANGLE 30
#define MAX_THROTTLE 100
#define MAX_YAW_SPEED 10

#define MAX_THROTTLE_OFF 15





#define MIN_VALUE_MOTOR 1000	//%5
#define MAX_VALUE_MOTOR 2000	//%10


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void startInputCaptureInterruptDutyCycle();
void startPwmMotors();

void calculateRadioControlValues();
float waitingForGettingFrequency();
void jumpHalfPeriod(float frequency);
float map(float val, float from_src, float to_src, float from_dst, float to_dst);
int getLevel(float dutyLevel);
void waitingForFirstValues(ChannelForDuty_Data values[], int ne);


void armMotors();
void stopMotors();
void moveMotors();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

ChannelForDuty_Data chDuty[NUMBER_CHANNELS];
ChannelForFrequency_Data chFrequency;

float desiredPitch, desiredRoll, desiredYaw;
float throttle;
int level;





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/* Activate HSEM notification for Cortex-M4*/
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
	/*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
	 */
	HAL_PWREx_ClearPendingEvent();
	HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
	/* Clear HSEM flag */
	__HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();


  /* USER CODE BEGIN 2 */

  Receiver_Init(&chFrequency, chDuty, NUMBER_CHANNELS);

  waitingForGettingFrequency();
  jumpHalfPeriod(chFrequency.frequency);
  startInputCaptureInterruptDutyCycle();

  //si attende di ottenere i primi valori dal radiocomando, prima di entrare nel loop
  waitingForFirstValues(chDuty, NUMBER_CHANNELS);

  startPwmMotors();


  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

		calculateRadioControlValues();

		switch(level)
		{
			//arms
			case 1:
				armMotors();
				break;

			//move
			case 2:
				if(throttle == 0)
					stopMotors();
				else
					moveMotors(throttle);
				break;

			//stop
			case 3:
				stopMotors();
				break;
		}




	}
  /* USER CODE END 3 */
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
void MX_ETH_Init(void)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 240;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 240-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 240;
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
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void startInputCaptureInterruptDutyCycle()
{
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);	//ch1
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); //ch2
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3); //ch3
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4); //ch4
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4); //ch5
}

void startPwmMotors()
{
 	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	//motore

 	//RAGAZZI PID
 	/*
 	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
 	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
 	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
 	*/
}



void calculateRadioControlValues()
{
	desiredPitch = map(chDuty[IC_CHANNEL1].duty, MIN_RANGE_DUTY, MAX_RANGE_DUTY, -MAX_ANGLE, MAX_ANGLE);
	desiredRoll = map(chDuty[IC_CHANNEL2].duty, MIN_RANGE_DUTY, MAX_RANGE_DUTY, -MAX_ANGLE, MAX_ANGLE) ;
	throttle = map(chDuty[IC_CHANNEL3].duty, MIN_RANGE_DUTY, MAX_RANGE_DUTY, 0, MAX_THROTTLE);
	desiredYaw = map(chDuty[IC_CHANNEL4].duty, MIN_RANGE_DUTY, MAX_RANGE_DUTY, -MAX_YAW_SPEED, MAX_YAW_SPEED);
	level = getLevel(chDuty[IC_CHANNEL5].duty);


	desiredPitch = correctAngle(desiredPitch, MAX_ERROR_STABILIZATION);
	desiredRoll = correctAngle(desiredRoll, MAX_ERROR_STABILIZATION);
	desiredYaw = correctAngle(desiredYaw, MAX_ERROR_YAW);
	throttle = correctThrottle(throttle, MAX_THROTTLE_OFF);
}




int getLevel(float dutyLevel)
{
	int level;

	if(dutyLevel > LEVEL1_DUTY - error_level && dutyLevel < LEVEL1_DUTY + error_level)
		level = 1;
	else if(dutyLevel > LEVEL2_DUTY - error_level && dutyLevel < LEVEL2_DUTY + error_level)
		level = 2;
	if(dutyLevel > LEVEL3_DUTY - error_level && dutyLevel < LEVEL3_DUTY + error_level)
		level = 3;

	return level;
}



float map(float val, float from_src, float to_src, float from_dst, float to_dst)
{
	return (((to_dst-from_dst)/(to_src-from_src))*(val-from_src)) + from_dst;
}



void jumpHalfPeriod (float frequency)
{
	HAL_Delay(1000/(2*frequency));
}


float waitingForGettingFrequency()
{
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

	while (chFrequency.frequency == 0 || chFrequency.frequency > 100 || chFrequency.flagFirstFrequency == 1) { }

	HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);

	return chFrequency.frequency;
}



void waitingForFirstValues(ChannelForDuty_Data values[], int ne)
{
	int i;
	do
	{
		for(i=0; i<ne; i++)
		{
			if(values[i].duty == 0)
				break;
		}
	}while(i != ne);
}




void armMotors()
{
	TIM3->CCR1 = 900;
	TIM3->CCR2 = 900;
	TIM3->CCR3 = 900;
	TIM3->CCR4 = 900;
}

//RAGAZZI PID
//funziona per il brushless sul banco di prova
void moveMotors()
{
	TIM3->CCR1 = MIN_VALUE_MOTOR + (throttle/100)*(MAX_VALUE_MOTOR - MIN_VALUE_MOTOR);
}



void stopMotors()
{
	TIM3->CCR1 = 900;
	TIM3->CCR2 = 900;
	TIM3->CCR3 = 900;
	TIM3->CCR4 = 900;
}








void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &htim5)
	{

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
		{
			if (chFrequency.firstCaptured == 0) // if the first value is not captured
			{
				chFrequency.firstCaptured = 1;  // set the first captured as true
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			}

			else   // if the first is already captured
			{
				chFrequency.val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read value

				chFrequency.frequency = (float)refClock/chFrequency.val;
				chFrequency.frequency = floorf(chFrequency.frequency * 100) / 100;

				chFrequency.firstCaptured = 0; // set it back to false
				chFrequency.flagFirstFrequency = 0;
			}
		}


		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) // if the interrupt source is channel4
		{
			if (chDuty[IC_CHANNEL5].firstCaptured == 0) // if the first value is not captured
			{
				chDuty[IC_CHANNEL5].firstCaptured = 1; // set the first captured as true
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			}

			else   // if the first is already captured
			{
				chDuty[IC_CHANNEL5].val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read value

				chDuty[IC_CHANNEL5].usWidth = (float) (chDuty[IC_CHANNEL5].val) / refClock;
				chDuty[IC_CHANNEL5].duty = chDuty[IC_CHANNEL5].usWidth * chFrequency.frequency * 100;
				chDuty[IC_CHANNEL5].duty = floorf(chDuty[IC_CHANNEL5].duty * 100) / 100;

				chDuty[IC_CHANNEL5].firstCaptured = 0; // set it back to false
			}
		}


	}
	else if(htim == &htim2)
	{

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 )  // if the interrupt source is channel1
		{

			if (chDuty[IC_CHANNEL1].firstCaptured == 0) // if the first value is not captured
			{
				chDuty[IC_CHANNEL1].firstCaptured = 1;  // set the first captured as true
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			}

			else   // if the first is already captured
			{
				chDuty[IC_CHANNEL1].val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read value

				chDuty[IC_CHANNEL1].usWidth = (float)(chDuty[IC_CHANNEL1].val) / refClock;
				chDuty[IC_CHANNEL1].duty = (chDuty[IC_CHANNEL1].usWidth * chFrequency.frequency * 100);
				chDuty[IC_CHANNEL1].duty = floorf(chDuty[IC_CHANNEL1].duty * 100) / 100;

				chDuty[IC_CHANNEL1].firstCaptured = 0; // set it back to false
			}
		}


		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 )  // if the interrupt source is channel2
		{
			chDuty[IC_CHANNEL2].val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read value

			chDuty[IC_CHANNEL2].usWidth = (float)(chDuty[IC_CHANNEL2].val) / refClock;
			chDuty[IC_CHANNEL2].duty = chDuty[IC_CHANNEL2].usWidth * chFrequency.frequency * 100 + ROLL_ERROR_CALIBRATION;
			chDuty[IC_CHANNEL2].duty = floorf(chDuty[IC_CHANNEL2].duty * 100) / 100;
		}


		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3 )  // if the interrupt source is channel3
		{
			chDuty[IC_CHANNEL3].val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read value

			chDuty[IC_CHANNEL3].usWidth = (float)(chDuty[IC_CHANNEL3].val) / refClock;
			chDuty[IC_CHANNEL3].duty = chDuty[IC_CHANNEL3].usWidth * chFrequency.frequency * 100;
			chDuty[IC_CHANNEL3].duty = floorf(chDuty[IC_CHANNEL3].duty * 100) / 100;
		}


		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )  // if the interrupt source is channel4
		{
			chDuty[IC_CHANNEL4].val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);	// read value

			chDuty[IC_CHANNEL4].usWidth = (float)(chDuty[IC_CHANNEL4].val) / refClock;
			chDuty[IC_CHANNEL4].duty = chDuty[IC_CHANNEL4].usWidth * chFrequency.frequency * 100;
			chDuty[IC_CHANNEL4].duty = floorf(chDuty[IC_CHANNEL4].duty * 100) / 100;
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
