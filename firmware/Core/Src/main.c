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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLTAGE_SENSE_DIVIDER 11.0
#define CURRENT_SENSE_AMP 50.0

#define EXECUTION_PERIOD_MS 1			//Execution period in ms
#define LED_TOGGLE_PERIOD_MS 1000		//Toggle of Heartbeat LED
#define LED_PULSE 10

#define PWM_PULSE_PERIOD 840

#define MICROSTEPPING_STEPS 1024

#define CURRENT_LIMIT 1500				// UNIT mA
#define SPEED_MAX 10000.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int targetPos = 0;
int encoderPosition = 0;
int lastEncoderPosition = 0;

double targetSpeed = 0.0;
double encoderSpeed = 0.0;

uint16_t ADC1Buffer[3];
uint16_t ADC2Buffer[2];
uint16_t ADC3Buffer[1];

int busVoltage = 0; // unit mV
int current[2];  	// unit mA

int targetCurrent[2];
int currentError[2];

double outputSignal[2];

double targetVoltage[4];

uint32_t pwmPulse[4];

uint32_t lastCycleTick = 0;
uint32_t lastLEDTick = 0;

double waveFormCounter = 0.0;
uint32_t wavePatternLength = 0;
double sinLookup[MICROSTEPPING_STEPS];
double cosLookup[MICROSTEPPING_STEPS];


int CURRENT_FAULT = 0;
int VOLTAGE_FAULT = 0;
int TEMP_FAULT = 0;
int BRAKE_MON = 0;
int ERROR_FLAG = 0;
int STO_IN = 0;
int STO_Latch = 0;

int LED_FLAG = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void initializePeripherals(void);
void measure(void);
void positionControl(void);
void calculateSpeed(void);
void speedControl(void);
void currentControl(void);
void voltageAdjust(void);
void output(void);
void errorHandling(void);
void loop(void);
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
  lastCycleTick = HAL_GetTick();
  lastLEDTick = HAL_GetTick();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  initializePeripherals();

  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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

/* USER CODE BEGIN 4 */

/* LOOP function called every 100us by TIM7 */
void loop(void){
	if(HAL_GetTick() >= lastCycleTick + EXECUTION_PERIOD_MS)
	{
		lastCycleTick = HAL_GetTick();

		if(HAL_GetTick() > lastLEDTick + LED_TOGGLE_PERIOD_MS)
		{
			lastLEDTick = HAL_GetTick();
			HAL_GPIO_WritePin(GPIOC, LED_HEARTBEAT_Pin, GPIO_PIN_RESET);
			LED_FLAG = 1;
			if(lastLEDTick % 2 == 0)
			{
				if( !ERROR_FLAG )
					targetPos = 0;
				else
					targetPos = encoderPosition;
			}
			else
			{
				if( !ERROR_FLAG )
					targetPos = -3000;
				else
					targetPos = encoderPosition;
			}


		}

		LED_FLAG ++;
		if(LED_FLAG > LED_PULSE)
		{
			HAL_GPIO_WritePin(GPIOC, LED_HEARTBEAT_Pin, GPIO_PIN_SET);
			LED_FLAG = 0;
		}


		/* calculate current Speed */
		calculateSpeed();

		/* position Controller */
		positionControl();

		/* error handling */
		errorHandling();
	}

	/* measure the current values */
	measure();

	/* speed Controller */
	speedControl();

	/* current Controller */
	currentControl();

	/* adjust output Voltages to Bus Voltage */
	voltageAdjust();

	/* output PWM  */
	output();

}

void initializePeripherals(void)
{

	/* Calculate SIN and COS Lookup */
	for( int i = 0; i < MICROSTEPPING_STEPS; i ++)
	{
		sinLookup[i] = sin( ( (double) i / (double) MICROSTEPPING_STEPS ) * 2 * M_PI );
		cosLookup[i] = cos( ( (double) i / (double) MICROSTEPPING_STEPS ) * 2 * M_PI );
	}

	/* read initial states of GPIO */
	CURRENT_FAULT = HAL_GPIO_ReadPin(GPIOC, NFAULT_Pin);
	TEMP_FAULT = HAL_GPIO_ReadPin(GPIOC, NOTW_Pin);
	BRAKE_MON = HAL_GPIO_ReadPin(BRAKE_MON_GPIO_Port, BRAKE_MON_Pin);
	STO_IN = HAL_GPIO_ReadPin(STO_IN_GPIO_Port, STO_IN_Pin);

	/* ADCs */
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3Buffer, 1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1Buffer, 3);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2Buffer, 2);

	/* Timers */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);

	/* Start timer for loop triggering */
	HAL_TIM_Base_Start_IT(&htim8);

}

void measure(void)
{
	encoderPosition = TIM5->CNT;

	uint32_t rawBusVoltage = ADC3Buffer[0];

	int rawCurrent1 = ADC1Buffer[0];
	int rawCurrent2 = ADC2Buffer[0];

	int rawRefCurrent1 = ADC1Buffer[1];
	int rawRefCurrent2 = ADC2Buffer[1];

	current[0] = ( ( (double)(rawCurrent1 - rawRefCurrent1) * (3300.0 / 4096.0) ) / CURRENT_SENSE_AMP ) / 0.01;
	current[1] = ( ( (double)(rawCurrent2 - rawRefCurrent2) * (3300.0 / 4096.0) ) / CURRENT_SENSE_AMP ) / 0.01;

	busVoltage = ( rawBusVoltage * (3.3 / 4096.0) ) * VOLTAGE_SENSE_DIVIDER * 1000; //unit mV

	if(busVoltage > 12500 && busVoltage < 26000 )
		VOLTAGE_FAULT = 1;
	else
		VOLTAGE_FAULT = 0;
}

void positionControl(void)
{
	double posGain = 15;
	double posError = targetPos - encoderPosition;

	targetSpeed = posError * posGain;
}

void calculateSpeed(void)
{
	encoderSpeed = encoderPosition - lastEncoderPosition;
	encoderSpeed = encoderSpeed / 0.1;

	lastEncoderPosition = encoderPosition;

}


void speedControl(void)
{
	if(targetSpeed > SPEED_MAX)
		targetSpeed = SPEED_MAX;

	if(targetSpeed < - SPEED_MAX)
		targetSpeed = - SPEED_MAX;

	double speedGain = 0.002;
	double speedError = targetSpeed - encoderSpeed;

	targetCurrent[0] = cosLookup[ (int)waveFormCounter ] * CURRENT_LIMIT;
	targetCurrent[1] = sinLookup[ (int)waveFormCounter ] * CURRENT_LIMIT;


	waveFormCounter += speedError * speedGain;
	if( waveFormCounter >= MICROSTEPPING_STEPS)
		waveFormCounter = 0;
	if( waveFormCounter < 0)
			waveFormCounter = MICROSTEPPING_STEPS;
}

void currentControl(void)
{

	double currentGain = 0.008;
	for(int i = 0; i < 2; i++)
	{
		currentError[i] = targetCurrent[i] - current[i];
		outputSignal[i] = currentGain * (double)currentError[i];

		if(outputSignal[i] >= 0)
		{
			targetVoltage[i*2] = 0;
			targetVoltage[i*2+1] = outputSignal[i];
		}
		if(outputSignal[i] < 0)
		{
			targetVoltage[i*2] = outputSignal[i];
			targetVoltage[i*2+1] = 0;
		}
	}

}

void voltageAdjust(void)
{

	for(int i = 0; i < 4; i++)
	{
		pwmPulse[i] = abs( (double)PWM_PULSE_PERIOD *   targetVoltage[i] / ( busVoltage * 0.001 ) );

		if(pwmPulse[i] > PWM_PULSE_PERIOD)
			pwmPulse[i] = PWM_PULSE_PERIOD;
	}

}

void output(void)
{
	/* set the reset pins high according to the pwm Command values */
	if( ( pwmPulse[0] >= 0 || pwmPulse[1] >= 0) && ERROR_FLAG == 0 )
	{
		HAL_GPIO_WritePin(GPIOB, NRST_CD_Pin|NRST_AB_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, NRST_CD_Pin|NRST_AB_Pin, GPIO_PIN_RESET);
	}

	if( ( pwmPulse[2] >= 0 || pwmPulse[3] >= 0) && ERROR_FLAG == 0 )
	{
		HAL_GPIO_WritePin(GPIOB, NRST_CD_Pin|NRST_AB_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, NRST_CD_Pin|NRST_AB_Pin, GPIO_PIN_RESET);
	}

	/* set the pulse values of the individual PWM outputs */

	//PWM C matches TIMER 1
	htim1.Instance->CCR1 = pwmPulse[2];

	//PWM A matches TIMER 2
	htim2.Instance->CCR1 = pwmPulse[0];

	//PWM B matches TIMER 3
	htim3.Instance->CCR1 = pwmPulse[1];

	//PWM D matches TIMER 4
	htim4.Instance->CCR1 = pwmPulse[3];

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch ( GPIO_Pin )
	{
		case NFAULT_Pin:
			CURRENT_FAULT = HAL_GPIO_ReadPin(GPIOC, NFAULT_Pin);
			break;
		case NOTW_Pin:
			TEMP_FAULT = HAL_GPIO_ReadPin(GPIOC, NOTW_Pin);
			break;
		case BRAKE_MON_Pin:
			BRAKE_MON = HAL_GPIO_ReadPin(BRAKE_MON_GPIO_Port, BRAKE_MON_Pin);
			break;
		case STO_IN_Pin:
			STO_IN = HAL_GPIO_ReadPin(STO_IN_GPIO_Port, STO_IN_Pin);
			break;
	}

}


void errorHandling(void)
{
	if( CURRENT_FAULT == 0 || TEMP_FAULT == 0 || STO_IN == 0 || VOLTAGE_FAULT == 0)
	{
		ERROR_FLAG = 1;
		HAL_GPIO_WritePin(GPIOC, LED_ERROR_Pin, GPIO_PIN_RESET);
	}
	else
	{
		ERROR_FLAG = 0;
		HAL_GPIO_WritePin(GPIOC, LED_ERROR_Pin, GPIO_PIN_SET);
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM8) {
	  loop();
  }
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
	HAL_GPIO_WritePin(GPIOC, LED_ERROR_Pin, GPIO_PIN_RESET);
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
