/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN1_RS_Pin GPIO_PIN_13
#define CAN1_RS_GPIO_Port GPIOC
#define CAN1_EN_Pin GPIO_PIN_14
#define CAN1_EN_GPIO_Port GPIOC
#define V__SENSE_Pin GPIO_PIN_0
#define V__SENSE_GPIO_Port GPIOC
#define STO_IN_Pin GPIO_PIN_2
#define STO_IN_GPIO_Port GPIOC
#define STO_IN_EXTI_IRQn EXTI2_IRQn
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define IREF_1_Pin GPIO_PIN_2
#define IREF_1_GPIO_Port GPIOA
#define IREF_2_Pin GPIO_PIN_3
#define IREF_2_GPIO_Port GPIOA
#define PWM_A_Pin GPIO_PIN_5
#define PWM_A_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_6
#define PWM_B_GPIO_Port GPIOA
#define I_1_Pin GPIO_PIN_4
#define I_1_GPIO_Port GPIOC
#define I_2_Pin GPIO_PIN_5
#define I_2_GPIO_Port GPIOC
#define HOME_IN_Pin GPIO_PIN_1
#define HOME_IN_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define NRST_CD_Pin GPIO_PIN_14
#define NRST_CD_GPIO_Port GPIOB
#define NRST_AB_Pin GPIO_PIN_15
#define NRST_AB_GPIO_Port GPIOB
#define NOTW_Pin GPIO_PIN_7
#define NOTW_GPIO_Port GPIOC
#define NOTW_EXTI_IRQn EXTI9_5_IRQn
#define NFAULT_Pin GPIO_PIN_8
#define NFAULT_GPIO_Port GPIOC
#define NFAULT_EXTI_IRQn EXTI9_5_IRQn
#define PWM_C_Pin GPIO_PIN_8
#define PWM_C_GPIO_Port GPIOA
#define BRAKE_MON_Pin GPIO_PIN_10
#define BRAKE_MON_GPIO_Port GPIOA
#define BRAKE_MON_EXTI_IRQn EXTI15_10_IRQn
#define LED_ERROR_Pin GPIO_PIN_10
#define LED_ERROR_GPIO_Port GPIOC
#define LED_HEARTBEAT_Pin GPIO_PIN_11
#define LED_HEARTBEAT_GPIO_Port GPIOC
#define LED_RX_Pin GPIO_PIN_12
#define LED_RX_GPIO_Port GPIOC
#define LED_TX_Pin GPIO_PIN_2
#define LED_TX_GPIO_Port GPIOD
#define PWM_D_Pin GPIO_PIN_6
#define PWM_D_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
