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
#include "stm32mp1xx_hal.h"

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

void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup);

/* USER CODE BEGIN EFP */

extern GPIO_TypeDef * const digital_regs[];

#define GPIO(PORT, NUM) (((PORT)-'A') * 16 + (NUM))
#define GPIO2PORT(PIN) ((PIN) / 16)
#define GPIO2BIT(PIN) (1<<((PIN) % 16))

#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_OPEN_DRAIN 0x100
#define GPIO_FUNCTION(fn) (2 | ((fn) << 4))
#define GPIO_ANALOG 3

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UART4_TX_Pin GPIO_PIN_11
#define UART4_TX_GPIO_Port GPIOG
#define UART4_RX_Pin GPIO_PIN_2
#define UART4_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
