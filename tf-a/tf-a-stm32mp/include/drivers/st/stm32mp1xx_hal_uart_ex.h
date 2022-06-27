/**
  ******************************************************************************
  * @file    stm32mp1xx_hal_uart_ex.h
  * @author  MCD Application Team
  * @version $VERSION$
  * @date    $DATE$
  * @brief   Header file of UART HAL Extended module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015-2017 STMicroelectronics</center></h2>
  *
  * SPDX-License-Identifier: BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32MP1xx_HAL_UART_EX_H
#define __STM32MP1xx_HAL_UART_EX_H

/* Exported constants --------------------------------------------------------*/
/** @defgroup UARTEx_Exported_Constants UARTEx Exported Constants
  * @{
  */

/** @defgroup UARTEx_Word_Length UART Word Length
  * @{
  */
#define UART_WORDLENGTH_7B	((uint32_t)USART_CR1_M1)   /*!< 7-bit long UART frame */
#define UART_WORDLENGTH_8B	((uint32_t)0x00000000U)    /*!< 8-bit long UART frame */
#define UART_WORDLENGTH_9B	((uint32_t)USART_CR1_M0)   /*!< 9-bit long UART frame */

/* Private macros ------------------------------------------------------------*/
/** @defgroup UARTEx_Private_Macros UARTEx Private Macros
  * @{
  */

/** @brief  Report the UART mask to apply to retrieve the received data
  *         according to the word length and to the parity bits activation.
  * @note   If PCE = 1, the parity bit is not included in the data extracted
  *         by the reception API().
  *         This masking operation is not carried out in the case of
  *         DMA transfers.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None, the mask to apply to UART RDR register is stored
  * in (__HANDLE__)->Mask field.
  */
#define UART_MASK_COMPUTATION(__HANDLE__)                             \
  do {                                                                \
  if ((__HANDLE__)->Init.WordLength == UART_WORDLENGTH_9B)            \
  {                                                                   \
     if ((__HANDLE__)->Init.Parity == UART_PARITY_NONE)               \
     {                                                                \
        (__HANDLE__)->Mask = 0x01FF ;                                 \
     }                                                                \
     else                                                             \
     {                                                                \
        (__HANDLE__)->Mask = 0x00FF ;                                 \
     }                                                                \
  }                                                                   \
  else if ((__HANDLE__)->Init.WordLength == UART_WORDLENGTH_8B)       \
  {                                                                   \
     if ((__HANDLE__)->Init.Parity == UART_PARITY_NONE)               \
     {                                                                \
        (__HANDLE__)->Mask = 0x00FF ;                                 \
     }                                                                \
     else                                                             \
     {                                                                \
        (__HANDLE__)->Mask = 0x007F ;                                 \
     }                                                                \
  }                                                                   \
  else if ((__HANDLE__)->Init.WordLength == UART_WORDLENGTH_7B)       \
  {                                                                   \
     if ((__HANDLE__)->Init.Parity == UART_PARITY_NONE)               \
     {                                                                \
        (__HANDLE__)->Mask = 0x007F ;                                 \
     }                                                                \
     else                                                             \
     {                                                                \
        (__HANDLE__)->Mask = 0x003F ;                                 \
     }                                                                \
  }                                                                   \
} while (0)

#endif /* __STM32MP1xx_HAL_UART_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
