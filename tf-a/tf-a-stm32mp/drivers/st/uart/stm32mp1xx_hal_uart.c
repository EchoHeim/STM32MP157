/**
  ******************************************************************************
  * @file    stm32mp1xx_hal_uart.c
  * @author  MCD Application Team
  * @version $VERSION$
  * @date    $DATE$
  * @brief   UART HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Universal Asynchronous Receiver Transmitter Peripheral (UART).
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *
  *
  * @verbatim
  * ===============================================================================
  *                      ##### How to use this driver #####
  * ===============================================================================
  * [..]
  * The UART HAL driver can be used as follows:
  *
  *  (#) Declare a UART_HandleTypeDef handle structure (eg. UART_HandleTypeDef huart).
  *  (#) Initialize the UART low level resources by implementing the HAL_UART_MspInit() API:
  *      (++) Enable the USARTx interface clock.
  *      (++) UART pins configuration:
  *          (+++) Enable the clock for the UART GPIOs.
  *          (+++) Configure these UART pins as alternate function pull-up.
  *      (++) NVIC configuration if you need to use interrupt process (HAL_UART_Transmit_IT()
  *           and HAL_UART_Receive_IT() APIs):
  *          (+++) Configure the USARTx interrupt priority.
  *          (+++) Enable the NVIC USART IRQ handle.
  *      (++) UART interrupts handling:
  *            -@@-  The specific UART interrupts (Transmission complete interrupt,
  *              RXNE interrupt and Error Interrupts) are managed using the macros
  *              __HAL_UART_ENABLE_IT() and __HAL_UART_DISABLE_IT() inside the transmit and receive processes.
  *      (++) DMA Configuration if you need to use DMA process (HAL_UART_Transmit_DMA()
  *           and HAL_UART_Receive_DMA() APIs):
  *          (+++) Declare a DMA handle structure for the Tx/Rx channel.
  *          (+++) Enable the DMAx interface clock.
  *          (+++) Configure the declared DMA handle structure with the required Tx/Rx parameters.
  *          (+++) Configure the DMA Tx/Rx channel.
  *          (+++) Associate the initialized DMA handle to the UART DMA Tx/Rx handle.
  *          (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on the DMA Tx/Rx channel.
  *
  *  (#) Program the Baud Rate, Word Length, Stop Bit, Parity, Hardware
  *      flow control and Mode (Receiver/Transmitter) in the huart handle Init structure.
  *
  *  (#) If required, program UART advanced features (TX/RX pins swap, auto Baud rate detection,...)
  *      in the huart handle AdvancedInit structure.
  *
  *  (#) For the UART asynchronous mode, initialize the UART registers by calling
  *      the HAL_UART_Init() API.
  *
  *  (#) For the UART Half duplex mode, initialize the UART registers by calling
  *      the HAL_HalfDuplex_Init() API.
  *
  *  (#) For the UART LIN (Local Interconnection Network) mode, initialize the UART registers
  *      by calling the HAL_LIN_Init() API.
  *
  *  (#) For the UART Multiprocessor mode, initialize the UART registers
  *      by calling the HAL_MultiProcessor_Init() API.
  *
  *  (#) For the UART RS485 Driver Enabled mode, initialize the UART registers
  *      by calling the HAL_RS485Ex_Init() API.
  *
  *  [..]
  *  (@) These API's (HAL_UART_Init(), HAL_HalfDuplex_Init(), HAL_LIN_Init(), HAL_MultiProcessor_Init(),
  *      also configure the low level Hardware GPIO, CLOCK, CORTEX...etc) by
  *      calling the customized HAL_UART_MspInit() API.
  *
  * @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015-2017 STMicroelectronics</center></h2>
  *
  * SPDX-License-Identifier: BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <string.h>

#include <platform_def.h>

#include <common/bl_common.h>
#include <drivers/st/stm32mp_clkfunc.h>
#include <drivers/st/stm32mp1xx_hal_uart.h>

/** @addtogroup STM32MP1xx_HAL_Driver
  * @{
  */

/** @defgroup UART UART
  * @brief HAL UART module driver
  * @{
  */

#ifdef HAL_UART_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup UART_Private_Constants UART Private Constants
  * @{
  */
#define UART_CR1_FIELDS \
	  ((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS |    \
		      USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8 | \
		      USART_CR1_FIFOEN))

#define USART_CR3_FIELDS \
	  ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT | \
		      USART_CR3_TXFTCFG | USART_CR3_RXFTCFG))


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup UART_Exported_Functions UART Exported Functions
  * @{
  */

/** @defgroup UART_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
  * @verbatim
  * ===============================================================================
  * ##### Initialization and Configuration functions #####
  * ===============================================================================
  * [..]
  * This subsection provides a set of functions allowing to initialize the USARTx or the UARTy
  * in asynchronous mode.
  *    (+) For the asynchronous mode the parameters below can be configured:
  *      (++) Baud Rate
  *      (++) Word Length
  *      (++) Stop Bit
  *      (++) Parity: If the parity is enabled, then the MSB bit of the data written
  *           in the data register is transmitted but is changed by the parity bit.
  *      (++) Hardware flow control
  *      (++) Receiver/transmitter modes
  *      (++) Over Sampling Method
  *      (++) One-Bit Sampling Method
  *    (+) For the asynchronous mode, the following advanced features can be configured as well:
  *      (++) TX and/or RX pin level inversion
  *      (++) data logical level inversion
  *      (++) RX and TX pins swap
  *      (++) RX overrun detection disabling
  *      (++) DMA disabling on RX error
  *      (++) MSB first on communication line
  *      (++) auto Baud rate detection
  *  [..]
  *  The HAL_UART_Init(), HAL_HalfDuplex_Init(), HAL_LIN_Init()and HAL_MultiProcessor_Init()API
  *  follow respectively the UART asynchronous, UART Half duplex, UART LIN mode
  *  and UART multiprocessor mode configuration procedures (details for the procedures
  *  are available in reference manual).
  *
  * @endverbatim
  *
  * Depending on the frame length defined by the M1 and M0 bits (7-bit,
  * 8-bit or 9-bit), the possible UART formats are listed in the
  * following table.
  *
  * Table 1. UART frame format.
  *  +-----------------------------------------------------------------------+
  *  |  M1 bit |  M0 bit |  PCE bit  |             UART frame                |
  *  |---------|---------|-----------|---------------------------------------|
  *  |    0    |    0    |    0      |    | SB |    8 bit data   | STB |     |
  *  |---------|---------|-----------|---------------------------------------|
  *  |    0    |    0    |    1      |    | SB | 7 bit data | PB | STB |     |
  *  |---------|---------|-----------|---------------------------------------|
  *  |    0    |    1    |    0      |    | SB |    9 bit data   | STB |     |
  *  |---------|---------|-----------|---------------------------------------|
  *  |    0    |    1    |    1      |    | SB | 8 bit data | PB | STB |     |
  *  |---------|---------|-----------|---------------------------------------|
  *  |    1    |    0    |    0      |    | SB |    7 bit data   | STB |     |
  *  |---------|---------|-----------|---------------------------------------|
  *  |    1    |    0    |    1      |    | SB | 6 bit data | PB | STB |     |
  *  +-----------------------------------------------------------------------+
  *
  * @{
  */

/**
  * @brief Initialize the UART mode according to the specified
  *        parameters in the UART_InitTypeDef and initialize the associated handle.
  * @param huart: UART handle.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart)
{
	/* Check the UART handle allocation */
	if (!huart)
		return HAL_ERROR;

	/* Check the parameters */
	if (huart->Init.HwFlowCtl != UART_HWCONTROL_NONE)
		assert_param(IS_UART_HWFLOW_INSTANCE(huart->Instance));
	else
		assert_param(IS_UART_INSTANCE(huart->Instance));

	/* Allocate lock resource and initialize it */
	if (huart->gState == HAL_UART_STATE_RESET)
		huart->Lock = HAL_UNLOCKED;

	huart->gState = HAL_UART_STATE_BUSY;

	/* Disable the Peripheral */
	__HAL_UART_DISABLE(huart);

	/* Set the UART Communication parameters */
	if (UART_SetConfig(huart) == HAL_ERROR)
		return HAL_ERROR;

	if (huart->AdvancedInit.AdvFeatureInit != UART_ADVFEATURE_NO_INIT)
		UART_AdvFeatureConfig(huart);

	/* In asynchronous mode, the following bits must be kept cleared:
	   - LINEN and CLKEN bits in the USART_CR2 register,
	   - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
	CLEAR_BIT(huart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
	CLEAR_BIT(huart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL |
					 USART_CR3_IREN));

	/* Enable the Peripheral */
	__HAL_UART_ENABLE(huart);

	/* TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready */
	return UART_CheckIdleState(huart);
}


/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group2 IO operation functions
  * @brief UART Transmit/Receive functions
  *
  * @verbatim
  * ===============================================================================
  *                    ##### IO operation functions #####
  * ===============================================================================
  * This subsection provides a set of functions allowing to manage the UART asynchronous
  * and Half duplex data transfers.
  *
  *  (#) There are two mode of transfer:
  *     (+) Blocking mode: The communication is performed in polling mode.
  *         The HAL status of all data processing is returned by the same function
  *         after finishing transfer.
  *     (+) Non-Blocking mode: The communication is performed using Interrupts
  *         or DMA, These API's return the HAL status.
  *         The end of the data processing will be indicated through the
  *         dedicated UART IRQ when using Interrupt mode or the DMA IRQ when
  *         using DMA mode.
  *         The HAL_UART_TxCpltCallback(), HAL_UART_RxCpltCallback() user callbacks
  *         will be executed respectively at the end of the transmit or Receive process
  *         The HAL_UART_ErrorCallback()user callback will be executed when a communication error is detected
  *
  *  (#) Blocking mode API's are :
  *      (+) HAL_UART_Transmit()
  *      (+) HAL_UART_Receive()
  *
  *  (#) Non-Blocking mode API's with Interrupt are :
  *      (+) HAL_UART_Transmit_IT()
  *      (+) HAL_UART_Receive_IT()
  *      (+) HAL_UART_IRQHandler()
  *
  *  (#) Non-Blocking mode API's with DMA are :
  *      (+) HAL_UART_Transmit_DMA()
  *      (+) HAL_UART_Receive_DMA()
  *      (+) HAL_UART_DMAPause()
  *      (+) HAL_UART_DMAResume()
  *      (+) HAL_UART_DMAStop()
  *
  *  (#) A set of Transfer Complete Callbacks are provided in Non_Blocking mode:
  *      (+) HAL_UART_TxHalfCpltCallback()
  *      (+) HAL_UART_TxCpltCallback()
  *      (+) HAL_UART_RxHalfCpltCallback()
  *      (+) HAL_UART_RxCpltCallback()
  *      (+) HAL_UART_ErrorCallback()
  *
  * -@- In the Half duplex communication, it is forbidden to run the transmit
  *  and receive process in parallel, the UART state HAL_UART_STATE_BUSY_TX_RX can't be useful.
  *
  * @endverbatim
  * @{
  */

/**
  * @brief Send an amount of data in blocking mode.
  * @param huart: UART handle.
  * @param pData: Pointer to data buffer.
  * @param Size: Amount of data to be sent.
  * @param Timeout: Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
				    uint16_t Size, uint32_t Timeout)
{
	uint16_t *tmp;
	uint32_t tickstart = 0U;
	HAL_StatusTypeDef ret = HAL_OK;

	/* Check that a Tx process is not already ongoing */
	if (huart->gState != HAL_UART_STATE_READY)
		return HAL_BUSY;

	if ((!pData) || (Size == 0U))
		return  HAL_ERROR;

	/* Process Locked */
	__HAL_LOCK(huart);

	huart->ErrorCode = HAL_UART_ERROR_NONE;
	huart->gState = HAL_UART_STATE_BUSY_TX;

	/* Init tickstart for timeout management*/
	tickstart = HAL_GetTick();

	huart->TxXferSize = Size;
	huart->TxXferCount = Size;
	while (huart->TxXferCount > 0U) {
		huart->TxXferCount--;
		if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET,
						tickstart, Timeout) != HAL_OK) {
			ret = HAL_TIMEOUT;
			goto end;
		}

		if ((huart->Init.WordLength == UART_WORDLENGTH_9B) &&
		    (huart->Init.Parity == UART_PARITY_NONE)) {
			tmp = (uint16_t *)pData;
			huart->Instance->TDR = (*tmp & (uint16_t)0x01FFU);
			pData += 2U;
		} else {
			huart->Instance->TDR = (*pData++ & (uint8_t)0xFFU);
		}
	}

	if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TC, RESET, tickstart,
					Timeout) != HAL_OK) {
		ret = HAL_TIMEOUT;
		goto end;
	}

end:
	/* At end of Tx process, restore huart->gState to Ready */
	huart->gState = HAL_UART_STATE_READY;

	/* Process Unlocked */
	__HAL_UNLOCK(huart);

	return ret;
}

/**
  * @brief Receive an amount of data in blocking mode.
  * @param huart: UART handle.
  * @param pData: pointer to data buffer.
  * @param Size: amount of data to be received.
  * @param Timeout: Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData,
				   uint16_t Size, uint32_t Timeout)
{
	uint16_t *tmp;
	uint16_t uhMask;
	uint32_t tickstart = 0;
	HAL_StatusTypeDef ret = HAL_OK;

	/* Check that a Rx process is not already ongoing */
	if (huart->RxState != HAL_UART_STATE_READY)
		return HAL_BUSY;

	if ((!pData) || (Size == 0U))
		return  HAL_ERROR;

	/* Process Locked */
	__HAL_LOCK(huart);

	huart->ErrorCode = HAL_UART_ERROR_NONE;
	huart->RxState = HAL_UART_STATE_BUSY_RX;

	/* Init tickstart for timeout management*/
	tickstart = HAL_GetTick();

	huart->RxXferSize = Size;
	huart->RxXferCount = Size;

	/* Computation of UART mask to apply to RDR register */
	UART_MASK_COMPUTATION(huart);
	uhMask = huart->Mask;

	/* as long as data have to be received */
	while (huart->RxXferCount > 0U) {
		huart->RxXferCount--;
		if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_RXNE, RESET,
						tickstart, Timeout) != HAL_OK) {
			ret = HAL_TIMEOUT;
			goto end;
		}

		if ((huart->Init.WordLength == UART_WORDLENGTH_9B) &&
		    (huart->Init.Parity == UART_PARITY_NONE)) {
			tmp = (uint16_t *)pData;
			*tmp = (uint16_t)(huart->Instance->RDR & uhMask);
			pData += 2U;
		} else {
			*pData++ = (uint8_t)(huart->Instance->RDR &
					     (uint8_t)uhMask);
		}
	}

end:
	/* At end of Rx process, restore huart->RxState to Ready */
	huart->RxState = HAL_UART_STATE_READY;

	/* Process Unlocked */
	__HAL_UNLOCK(huart);

	return ret;
}


/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group3 Peripheral Control functions
  *  @brief   UART control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the UART.
     (+) HAL_MultiProcessor_EnableMuteMode() API enables mute mode
     (+) HAL_MultiProcessor_DisableMuteMode() API disables mute mode
     (+) HAL_MultiProcessor_EnterMuteMode() API enters mute mode
     (+) HAL_MultiProcessor_EnableMuteMode() API enables mute mode
     (+) UART_SetConfig() API configures the UART peripheral
     (+) UART_AdvFeatureConfig() API optionally configures the UART advanced features
     (+) UART_CheckIdleState() API ensures that TEACK and/or REACK are set after initialization
     (+) UART_Wakeup_AddressConfig() API configures the wake-up from stop mode parameters
     (+) HAL_HalfDuplex_EnableTransmitter() API disables receiver and enables transmitter
     (+) HAL_HalfDuplex_EnableReceiver() API disables transmitter and enables receiver
     (+) HAL_LIN_SendBreak() API transmits the break characters
@endverbatim
  * @{
  */



/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group4 Peripheral State and Error functions
 *  @brief   UART Peripheral State functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral State and Error functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) Return the UART handle state.
      (+) Return the UART handle error code

@endverbatim
  * @{
  */

/**
  * @brief  Return the UART handle state.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART.
  * @retval HAL state
  */
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart)
{
	uint32_t temp1 = huart->gState;
	uint32_t temp2 = huart->RxState;

	return (HAL_UART_StateTypeDef)(temp1 | temp2);
}

/**
  * @brief  Return the UART handle error code.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART.
  * @retval UART Error Code
*/
uint32_t HAL_UART_GetError(UART_HandleTypeDef *huart)
{
	return huart->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup UART_Private_Functions UART Private Functions
  * @{
  */

static unsigned long uart_get_clock_freq(UART_HandleTypeDef *huart)
{
	return fdt_get_uart_clock_freq((uintptr_t)huart->Instance);
}

/**
  * @brief Configure the UART peripheral.
  * @param huart: UART handle.
  * @retval HAL status
  */
HAL_StatusTypeDef UART_SetConfig(UART_HandleTypeDef *huart)
{
	uint32_t tmpreg;
	unsigned long clockfreq;

	/*---------------------- USART CR1 Configuration --------------------
	 * Clear M, PCE, PS, TE, RE and OVER8 bits and configure
	 * the UART Word Length, Parity, Mode and oversampling:
	 * set the M bits according to huart->Init.WordLength value
	 * set PCE and PS bits according to huart->Init.Parity value
	 * set TE and RE bits according to huart->Init.Mode value
	 * set OVER8 bit according to huart->Init.OverSampling value
	 */
	tmpreg = (uint32_t)(huart->Init.WordLength |
			    huart->Init.Parity |
			    huart->Init.Mode |
			    huart->Init.OverSampling);
	tmpreg |=  (uint32_t)huart->Init.FIFOMode;
	MODIFY_REG(huart->Instance->CR1, UART_CR1_FIELDS, tmpreg);

	/*--------------------- USART CR2 Configuration ---------------------
	 * Configure the UART Stop Bits: Set STOP[13:12] bits according
	 * to huart->Init.StopBits value
	 */
	MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits);

	/*--------------------- USART CR3 Configuration ---------------------
	 * Configure
	 * - UART HardWare Flow Control: set CTSE and RTSE bits according
	 *   to huart->Init.HwFlowCtl value
	 * - one-bit sampling method versus three samples' majority rule
	 *   according to huart->Init.OneBitSampling (not applicable to LPUART)
	 * - set TXFTCFG bit according to husart->Init.TXFIFOThreshold value
	 * - set RXFTCFG bit according to husart->Init.RXFIFOThreshold value
	 */
	tmpreg = (uint32_t)huart->Init.HwFlowCtl;

	tmpreg |= huart->Init.OneBitSampling;

	if (huart->Init.FIFOMode == UART_FIFOMODE_ENABLE)
		tmpreg |= ((uint32_t)huart->Init.TXFIFOThreshold |
			   (uint32_t)huart->Init.RXFIFOThreshold);

	MODIFY_REG(huart->Instance->CR3, USART_CR3_FIELDS, tmpreg);

	/*--------------------- USART PRESC Configuration -------------------
	 * Configure
	 * - UART Clock Prescaler : set PRESCALER according to
	 * huart->Init.Prescaler value
	 */
	MODIFY_REG(huart->Instance->PRESC, USART_PRESC_PRESCALER,
		   huart->Init.Prescaler);

	/*-------------------------- USART BRR Configuration -----------------------*/

	clockfreq = uart_get_clock_freq(huart);
	if (!clockfreq)
		return HAL_ERROR;

	if (huart->Init.OverSampling == UART_OVERSAMPLING_8) {
		uint16_t usartdiv =
			(uint16_t)uart_div_sampling8(clockfreq,
						     huart->Init.BaudRate,
						     huart->Init.Prescaler);

		uint16_t brrtemp = usartdiv & 0xFFF0U;

		brrtemp |= (uint16_t)((usartdiv & (uint16_t)0x000FU) >> 1U);
		huart->Instance->BRR = brrtemp;
	} else {
		huart->Instance->BRR =
			(uint16_t)uart_div_sampling16(clockfreq,
						      huart->Init.BaudRate,
						      huart->Init.Prescaler);
	}

	return HAL_OK;
}

/**
  * @brief Configure the UART peripheral advanced features.
  * @param huart: UART handle.
  * @retval None
  */
void UART_AdvFeatureConfig(UART_HandleTypeDef *huart)
{
	/* Check whether the set of advanced features to configure is properly set */
	assert_param(
		IS_UART_ADVFEATURE_INIT(huart->AdvancedInit.AdvFeatureInit));

	/* if required, configure TX pin active level inversion */
	if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit,
			   UART_ADVFEATURE_TXINVERT_INIT)) {
		assert_param(
			IS_UART_ADVFEATURE_TXINV(
					huart->AdvancedInit.TxPinLevelInvert));
		MODIFY_REG(huart->Instance->CR2, USART_CR2_TXINV,
			   huart->AdvancedInit.TxPinLevelInvert);
	}

	/* if required, configure RX pin active level inversion */
	if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit,
			   UART_ADVFEATURE_RXINVERT_INIT)) {
		assert_param(
			IS_UART_ADVFEATURE_RXINV(
					huart->AdvancedInit.RxPinLevelInvert));
		MODIFY_REG(huart->Instance->CR2, USART_CR2_RXINV,
			   huart->AdvancedInit.RxPinLevelInvert);
	}

	/* if required, configure data inversion */
	if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit,
			   UART_ADVFEATURE_DATAINVERT_INIT)) {
		assert_param(
			IS_UART_ADVFEATURE_DATAINV(
					huart->AdvancedInit.DataInvert));
		MODIFY_REG(huart->Instance->CR2, USART_CR2_DATAINV,
			   huart->AdvancedInit.DataInvert);
	}

	/* if required, configure RX/TX pins swap */
	if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit,
			   UART_ADVFEATURE_SWAP_INIT)) {
		assert_param(IS_UART_ADVFEATURE_SWAP(huart->AdvancedInit.Swap));
		MODIFY_REG(huart->Instance->CR2, USART_CR2_SWAP,
			   huart->AdvancedInit.Swap);
	}

	/* if required, configure RX overrun detection disabling */
	if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit,
			   UART_ADVFEATURE_RXOVERRUNDISABLE_INIT)) {
		assert_param(IS_UART_OVERRUN(
					huart->AdvancedInit.OverrunDisable));
		MODIFY_REG(huart->Instance->CR3, USART_CR3_OVRDIS,
			   huart->AdvancedInit.OverrunDisable);
	}

	/* if required, configure DMA disabling on reception error */
	if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit,
			   UART_ADVFEATURE_DMADISABLEONERROR_INIT)) {
		assert_param(
			IS_UART_ADVFEATURE_DMAONRXERROR(
				huart->AdvancedInit.DMADisableonRxError));
		MODIFY_REG(huart->Instance->CR3, USART_CR3_DDRE,
			   huart->AdvancedInit.DMADisableonRxError);
	}

	/* if required, configure auto Baud rate detection scheme */
	if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit,
			   UART_ADVFEATURE_AUTOBAUDRATE_INIT)) {
		assert_param(
			IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(
							huart->Instance));
		assert_param(
			IS_UART_ADVFEATURE_AUTOBAUDRATE(
				huart->AdvancedInit.AutoBaudRateEnable));
		MODIFY_REG(huart->Instance->CR2, USART_CR2_ABREN,
			   huart->AdvancedInit.AutoBaudRateEnable);
		/* set auto Baudrate detection parameters if detection is enabled */
		if (huart->AdvancedInit.AutoBaudRateEnable ==
		    UART_ADVFEATURE_AUTOBAUDRATE_ENABLE) {
			assert_param(
				IS_UART_ADVFEATURE_AUTOBAUDRATEMODE(
					huart->AdvancedInit.AutoBaudRateMode));
			MODIFY_REG(huart->Instance->CR2, USART_CR2_ABRMODE,
				   huart->AdvancedInit.AutoBaudRateMode);
		}
	}

	/* if required, configure MSB first on communication line */
	if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit,
			   UART_ADVFEATURE_MSBFIRST_INIT)) {
		assert_param(
			IS_UART_ADVFEATURE_MSBFIRST(
						huart->AdvancedInit.MSBFirst));
		MODIFY_REG(huart->Instance->CR2, USART_CR2_MSBFIRST,
			   huart->AdvancedInit.MSBFirst);
	}
}

/**
  * @brief Check the UART Idle State.
  * @param huart: UART handle.
  * @retval HAL status
  */
HAL_StatusTypeDef UART_CheckIdleState(UART_HandleTypeDef *huart)
{
	uint32_t tickstart;

	/* Initialize the UART ErrorCode */
	huart->ErrorCode = HAL_UART_ERROR_NONE;

	/* Init tickstart for timeout management*/
	tickstart = HAL_GetTick();

	/* Check if the Transmitter is enabled */
	if ((huart->Instance->CR1 & USART_CR1_TE) == USART_CR1_TE) {
		/* Wait until TEACK flag is set */
		if (UART_WaitOnFlagUntilTimeout(huart, USART_ISR_TEACK,
						RESET, tickstart,
						HAL_UART_TIMEOUT_VALUE) !=
		    HAL_OK) {
			/* Timeout occurred */
			return HAL_TIMEOUT;
		}
	}
	/* Check if the Receiver is enabled */
	if ((huart->Instance->CR1 & USART_CR1_RE) == USART_CR1_RE) {
		/* Wait until REACK flag is set */
		if (UART_WaitOnFlagUntilTimeout(huart, USART_ISR_REACK, RESET,
						tickstart,
						HAL_UART_TIMEOUT_VALUE) !=
		   HAL_OK) {
			/* Timeout occurred */
			return HAL_TIMEOUT;
		}
	}

	/* Initialize the UART State */
	huart->gState = HAL_UART_STATE_READY;
	huart->RxState = HAL_UART_STATE_READY;

	/* Process Unlocked */
	__HAL_UNLOCK(huart);

	return HAL_OK;
}

/**
  * @brief  Handle UART Communication Timeout.
  * @param  huart: UART handle.
  * @param  Flag Specifies the UART flag to check
  * @param  Status Flag status (SET or RESET)
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart,
					      uint32_t Flag, FlagStatus Status,
					      uint32_t Tickstart,
					      uint32_t Timeout)
{
	/* Wait until flag is set */
	while ((__HAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status) {
		/* Check for the Timeout */
		if (Timeout != HAL_MAX_DELAY) {
			if ((Timeout == 0U) ||
			    ((HAL_GetTick() - Tickstart) > Timeout)) {
				/*
				 * Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error)
				 * interrupts for the interrupt process
				 */
				CLEAR_BIT(huart->Instance->CR1,
					  (USART_CR1_RXNEIE | USART_CR1_PEIE |
					   USART_CR1_TXEIE));
				CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

				huart->gState = HAL_UART_STATE_READY;
				huart->RxState = HAL_UART_STATE_READY;

				/* Process Unlocked */
				__HAL_UNLOCK(huart);

				return HAL_TIMEOUT;
			}
		}
	}
	return HAL_OK;
}

#endif /* HAL_UART_MODULE_ENABLED */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
