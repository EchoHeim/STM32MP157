/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __STM32_HAL_H
#define __STM32_HAL_H

#include <stdint.h>
#include <stddef.h>

#include <arch_helpers.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>

typedef enum {
	HAL_OK       = 0x00,
	HAL_ERROR    = 0x01,
	HAL_BUSY     = 0x02,
	HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

typedef enum {
	HAL_UNLOCKED = 0x00,
	HAL_LOCKED   = 0x01
} HAL_LockTypeDef;

typedef enum {
	RESET = 0,
	SET = !RESET
} FlagStatus, ITStatus;

typedef uint32_t Std_ReturnType;

#define STD_OK         ((uint32_t)0x77)
#define STD_NOT_OK     ((uint32_t)0x66)

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  \
	WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) != RESET)
#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == RESET)

#define SET_BIT(_reg, _val)	mmio_setbits_32((uintptr_t)&(_reg), _val)
#define CLEAR_BIT(_reg, _val)	mmio_clrbits_32((uintptr_t)&(_reg), _val)

#define	__IO    volatile             /*!< Defines 'read / write' permissions */
#define	__I     volatile const       /*!< Defines 'read only' permissions    */

#define HAL_MAX_DELAY      0xFFFFFFFF

#define assert_param(expr) ((void)0)

#define HAL_GetTick() (uint32_t)read_cntpct_el0()

static inline void HAL_Delay(uint32_t x)
{
	udelay(x);
}

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct {
	__IO uint32_t CR1;    /*!< USART Control register 1,                 Address offset: 0x00 */
	__IO uint32_t CR2;    /*!< USART Control register 2,                 Address offset: 0x04 */
	__IO uint32_t CR3;    /*!< USART Control register 3,                 Address offset: 0x08 */
	__IO uint32_t BRR;    /*!< USART Baud rate register,                 Address offset: 0x0C */
	__IO uint16_t GTPR;   /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
	uint16_t  RESERVED2;  /*!< Reserved, 0x12                                                 */
	__IO uint32_t RTOR;   /*!< USART Receiver Time Out register,         Address offset: 0x14 */
	__IO uint16_t RQR;    /*!< USART Request register,                   Address offset: 0x18 */
	uint16_t  RESERVED3;  /*!< Reserved, 0x1A                                                 */
	__IO uint32_t ISReg;    /*!< USART Interrupt and status register,      Address offset: 0x1C */
	__IO uint32_t ICR;    /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
	__IO uint16_t RDR;    /*!< USART Receive Data register,              Address offset: 0x24 */
	uint16_t  RESERVED4;  /*!< Reserved, 0x26                                                 */
	__IO uint16_t TDR;    /*!< USART Transmit Data register,             Address offset: 0x28 */
	uint16_t  RESERVED5;  /*!< Reserved, 0x2A                                                 */
	__IO uint32_t PRESC;  /*!< USART clock Prescaler register,           Address offset: 0x2C */
} USART_TypeDef;

typedef struct {
	__IO uint32_t POWER;          /*!< SDMMC power control register,    Address offset: 0x00 */
	__IO uint32_t CLKCR;          /*!< SDMMC clock control register,    Address offset: 0x04 */
	__IO uint32_t ARG;            /*!< SDMMC argument register,         Address offset: 0x08 */
	__IO uint32_t CMD;            /*!< SDMMC command register,          Address offset: 0x0C */
	__I uint32_t  RESPCMD;        /*!< SDMMC command response register, Address offset: 0x10 */
	__I uint32_t  RESP1;          /*!< SDMMC response 1 register,       Address offset: 0x14 */
	__I uint32_t  RESP2;          /*!< SDMMC response 2 register,       Address offset: 0x18 */
	__I uint32_t  RESP3;          /*!< SDMMC response 3 register,       Address offset: 0x1C */
	__I uint32_t  RESP4;          /*!< SDMMC response 4 register,       Address offset: 0x20 */
	__IO uint32_t DTIMER;         /*!< SDMMC data timer register,       Address offset: 0x24 */
	__IO uint32_t DLEN;           /*!< SDMMC data length register,      Address offset: 0x28 */
	__IO uint32_t DCTRL;          /*!< SDMMC data control register,     Address offset: 0x2C */
	__I uint32_t  DCOUNT;         /*!< SDMMC data counter register,     Address offset: 0x30 */
	__I uint32_t  STA;            /*!< SDMMC status register,           Address offset: 0x34 */
	__IO uint32_t ICR;            /*!< SDMMC interrupt clear register,  Address offset: 0x38 */
	__IO uint32_t MASK;           /*!< SDMMC mask register,             Address offset: 0x3C */
	__IO uint32_t ACKTIME;        /*!< SDMMC Acknowledgment timer register,     Address offset: 0x40 */
	uint32_t      RESERVED0[3];   /*!< Reserved, 0x44 - 0x4C - 0x4C                                   */
	__IO uint32_t IDMACTRL;       /*!< SDMMC DMA control register,               Address offset: 0x50 */
	__IO uint32_t IDMABSIZE;      /*!< SDMMC DMA buffer size register,           Address offset: 0x54 */
	__IO uint32_t IDMABASE0;      /*!< SDMMC DMA buffer 0 base address register, Address offset: 0x58 */
	__IO uint32_t IDMABASE1;      /*!< SDMMC DMA buffer 1 base address register, Address offset: 0x5C */
	uint32_t      RESERVED1[8];   /*!< Reserved, 0x4C-0x7C                                            */
	__IO uint32_t FIFO;           /*!< SDMMC data FIFO register,                 Address offset: 0x80 */
} SDMMC_TypeDef;

typedef struct
{
	__IO uint32_t BCR1;   /*!< Address offset: 0x00 */
	__IO uint32_t BTR1;   /*!< Address offset: 0x04 */
	__IO uint32_t BCR2;   /*!< Address offset: 0x08 */
	__IO uint32_t BTR2;   /*!< Address offset: 0x0C */
	__IO uint32_t BCR3;   /*!< Address offset: 0x10 */
	__IO uint32_t BTR3;   /*!< Address offset: 0x14 */
	__IO uint32_t BCR4;   /*!< Address offset: 0x18 */
	__IO uint32_t BTR4;   /*!< Address offset: 0x1C */
	uint32_t      RESERVED0[24];
	__IO uint32_t PCReg;    /*!< Address offset: 0x80 */
	__IO uint32_t SR;     /*!< Address offset: 0x84 */
	__IO uint32_t PMEM;   /*!< Address offset: 0x88 */
	__IO uint32_t PATT;   /*!< Address offset: 0x8C */
	__IO uint32_t HPR;    /*! Address offset: 0x90 */
	__IO uint32_t HECCR;  /*!< Address offset: 0x94 */
	uint32_t      RESERVED2[27];
	__IO uint32_t BWTR1;  /*!< Address offset: 0x104 */
	uint32_t      RESERVED3;
	__IO uint32_t BWTR2;  /*!< Address offset: 0x10C */
	uint32_t      RESERVED4;
	__IO uint32_t BWTR3;  /*!< Address offset: 0x114 */
	uint32_t      RESERVED5;
	__IO uint32_t BWTR4;   /*!< Address offset: 0x11c */
	uint32_t      RESERVED6[8];
	__IO uint32_t SDCR1;  /*!< Address offset: 0x140 */
	__IO uint32_t SDCR2;  /*!< Address offset: 0x144 */
	__IO uint32_t SDTR1;  /*!< Address offset: 0x148 */
	__IO uint32_t SDTR2;  /*!<  Address offset: 0x14c */
	__IO uint32_t SDCMR;  /*!<  Address offset: 0x150 */
	__IO uint32_t SDRTR;  /*!< Address offset: 0x154 */
	__IO uint32_t SDSR;   /*!< Address offset: 0x158 */
	uint32_t      RESERVED7[9];
	__IO uint32_t IER;    /*!< Address offset: 0x180 */
	__IO uint32_t ISReg;    /*!< Address offset: 0x184 */
	__IO uint32_t ICR;    /*!< Address offset: 0x188 */
	uint32_t      RESERVED8[29];
	__IO uint32_t CSQCR;  /*!< Address offset: 0x200 */
	__IO uint32_t CSQCFGR1;   /*!< Address offset: 0x204 */
	__IO uint32_t CSQCFGR2;   /*!< Address offset: 0x208 */
	__IO uint32_t CSQCFGR3;   /*!< Address offset: 0x20c */
	__IO uint32_t CSQAR1;     /*!< Address offset: 0x210 */
	__IO uint32_t CSQAR2;     /*!< Address offset: 0x214 */
	uint32_t      RESERVED9[2];
	__IO uint32_t CSQIER;     /*!< Address offset: 0x220 */
	__IO uint32_t CSQISR;     /*!< Address offset: 0x224 */
	__IO uint32_t CSQICR;     /*!< Address offset: 0x228 */
	uint32_t      RESERVED10;
	__IO uint32_t CSQEMSR;    /*!< Address offset: 0x230 */
	uint32_t      RESERVED11[7];
	__IO uint32_t BCHIER;     /*!< Address offset: 0x250 */
	__IO uint32_t BCHISR;     /*!< Address offset: 0x254 */
	__IO uint32_t BCHICR;     /*!< Address offset: 0x258 */
	__IO uint32_t BCHSR;      /*!< Address offset: 0x25c */
	__IO uint32_t BCHPBR1;    /*!< Address offset: 0x260 */
	__IO uint32_t BCHPBR2;    /*!< Address offset: 0x264 */
	__IO uint32_t BCHPBR3;    /*!< Address offset: 0x268 */
	__IO uint32_t BCHPBR4;    /*!< Address offset: 0x26c */
	uint32_t      RESERVED12[3];
	__IO uint32_t BCHDSR0;    /*!< Address offset: 0x27c */
	__IO uint32_t BCHDSR1;    /*!< Address offset: 0x280 */
	__IO uint32_t BCHDSR2;    /*!< Address offset: 0x284 */
	__IO uint32_t BCHDSR3;    /*!< Address offset: 0x288 */
	__IO uint32_t BCHDSR4;    /*!< Address offset: 0x28c */
	uint32_t      RESERVED13[347];
	__IO uint32_t HWCFGR2;    /*!< Address offset: 0x3ec */
	__IO uint32_t HWCFGR1;    /*!< Address offset: 0x3f0 */
	__IO uint32_t VER;        /*!< Address offset: 0x3f4 */
	__IO uint32_t ID;         /*!< Address offset: 0x3f8 */
	__IO uint32_t SID;        /*!< Address offset: 0x3fc */
} FMC_TypeDef;

#define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                  }while (0)

  #define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                    }while (0)

#endif /*__STM32_HAL_H*/
