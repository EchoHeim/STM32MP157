/*
 * Copyright (c) 2016-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_SMC_H
#define STM32MP1_SMC_H

/* SMC service generic return codes */
#define STM32_SMC_OK			0x00000000U
#define STM32_SMC_NOT_SUPPORTED		0xFFFFFFFFU
#define STM32_SMC_FAILED		0xFFFFFFFEU
#define STM32_SMC_INVALID_PARAMS	0xFFFFFFFDU

/*
 * SMC function IDs for STM32 Service queries.
 * STM32 SMC services use the space between 0x82000000 and 0x8200FFFF
 * like this is defined in SMC calling Convention by ARM
 * for SiP (silicon Partner).
 * https://developer.arm.com/docs/den0028/latest
 */

/* Secure Service access from Non-secure */

/*
 * SMC function STM32_SMC_RCC.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a1: (input) Service ID (STM32_SMC_REG_xxx).
 * Argument a2: (input) Register offset or physical address.
 *		(output) Register read value, if applicable.
 * Argument a3: (input) Register target value if applicable.
 */
#define STM32_SMC_RCC			0x82001000

/*
 * SMC function STM32_SMC_PWR.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a1: (input) Service ID (STM32_SMC_REG_xxx).
 * Argument a2: (input) Register offset or physical address.
 *		(output) Register read value, if applicable.
 * Argument a3: (input) Register target value if applicable.
 */
#define STM32_SMC_PWR			0x82001001

/*
 * SMC functions STM32_SMC_RCC_CAL.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a1: (input) Clock ID (from DT clock bindings).
 */
#define STM32_SMC_RCC_CAL		0x82001002

/*
 * SMC functions STM32_SMC_BSEC.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a1: (input) Service ID (STM32_SMC_READ_xxx/_PROG_xxx/_WRITE_xxx).
 *		(output) OTP read value, if applicable.
 * Argument a2: (input) OTP index.
 * Argument a3: (input) OTP value if applicable.
 */
#define STM32_SMC_BSEC			0x82001003

/* Low Power services */

/*
 * SIP function STM32_SMC_PD_DOMAIN.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a2: (index) ID of target power domain to be enabled/disabled.
 * Argument a3: (input) 0 to disable, 1 to enable target domain.
 */
#define STM32_SMC_PD_DOMAIN		0x82001008

/*
 * SIP function STM32_SMC_RCC_OPP.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a1: (input) Service ID (STM32_SMC_RCC_OPP_xxx).
 *		(output) Rounded frequency, if applicable.
 * Argument a2: (input) Requested frequency.
 */
#define STM32_SMC_RCC_OPP		0x82001009

/*
 * SIP function STM32_SIP_SVC_FUNC_SCMI_AGENT0/1
 *
 * Process SCMI message pending in SCMI shared memory buffer
 * related to SCMI agent IDs 0 and 1. No input or output arguments
 * passed through CPU general purpose registers, messages are transfer
 * through a dedicated area in SYSRAM, mapped as device memory.
 */
#define STM32_SMC_SCMI_MESSAGE_AGENT0	0x82002000
#define STM32_SMC_SCMI_MESSAGE_AGENT1	0x82002001

/* SMC function IDs for SiP Service queries */

/*
 * SIP function STM32_SIP_SVC_CALL_COUNT.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Dummy value 0.
 */
#define STM32_SIP_SVC_CALL_COUNT	0x8200ff00

/*
 * SIP function STM32_SIP_SVC_UID.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Lowest 32bit of the stm32mp1 SIP service UUID.
 * Argument a1: (output) Next 32bit of the stm32mp1 SIP service UUID.
 * Argument a2: (output) Next 32bit of the stm32mp1 SIP service UUID.
 * Argument a3: (output) Last 32bit of the stm32mp1 SIP service UUID.
 */
#define STM32_SIP_SVC_UID		0x8200ff01

/* 0x8200ff02 is reserved */

/*
 * SIP function STM32_SIP_SVC_VERSION.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) STM32 SIP service major.
 * Argument a1: (output) STM32 SIP service minor.
 */
#define STM32_SIP_SVC_VERSION		0x8200ff03

/* Service ID for STM32_SMC_RCC/_PWR */
#define STM32_SMC_REG_READ		0x0
#define STM32_SMC_REG_WRITE		0x1
#define STM32_SMC_REG_SET		0x2
#define STM32_SMC_REG_CLEAR		0x3

/* Service ID for STM32_SMC_BSEC */
#define STM32_SMC_READ_SHADOW		0x01
#define STM32_SMC_PROG_OTP		0x02
#define STM32_SMC_WRITE_SHADOW		0x03
#define STM32_SMC_READ_OTP		0x04
#define STM32_SMC_READ_ALL		0x05
#define STM32_SMC_WRITE_ALL		0x06
#define STM32_SMC_WRLOCK_OTP		0x07

/* Service ID for STM32_SMC_RCC_OPP */
#define STM32_SMC_RCC_OPP_SET		0x0
#define STM32_SMC_RCC_OPP_ROUND		0x1

/* STM32 SiP Service Calls version numbers */
#define STM32_SIP_SVC_VERSION_MAJOR	0x0
#define STM32_SIP_SVC_VERSION_MINOR	0x1

/* Number of STM32 SiP Calls implemented */
#define STM32_COMMON_SIP_NUM_CALLS	13

#endif /* STM32MP1_SMC_H */
