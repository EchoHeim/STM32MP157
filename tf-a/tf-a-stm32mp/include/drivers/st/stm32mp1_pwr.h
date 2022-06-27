/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_PWR_H
#define STM32MP1_PWR_H

#include <lib/utils_def.h>

#define PWR_CR1			U(0x00)
#define PWR_CR2			U(0x08)
#define PWR_CR3			U(0x0C)
#define PWR_MPUCR		U(0x10)
#define PWR_MCUCR		U(0x14)
#define PWR_WKUPCR		U(0x20)
#define PWR_MPUWKUPENR		U(0x28)

#define PWR_OFFSET_MASK		GENMASK(9, 0)

#define PWR_CR1_LPDS		BIT(0)
#define PWR_CR1_LPCFG		BIT(1)
#define PWR_CR1_LVDS		BIT(2)
#define PWR_CR1_DBP		BIT(8)

#define PWR_CR2_BREN		BIT(0)
#define PWR_CR2_RREN		BIT(1)
#define PWR_CR2_BRRDY		BIT(16)
#define PWR_CR2_RRRDY		BIT(17)

#define PWR_CR3_VBE		BIT(8)
#define PWR_CR3_VBRS		BIT(9)
#define PWR_CR3_DDRSREN		BIT(10)
#define PWR_CR3_DDRSRDIS	BIT(11)
#define PWR_CR3_DDRRETEN	BIT(12)
#define PWR_CR3_USB33DEN	BIT(24)
#define PWR_CR3_REG18EN		BIT(28)
#define PWR_CR3_REG11EN		BIT(30)

#define PWR_MPUCR_PDDS		BIT(0)
#define PWR_MPUCR_CSTDBYDIS	BIT(3)
#define PWR_MPUCR_CSSF		BIT(9)

#define PWR_MCUCR_PDDS		BIT(0)

#define PWR_WKUPCR_MASK		GENMASK(27, 16) | GENMASK(13, 8) | GENMASK(5, 0)

#define PWR_MPUWKUPENR_MASK	GENMASK(5, 0)

#endif /* STM32MP1_PWR_H */
