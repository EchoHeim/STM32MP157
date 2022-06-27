/*
 * Copyright (C) 2018-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_SHRES_HELPERS_H
#define STM32MP_SHRES_HELPERS_H

#include <stdint.h>

#include <common/debug.h>

/*
 * Lock/unlock access to shared registers
 *
 * @lock - NULL or pointer to spin lock
 */

void stm32mp_lock_shregs(void);
void stm32mp_unlock_shregs(void);
void stm32mp_mmio_clrsetbits_32_shregs(uintptr_t addr, uint32_t clear,
				       uint32_t set);
void stm32mp_mmio_clrbits_32_shregs(uintptr_t addr, uint32_t clear);
void stm32mp_mmio_setbits_32_shregs(uintptr_t addr, uint32_t set);

#endif /* STM32MP_SHRES_HELPERS_H */
