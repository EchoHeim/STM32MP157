/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_LOW_POWER_H
#define STM32MP1_LOW_POWER_H

#include <stdbool.h>
#include <stdint.h>

void stm32_rcc_wakeup_update(bool state);
void stm32_apply_pmic_suspend_config(uint32_t mode);
void stm32_exit_cstop(void);
void stm32_pwr_down_wfi(void);
void stm32_enter_low_power(uint32_t mode, uint32_t nsec_addr);

#endif /* STM32MP1_LOW_POWER_H */
