/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_POWER_CONFIG_H
#define STM32MP1_POWER_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define PSCI_MODE_SYSTEM_SUSPEND	0
#define PSCI_MODE_SYSTEM_OFF		1

enum stm32mp1_pm_domain {
	STM32MP1_PD_VSW,
	STM32MP1_PD_CORE_RET,
	STM32MP1_PD_CORE,
	STM32MP1_PD_MAX_PM_DOMAIN
};

void stm32mp1_init_lp_states(void);
int stm32mp1_set_pm_domain_state(enum stm32mp1_pm_domain domain, bool status);
uint32_t stm32mp1_get_lp_soc_mode(uint32_t psci_mode);
int stm32mp1_set_lp_deepest_soc_mode(uint32_t psci_mode, uint32_t soc_mode);

#endif /* STM32MP1_POWER_CONFIG_H */
