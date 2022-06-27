/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <smccc_helpers.h>

#include <stm32mp1_power_config.h>
#include <stm32mp1_smc.h>

#include "low_power_svc.h"

uint32_t pm_domain_scv_handler(uint32_t x1, uint32_t x2)
{
	if (stm32mp1_set_pm_domain_state((enum stm32mp1_pm_domain)x1,
					 (bool)x2) < 0) {
		return STM32_SMC_FAILED;
	}

	return STM32_SMC_OK;
}
