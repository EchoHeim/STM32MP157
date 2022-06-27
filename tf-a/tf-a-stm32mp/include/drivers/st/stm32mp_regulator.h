/*
 * Copyright (C) 2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_REGULATOR_H
#define STM32MP_REGULATOR_H

#include <stdbool.h>

struct stm32mp_regulator;

struct stm32mp_regulator_ops {
	int (*enable)(struct stm32mp_regulator *regu);
	int (*disable)(struct stm32mp_regulator *regu);
};

struct stm32mp_regulator {
	const struct stm32mp_regulator_ops *ops;
	int id;
	bool always_on;
};

int stm32mp_regulator_enable(struct stm32mp_regulator *regu);
int stm32mp_regulator_disable(struct stm32mp_regulator *regu);
int stm32mp_regulator_register(struct stm32mp_regulator *regu);

int plat_bind_regulator(struct stm32mp_regulator *regu);

#endif /* STM32MP_REGULATOR_H */
