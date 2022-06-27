/*
 * Copyright (C) 2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <drivers/st/stm32mp_regulator.h>

#pragma weak plat_bind_regulator
int plat_bind_regulator(struct stm32mp_regulator *regu)
{
	return -1;
}

int stm32mp_regulator_enable(struct stm32mp_regulator *regu)
{
	assert((regu->ops != NULL) && (regu->ops->enable != NULL));

	return regu->ops->enable(regu);
}

int stm32mp_regulator_disable(struct stm32mp_regulator *regu)
{
	assert((regu->ops != NULL) && (regu->ops->disable != NULL));

	if (regu->always_on) {
		return 0;
	}

	return regu->ops->disable(regu);
}

int stm32mp_regulator_register(struct stm32mp_regulator *regu)
{
	return plat_bind_regulator(regu);
}
