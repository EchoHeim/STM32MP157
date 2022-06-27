/*
 * Copyright (C) 2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <drivers/st/stm32mp_dummy_regulator.h>

static int dummy_regulator_enable(struct stm32mp_regulator *regu)
{
	return 0;
}

static int dummy_regulator_disable(struct stm32mp_regulator *regu)
{
	return 0;
}

static const struct stm32mp_regulator_ops dummy_regu_ops = {
	.enable = dummy_regulator_enable,
	.disable = dummy_regulator_disable,
};

void bind_dummy_regulator(struct stm32mp_regulator *regu)
{
	regu->ops = &dummy_regu_ops;
}
