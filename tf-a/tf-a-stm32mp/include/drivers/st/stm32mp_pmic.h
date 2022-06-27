/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_PMIC_H
#define STM32MP_PMIC_H

#include <stdbool.h>

#include <platform_def.h>

#include <drivers/st/stm32mp_regulator.h>

/*
 * dt_pmic_status - Check PMIC status from device tree
 *
 * Returns the status of the PMIC (secure, non-secure), or a negative value on
 * error
 */
int dt_pmic_status(void);

/*
 * dt_pmic_configure_boot_on_regulators - Configure boot-on and always-on
 * regulators from device tree configuration
 *
 * Returns 0 on success, and negative values on errors
 */
int pmic_configure_boot_on_regulators(void);

int pmic_set_lp_config(const char *node_name);

/*
 * dt_pmic_find_supply - Find the supply name related to a regulator name
 *
 * Returns 0 on success, and negative values on errors
 */
int dt_pmic_find_supply(const char **supply_name, const char *regu_name);

/*
 * pmic_set_regulator_min_voltage - Set target supply to its device tree
 * "regulator-min-microvolt" value.
 *
 * Returns 0 on success, and negative values on errors
 */
int pmic_set_regulator_min_voltage(const char *regu_name);

/*
 * initialize_pmic_i2c - Initialize I2C for the PMIC control
 *
 * Returns true if PMIC is available, false if not found, panics on errors
 */
bool initialize_pmic_i2c(void);

/*
 * initialize_pmic - Main PMIC initialization function, called at platform init
 *
 * Panics on errors
 */
void initialize_pmic(void);

/*
 * configure_pmic - PMIC configuration function, called at platform init
 *
 * Panics on errors
 */
void configure_pmic(void);

#if DEBUG
void print_pmic_info_and_debug(void);
#else
static inline void print_pmic_info_and_debug(void)
{
}
#endif

bool is_pmic_regulator(struct stm32mp_regulator *regu);
void bind_pmic_regulator(struct stm32mp_regulator *regu);

/*
 * pmic_ddr_power_init - Initialize regulators required for DDR
 *
 * Returns 0 on success, and negative values on errors
 */
int pmic_ddr_power_init(enum ddr_type ddr_type);

#endif /* STM32MP_PMIC_H */
