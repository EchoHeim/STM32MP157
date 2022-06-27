/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_CLKFUNC_H
#define STM32MP_CLKFUNC_H

#include <stdbool.h>

#include <libfdt.h>

#include <platform_def.h>

int fdt_osc_read_freq(const char *name, uint32_t *freq);
bool fdt_osc_read_bool(enum stm32mp_osc_id osc_id, const char *prop_name);
uint32_t fdt_osc_read_uint32_default(enum stm32mp_osc_id osc_id,
				     const char *prop_name,
				     uint32_t dflt_value);

int fdt_get_rcc_node(void);
int fdt_rcc_read_uint32_array(const char *prop_name,
			      uint32_t *array, uint32_t count);
uint32_t fdt_rcc_read_uint32_default(const char *prop_name,
				     uint32_t dflt_value);
int fdt_rcc_subnode_offset(const char *name);
const fdt32_t *fdt_rcc_read_prop(const char *prop_name, int *lenp);
bool fdt_get_rcc_secure_status(void);
int fdt_rcc_enable_it(const char *name);

int fdt_get_clock_id(int node);
int fdt_get_clock_id_by_name(int node, const char *name);
unsigned long fdt_get_uart_clock_freq(uintptr_t instance);

bool fdt_is_pll1_predefined(void);

void stm32mp_stgen_config(unsigned long rate);
void stm32mp_stgen_restore_counter(unsigned long long value,
				   unsigned long long offset_in_ms);
unsigned long long stm32mp_stgen_get_counter(void);

#endif /* STM32MP_CLKFUNC_H */
