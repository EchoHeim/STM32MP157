/*
 * Copyright (c) 2017-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_CONTEXT_H
#define STM32MP1_CONTEXT_H

#include <stdbool.h>
#include <stdint.h>

#define DDR_CRC_GRANULE		32

void stm32_clean_context(void);
int stm32_save_context(uint32_t zq0cr0_zdata);
int stm32_restore_context(void);
unsigned long long stm32_get_stgen_from_context(void);
int stm32_restore_backup_reg(void);
uint32_t stm32_get_zdata_from_context(void);
int stm32_get_pll1_settings_from_context(void);
bool stm32_are_pll1_settings_valid_in_context(void);
int stm32_save_boot_interface(uint32_t interface, uint32_t instance);
int stm32_get_boot_interface(uint32_t *interface, uint32_t *instance);
void stm32_save_ddr_training_area(void);
void stm32_restore_ddr_training_area(void);
uint32_t stm32_pm_get_optee_ep(void);

void stm32mp1_pm_save_clock_cfg(size_t offset, uint8_t *data, size_t size);
void stm32mp1_pm_restore_clock_cfg(size_t offset, uint8_t *data, size_t size);

#endif /* STM32MP1_CONTEXT_H */
