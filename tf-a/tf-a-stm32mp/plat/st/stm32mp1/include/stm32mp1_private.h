/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_PRIVATE_H
#define STM32MP1_PRIVATE_H

#include <stdint.h>

#include <drivers/st/etzpc.h>

enum boot_device_e {
	BOOT_DEVICE_USB,
	BOOT_DEVICE_BOARD
};

void configure_mmu(void);

void stm32mp_mask_timer(void);
void __dead2 stm32mp_wait_cpu_reset(void);

void stm32mp1_arch_security_setup(void);
void stm32mp1_security_setup(void);

enum boot_device_e get_boot_device(void);

enum etzpc_decprot_attributes stm32mp_etzpc_binding2decprot(uint32_t mode);

void stm32mp1_syscfg_init(void);
void stm32mp1_syscfg_enable_io_compensation(void);
void stm32mp1_syscfg_disable_io_compensation(void);

void stm32mp1_init_scmi_server(void);
void stm32mp1_pm_save_scmi_state(uint8_t *state, size_t size);
void stm32mp1_pm_restore_scmi_state(uint8_t *state, size_t size);

#endif /* STM32MP1_PRIVATE_H */
