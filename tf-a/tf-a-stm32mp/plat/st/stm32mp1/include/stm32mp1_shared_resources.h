/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_SHARED_RESOURCES_H
#define STM32MP1_SHARED_RESOURCES_H

#include <stdbool.h>

#include <common/debug.h>
#include <drivers/st/etzpc.h>

#define STM32MP1_SHRES_GPIOZ(i)		(STM32MP1_SHRES_GPIOZ_0 + (i))

enum stm32mp_shres {
	STM32MP1_SHRES_CRYP1,
	STM32MP1_SHRES_GPIOZ_0,
	STM32MP1_SHRES_GPIOZ_1,
	STM32MP1_SHRES_GPIOZ_2,
	STM32MP1_SHRES_GPIOZ_3,
	STM32MP1_SHRES_GPIOZ_4,
	STM32MP1_SHRES_GPIOZ_5,
	STM32MP1_SHRES_GPIOZ_6,
	STM32MP1_SHRES_GPIOZ_7,
	STM32MP1_SHRES_HASH1,
	STM32MP1_SHRES_I2C4,
	STM32MP1_SHRES_I2C6,
	STM32MP1_SHRES_IWDG1,
	STM32MP1_SHRES_MCU,
	STM32MP1_SHRES_MDMA,
	STM32MP1_SHRES_RNG1,
	STM32MP1_SHRES_RTC,
	STM32MP1_SHRES_SPI6,
	STM32MP1_SHRES_USART1,
	STM32MP1_SHRES_PLL3,

	STM32MP1_SHRES_COUNT
};

#ifdef IMAGE_BL32
/* Register a peripheral as secure or non-secure based on identifier */
void stm32mp_register_secure_periph(unsigned int id);
void stm32mp_register_non_secure_periph(unsigned int id);

/* Register a peripheral as secure or non-secure based on IO base address */
void stm32mp_register_secure_periph_iomem(uintptr_t base);
void stm32mp_register_non_secure_periph_iomem(uintptr_t base);

/* Register a GPIO as secure or non-secure based on its bank and pin numbers */
void stm32mp_register_secure_gpio(unsigned int bank, unsigned int pin);
void stm32mp_register_non_secure_gpio(unsigned int bank, unsigned int pin);

/*
 * Register a (non-)secure peripheral based on the ETZPC DECPROT configuration
 */
void stm32mp1_register_etzpc_decprot(unsigned int id,
				     enum etzpc_decprot_attributes attr);

/* Consolidate peripheral states and locknew periph registering */
void stm32mp_lock_periph_registering(void);

/* Get peripheral state in shared resource driver */
bool stm32mp1_periph_is_non_secure(unsigned long id);
bool stm32mp1_periph_is_secure(unsigned long id);

bool stm32mp_gpio_bank_is_non_secure(unsigned int bank);
bool stm32mp_gpio_bank_is_shared(unsigned int bank);

bool stm32mp_nsec_can_access_clock(unsigned long clock_id);
bool stm32mp_nsec_can_access_reset(unsigned int reset_id);
#else /* IMAGE_BL32 */
static inline void stm32mp_register_secure_periph(unsigned int id)
{
}

static inline void stm32mp_register_non_secure_periph(unsigned int id)
{
}

static inline void stm32mp_register_secure_periph_iomem(uintptr_t base)
{
}

static inline void stm32mp_register_non_secure_periph_iomem(uintptr_t base)
{
}

static inline void stm32mp_register_secure_gpio(unsigned int bank,
						unsigned int pin)
{
}

static inline void stm32mp_register_non_secure_gpio(unsigned int bank,
						    unsigned int pin)
{
}

static inline void stm32mp1_register_etzpc_decprot(unsigned int id,
					enum etzpc_decprot_attributes attr)
{
}

static inline void stm32mp_lock_periph_registering(void)
{
	/* Platform is not expected to lock a non existing database */
	panic();
}

static inline bool stm32mp1_periph_is_non_secure(unsigned long id)
{
	return false;
}

static inline bool stm32mp1_periph_is_secure(unsigned long id)
{
	return true;
}

static inline bool stm32mp_gpio_bank_is_non_secure(unsigned int bank)
{
	return false;
}

static inline bool stm32mp_gpio_bank_is_shared(unsigned int bank)
{
	return false;
}
#endif /* IMAGE_BL32 */

#endif /* STM32MP1_SHARED_RESOURCES_H */
