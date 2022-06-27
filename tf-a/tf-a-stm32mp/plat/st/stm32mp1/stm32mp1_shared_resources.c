/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include <platform_def.h>

#include <arch.h>
#include <common/debug.h>
#include <drivers/st/stm32_gpio.h>

#include <stm32mp_dt.h>
#include <stm32mp_shres_helpers.h>
#include <stm32mp1_shared_resources.h>

static bool registering_locked;
static int8_t gpioz_nbpin = -1;

/*
 * Shared peripherals and resources.
 * Defines resource that may be non secure, secure or shared.
 * May be a device, a bus, a clock, a memory.
 * Shared peripherals and resources registration
 *
 * Each resource assignation is stored in a table. The state defaults
 * to SHRES_UNREGISTERED if the resource is not explicitly assigned.
 *
 * Each IO of the GPIOZ IO can be secure or non-secure.
 */
#define SHRES_NON_SECURE		2
#define SHRES_SECURE			1
#define SHRES_UNREGISTERED		0

static uint8_t shres_state[STM32MP1_SHRES_COUNT];

static const char *shres2str_id_tbl[STM32MP1_SHRES_COUNT] = {
	[STM32MP1_SHRES_GPIOZ(0)] = "GPIOZ0",
	[STM32MP1_SHRES_GPIOZ(1)] = "GPIOZ1",
	[STM32MP1_SHRES_GPIOZ(2)] = "GPIOZ2",
	[STM32MP1_SHRES_GPIOZ(3)] = "GPIOZ3",
	[STM32MP1_SHRES_GPIOZ(4)] = "GPIOZ4",
	[STM32MP1_SHRES_GPIOZ(5)] = "GPIOZ5",
	[STM32MP1_SHRES_GPIOZ(6)] = "GPIOZ6",
	[STM32MP1_SHRES_GPIOZ(7)] = "GPIOZ7",
	[STM32MP1_SHRES_IWDG1] = "IWDG1",
	[STM32MP1_SHRES_USART1] = "USART1",
	[STM32MP1_SHRES_SPI6] = "SPI6",
	[STM32MP1_SHRES_I2C4] = "I2C4",
	[STM32MP1_SHRES_RNG1] = "RNG1",
	[STM32MP1_SHRES_HASH1] = "HASH1",
	[STM32MP1_SHRES_CRYP1] = "CRYP1",
	[STM32MP1_SHRES_I2C6] = "I2C6",
	[STM32MP1_SHRES_RTC] = "RTC",
	[STM32MP1_SHRES_MCU] = "MCU",
	[STM32MP1_SHRES_MDMA] = "MDMA",
	[STM32MP1_SHRES_PLL3] = "PLL3",
};

static const char *shres2str_id(unsigned int id)
{
	return shres2str_id_tbl[id];
}

static const char *shres2str_state_tbl[4] = {
	[SHRES_UNREGISTERED] = "unregistered",
	[SHRES_NON_SECURE] = "non-secure",
	[SHRES_SECURE] = "secure",
};

static const char *shres2str_state(unsigned int id)
{
	return shres2str_state_tbl[id];
}

struct shres2decprot {
	unsigned int shres_id;
	unsigned int decprot_id;
	const char *decprot_str;
};

#define SHRES2DECPROT(shres, decprot, str) {	\
		.shres_id = shres,		\
		.decprot_id = decprot,		\
		.decprot_str = str,		\
	}

#define SHRES_INVALID		~0U

static const struct shres2decprot shres2decprot_tbl[] = {
	SHRES2DECPROT(STM32MP1_SHRES_IWDG1, STM32MP1_ETZPC_IWDG1_ID, "IWDG1"),
	SHRES2DECPROT(STM32MP1_SHRES_USART1, STM32MP1_ETZPC_USART1_ID, "UART1"),
	SHRES2DECPROT(STM32MP1_SHRES_SPI6, STM32MP1_ETZPC_SPI6_ID, "SPI6"),
	SHRES2DECPROT(STM32MP1_SHRES_I2C4, STM32MP1_ETZPC_I2C4_ID, "I2C4"),
	SHRES2DECPROT(STM32MP1_SHRES_RNG1, STM32MP1_ETZPC_RNG1_ID, "RNG1"),
	SHRES2DECPROT(STM32MP1_SHRES_HASH1, STM32MP1_ETZPC_HASH1_ID, "HASH1"),
	SHRES2DECPROT(STM32MP1_SHRES_CRYP1, STM32MP1_ETZPC_CRYP1_ID, "CRYP1"),
	SHRES2DECPROT(STM32MP1_SHRES_I2C6, STM32MP1_ETZPC_I2C6_ID, "I2C6"),
	/* Below are specific IDs without a 1-to-1 mapping to SHRES IDs */
	SHRES2DECPROT(SHRES_INVALID, STM32MP1_ETZPC_STGENC_ID, "STGEN"),
	SHRES2DECPROT(SHRES_INVALID, STM32MP1_ETZPC_BKPSRAM_ID, "BKPSRAM"),
	SHRES2DECPROT(SHRES_INVALID, STM32MP1_ETZPC_DDRCTRL_ID, "DDRCTRL"),
	SHRES2DECPROT(SHRES_INVALID, STM32MP1_ETZPC_DDRPHYC_ID, "DDRPHY"),
};

static unsigned int decprot2shres(unsigned int decprot_id)
{
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(shres2decprot_tbl); i++) {
		if (shres2decprot_tbl[i].decprot_id == decprot_id) {
			return shres2decprot_tbl[i].shres_id;
		}
	}

	VERBOSE("No shared resource %u", decprot_id);
	return SHRES_INVALID;
}

static const char *decprot2str(unsigned int decprot_id)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(shres2decprot_tbl); i++) {
		if (shres2decprot_tbl[i].decprot_id == decprot_id) {
			return shres2decprot_tbl[i].decprot_str;
		}
	}

	ERROR("Invalid ID %u", decprot_id);
	panic();
}

static unsigned int get_gpioz_nbpin(void)
{
	if (gpioz_nbpin < 0) {
		gpioz_nbpin = (int8_t)fdt_get_gpioz_nbpins_from_dt();
		assert((gpioz_nbpin == 0) ||
		       (gpioz_nbpin == STM32MP_GPIOZ_PIN_MAX_COUNT));
	}

	return (unsigned int)gpioz_nbpin;
}

static void register_periph(unsigned int id, unsigned int state)
{
	assert((id < STM32MP1_SHRES_COUNT) &&
	       ((state == SHRES_SECURE) || (state == SHRES_NON_SECURE)));

	if (registering_locked) {
		if (shres_state[id] == state) {
			return;
		}

		panic();
	}

	if ((shres_state[id] != SHRES_UNREGISTERED) &&
	    (shres_state[id] != state)) {
		VERBOSE("Cannot change %s from %s to %s\n",
			shres2str_id(id),
			shres2str_state(shres_state[id]),
			shres2str_state(state));
		panic();
	}

	shres_state[id] = (uint8_t)state;

	if (shres_state[id] == SHRES_UNREGISTERED) {
		VERBOSE("Register %s as %s\n",
			shres2str_id(id), shres2str_state(state));
	}

	switch (id) {
	case STM32MP1_SHRES_GPIOZ(0) ... STM32MP1_SHRES_GPIOZ(7):
		if ((id - STM32MP1_SHRES_GPIOZ(0)) >= get_gpioz_nbpin()) {
			ERROR("Invalid GPIO pin %u, %u pin(s) available\n",
			      id - STM32MP1_SHRES_GPIOZ(0),
			      get_gpioz_nbpin());
			panic();
		}
		break;
	default:
		break;
	}

	/* Explore clock tree to lock dependencies */
	if (state == SHRES_SECURE) {
		switch (id) {
		case STM32MP1_SHRES_GPIOZ(0) ... STM32MP1_SHRES_GPIOZ(7):
			stm32mp1_register_clock_parents_secure(GPIOZ);
			break;
		case STM32MP1_SHRES_IWDG1:
			stm32mp1_register_clock_parents_secure(IWDG1);
			break;
		case STM32MP1_SHRES_USART1:
			stm32mp1_register_clock_parents_secure(USART1_K);
			break;
		case STM32MP1_SHRES_SPI6:
			stm32mp1_register_clock_parents_secure(SPI6_K);
			break;
		case STM32MP1_SHRES_I2C4:
			stm32mp1_register_clock_parents_secure(I2C4_K);
			break;
		case STM32MP1_SHRES_RNG1:
			stm32mp1_register_clock_parents_secure(RNG1_K);
			break;
		case STM32MP1_SHRES_HASH1:
			stm32mp1_register_clock_parents_secure(HASH1);
			break;
		case STM32MP1_SHRES_CRYP1:
			stm32mp1_register_clock_parents_secure(CRYP1);
			break;
		case STM32MP1_SHRES_I2C6:
			stm32mp1_register_clock_parents_secure(I2C6_K);
			break;
		case STM32MP1_SHRES_RTC:
			stm32mp1_register_clock_parents_secure(RTC);
			break;
		default:
			/* No expected resource dependency */
			break;
		}
	}
}

static bool stm32mp1_mckprot_resource(unsigned int id)
{
	switch (id) {
	case STM32MP1_SHRES_MCU:
	case STM32MP1_SHRES_PLL3:
		return true;
	default:
		return false;
	}
}

/* Register resource by ID */
void stm32mp_register_secure_periph(unsigned int id)
{
	register_periph(id, SHRES_SECURE);
}

void stm32mp_register_non_secure_periph(unsigned int id)
{
	register_periph(id, SHRES_NON_SECURE);
}

/* Register resource by IO memory base address */
static void register_periph_iomem(uintptr_t base, unsigned int state)
{
	unsigned int id;

	switch (base) {
	case IWDG1_BASE:
		id = STM32MP1_SHRES_IWDG1;
		break;
	case USART1_BASE:
		id = STM32MP1_SHRES_USART1;
		break;
	case SPI6_BASE:
		id = STM32MP1_SHRES_SPI6;
		break;
	case I2C4_BASE:
		id = STM32MP1_SHRES_I2C4;
		break;
	case I2C6_BASE:
		id = STM32MP1_SHRES_I2C6;
		break;
	case RTC_BASE:
		id = STM32MP1_SHRES_RTC;
		break;
	case RNG1_BASE:
		id = STM32MP1_SHRES_RNG1;
		break;
	case CRYP1_BASE:
		id = STM32MP1_SHRES_CRYP1;
		break;
	case HASH1_BASE:
		id = STM32MP1_SHRES_HASH1;
		break;

	case GPIOA_BASE:
	case GPIOB_BASE:
	case GPIOC_BASE:
	case GPIOD_BASE:
	case GPIOE_BASE:
	case GPIOF_BASE:
	case GPIOG_BASE:
	case GPIOH_BASE:
	case GPIOI_BASE:
	case GPIOJ_BASE:
	case GPIOK_BASE:
	case USART2_BASE:
	case USART3_BASE:
	case UART4_BASE:
	case UART5_BASE:
	case USART6_BASE:
	case UART7_BASE:
	case UART8_BASE:
	case IWDG2_BASE:
		/* Allow drivers to register some non-secure resources */
		VERBOSE("IO for non-secure resource 0x%x\n",
			(unsigned int)base);
		if (state != SHRES_NON_SECURE) {
			panic();
		}

		return;

	default:
		panic();
		break;
	}

	register_periph(id, state);
}

void stm32mp_register_secure_periph_iomem(uintptr_t base)
{
	register_periph_iomem(base, SHRES_SECURE);
}

void stm32mp_register_non_secure_periph_iomem(uintptr_t base)
{
	register_periph_iomem(base, SHRES_NON_SECURE);
}

/* Register GPIO resource */
void stm32mp_register_secure_gpio(unsigned int bank, unsigned int pin)
{
	switch (bank) {
	case GPIO_BANK_Z:
		register_periph(STM32MP1_SHRES_GPIOZ(pin), SHRES_SECURE);
		break;
	default:
		ERROR("GPIO bank %u cannot be secured\n", bank);
		panic();
	}
}

void stm32mp_register_non_secure_gpio(unsigned int bank, unsigned int pin)
{
	switch (bank) {
	case GPIO_BANK_Z:
		register_periph(STM32MP1_SHRES_GPIOZ(pin), SHRES_NON_SECURE);
		break;
	default:
		break;
	}
}

void stm32mp1_register_etzpc_decprot(unsigned int id,
				     enum etzpc_decprot_attributes attr)
{
	unsigned int state = SHRES_SECURE;
	unsigned int id_shres;

	switch (attr) {
	case TZPC_DECPROT_S_RW:
		break;
	case TZPC_DECPROT_NS_R_S_W:
	case TZPC_DECPROT_MCU_ISOLATION:
	case TZPC_DECPROT_NS_RW:
		state = SHRES_NON_SECURE;
		break;
	default:
		panic();
	}

	switch (id) {
	case STM32MP1_ETZPC_STGENC_ID:
	case STM32MP1_ETZPC_BKPSRAM_ID:
	case STM32MP1_ETZPC_DDRCTRL_ID:
	case STM32MP1_ETZPC_DDRPHYC_ID:
		/* We assume these must always be assigned to secure world */
		if (state != SHRES_SECURE) {
			panic();
		}
		break;
	default:
		id_shres = decprot2shres(id);
		if (id_shres == SHRES_INVALID) {
			if (state == SHRES_SECURE) {
				panic();
			}
		} else {
			register_periph(id_shres, state);
		}
		break;
	}
}

/* Get resource state: these accesses lock the registering support */
static void lock_registering(void)
{
	registering_locked = true;
}

bool stm32mp1_periph_is_non_secure(unsigned long id)
{
	lock_registering();

	/* Resource not registered are assumed non-secure */
	return (shres_state[id] == SHRES_NON_SECURE) ||
	       (shres_state[id] == SHRES_UNREGISTERED);
}

bool stm32mp1_periph_is_secure(unsigned long id)
{
	lock_registering();

	return shres_state[id] == SHRES_SECURE;
}

bool stm32mp_gpio_bank_is_shared(unsigned int bank)
{
	unsigned int non_secure = 0;
	unsigned int i;

	lock_registering();

	if (bank != GPIO_BANK_Z) {
		return false;
	}

	for (i = 0U; i < get_gpioz_nbpin(); i++) {
		if (!stm32mp1_periph_is_secure(STM32MP1_SHRES_GPIOZ(i))) {
			non_secure++;
		}
	}

	return (non_secure != 0) && (non_secure < get_gpioz_nbpin());
}

bool stm32mp_gpio_bank_is_non_secure(unsigned int bank)
{
	unsigned int non_secure = 0;
	unsigned int i;

	lock_registering();

	if (bank != GPIO_BANK_Z) {
		return true;
	}

	for (i = 0U; i < get_gpioz_nbpin(); i++) {
		if (!stm32mp1_periph_is_secure(STM32MP1_SHRES_GPIOZ(i))) {
			non_secure++;
		}
	}

	return non_secure == get_gpioz_nbpin();
}

static bool stm32mp_gpio_bank_is_secure(unsigned int bank)
{
	unsigned int secure = 0;
	unsigned int i;

	lock_registering();

	if (bank != GPIO_BANK_Z) {
		return false;
	}

	for (i = 0U; i < get_gpioz_nbpin(); i++) {
		if (stm32mp1_periph_is_secure(STM32MP1_SHRES_GPIOZ(i))) {
			secure++;
		}
	}

	return secure == get_gpioz_nbpin();
}

CASSERT((CK_HSE == 0) &&
	((CK_HSE + 1) == CK_CSI) &&
	((CK_HSE + 2) == CK_LSI) &&
	((CK_HSE + 3) == CK_LSE) &&
	((CK_HSE + 4) == CK_HSI) &&
	((CK_HSE + 5) == CK_HSE_DIV2) &&
	((PLL1_P + 1) == PLL1_Q) &&
	((PLL1_P + 2) == PLL1_R) &&
	((PLL1_P + 3) == PLL2_P) &&
	((PLL1_P + 4) == PLL2_Q) &&
	((PLL1_P + 5) == PLL2_R) &&
	((PLL1_P + 6) == PLL3_P) &&
	((PLL1_P + 7) == PLL3_Q) &&
	((PLL1_P + 8) == PLL3_R),
	assert_clock_id_not_as_expected);

bool stm32mp_nsec_can_access_clock(unsigned long clock_id)
{
	enum stm32mp_shres shres_id = STM32MP1_SHRES_COUNT;

	/* Oscillators and PLLs are visible from non-secure world */
	if ((clock_id <= CK_HSE_DIV2) ||
	    ((clock_id >= PLL1_P) && (clock_id <= PLL3_R))) {
		return true;
	}

	switch (clock_id) {
	case BSEC:
	case CK_AXI:
	case CK_MPU:
	case RTCAPB:
		return true;
	case GPIOZ:
		return !stm32mp_gpio_bank_is_secure(GPIO_BANK_Z);
	case SPI6_K:
		shres_id = STM32MP1_SHRES_SPI6;
		break;
	case I2C4_K:
		shres_id = STM32MP1_SHRES_I2C4;
		break;
	case I2C6_K:
		shres_id = STM32MP1_SHRES_I2C6;
		break;
	case USART1_K:
		shres_id = STM32MP1_SHRES_USART1;
		break;
	case IWDG1:
		shres_id = STM32MP1_SHRES_IWDG1;
		break;
	case CRYP1:
		shres_id = STM32MP1_SHRES_CRYP1;
		break;
	case HASH1:
		shres_id = STM32MP1_SHRES_HASH1;
		break;
	case RNG1_K:
		shres_id = STM32MP1_SHRES_RNG1;
		break;
	case RTC:
		shres_id = STM32MP1_SHRES_RTC;
		break;
	case CK_MCU:
		shres_id = STM32MP1_SHRES_MCU;
		break;
	default:
		return false;
	}

	return !stm32mp1_periph_is_secure(shres_id);
}

bool stm32mp_nsec_can_access_reset(unsigned int reset_id)
{
	enum stm32mp_shres shres_id = STM32MP1_SHRES_COUNT;

	switch (reset_id) {
	case GPIOZ_R:
		return stm32mp_gpio_bank_is_non_secure(GPIO_BANK_Z);
	case SPI6_R:
		shres_id = STM32MP1_SHRES_SPI6;
		break;
	case I2C4_R:
		shres_id = STM32MP1_SHRES_I2C4;
		break;
	case I2C6_R:
		shres_id = STM32MP1_SHRES_I2C6;
		break;
	case USART1_R:
		shres_id = STM32MP1_SHRES_USART1;
		break;
	case CRYP1_R:
		shres_id = STM32MP1_SHRES_CRYP1;
		break;
	case HASH1_R:
		shres_id = STM32MP1_SHRES_HASH1;
		break;
	case RNG1_R:
		shres_id = STM32MP1_SHRES_RNG1;
		break;
	case MDMA_R:
		shres_id = STM32MP1_SHRES_MDMA;
		break;
	case MCU_R:
		shres_id = STM32MP1_SHRES_MCU;
		break;
	default:
		return false;
	}

	return !stm32mp1_periph_is_secure(shres_id);
}

/* ETZPC configuration at drivers initialization completion */
static enum etzpc_decprot_attributes decprot_periph_attr(unsigned int id)
{
	switch (id) {
	case STM32MP1_SHRES_GPIOZ(0) ... STM32MP1_SHRES_GPIOZ(7):
		assert((id - STM32MP1_SHRES_GPIOZ(0)) < get_gpioz_nbpin());
		return TZPC_DECPROT_NS_RW;
	default:
		if (!stm32mp1_periph_is_secure(id)) {
			return TZPC_DECPROT_NS_RW;
		}

		return TZPC_DECPROT_S_RW;
	}
}

static bool check_decprot(unsigned int id, enum etzpc_decprot_attributes exp)
{
	enum etzpc_decprot_attributes cur = etzpc_get_decprot(id);

	if (cur == exp) {
		return true;
	}

	switch (exp) {
	case TZPC_DECPROT_NS_RW:
		if (cur == TZPC_DECPROT_S_RW) {
			INFO("ETZPC: %s (%d) could be non secure\n",
			     decprot2str(id), id);
		}
		return true;

	case TZPC_DECPROT_S_RW:
		ERROR("ETZPC: %s (%d) expected secure but DECPROT = %d\n",
		      decprot2str(id), id, cur);
		break;

	case TZPC_DECPROT_NS_R_S_W:
	case TZPC_DECPROT_MCU_ISOLATION:
	default:
		panic();
	}

	return false;
}

static void check_etzpc_secure_configuration(void)
{
	bool error = false;

	assert(registering_locked);

	error |= !check_decprot(STM32MP1_ETZPC_STGENC_ID, TZPC_DECPROT_S_RW);

	error |= !check_decprot(STM32MP1_ETZPC_BKPSRAM_ID, TZPC_DECPROT_S_RW);

	error |= !check_decprot(STM32MP1_ETZPC_USART1_ID,
				decprot_periph_attr(STM32MP1_SHRES_USART1));

	error |= !check_decprot(STM32MP1_ETZPC_SPI6_ID,
				decprot_periph_attr(STM32MP1_SHRES_SPI6));

	error |= !check_decprot(STM32MP1_ETZPC_I2C4_ID,
				decprot_periph_attr(STM32MP1_SHRES_I2C4));

	error |= !check_decprot(STM32MP1_ETZPC_RNG1_ID,
				decprot_periph_attr(STM32MP1_SHRES_RNG1));

	error |= !check_decprot(STM32MP1_ETZPC_HASH1_ID,
				decprot_periph_attr(STM32MP1_SHRES_HASH1));

	error |= !check_decprot(STM32MP1_ETZPC_CRYP1_ID,
				decprot_periph_attr(STM32MP1_SHRES_CRYP1));

	error |= !check_decprot(STM32MP1_ETZPC_DDRCTRL_ID, TZPC_DECPROT_S_RW);

	error |= !check_decprot(STM32MP1_ETZPC_DDRPHYC_ID, TZPC_DECPROT_S_RW);

	error |= !check_decprot(STM32MP1_ETZPC_I2C6_ID,
				decprot_periph_attr(STM32MP1_SHRES_I2C6));

	if (error) {
		panic();
	}
}

static void check_rcc_secure_configuration(void)
{
	uint32_t n;
	uint32_t error = 0;
	bool mckprot = stm32mp1_rcc_is_mckprot();
	bool secure = stm32mp1_rcc_is_secure();

	for (n = 0; n < ARRAY_SIZE(shres_state); n++) {
		if  (shres_state[n] == SHRES_SECURE) {
			if ((stm32mp1_mckprot_resource(n) && (!mckprot)) ||
			    !secure) {
				ERROR("RCC %s MCKPROT %s and %s (%u) secure\n",
				      secure ? "secure" : "non secure",
				      mckprot ? "set" : "not set",
				      shres2str_id(n), n);
				error++;
			}
		}
	}

	if (error != 0U) {
		panic();
	}
}

static void check_gpio_secure_configuration(void)
{
	uint32_t pin;

	for (pin = 0U; pin < get_gpioz_nbpin(); pin++) {
		unsigned int id = STM32MP1_SHRES_GPIOZ(pin);
		bool secure = stm32mp1_periph_is_secure(id);

		set_gpio_secure_cfg(GPIO_BANK_Z, pin, secure);
	}
}

void stm32mp_lock_periph_registering(void)
{
	uint32_t __unused id;

	registering_locked = true;

	for (id = 0; id < STM32MP1_SHRES_COUNT; id++) {
		uint8_t state = shres_state[id];

		assert((state == SHRES_SECURE) ||
		       (state == SHRES_NON_SECURE) ||
		       (state == SHRES_UNREGISTERED));

		if (state == SHRES_SECURE) {
			INFO("stm32mp %s (%u): %s\n",
			     shres2str_id(id), id,
			     state == SHRES_SECURE ? "Secure" :
			     state == SHRES_NON_SECURE ? "Non-secure" :
			     state == SHRES_UNREGISTERED ? "Unregistered" :
			     "<Invalid>");
		}
	}

	stm32mp1_dump_clocks_state();

	check_rcc_secure_configuration();
	check_etzpc_secure_configuration();
	check_gpio_secure_configuration();
}
