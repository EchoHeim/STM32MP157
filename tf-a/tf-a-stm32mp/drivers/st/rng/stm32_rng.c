/*
 * Copyright (c) 2018-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>

#include <libfdt.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <drivers/delay_timer.h>
#include <drivers/st/stm32_rng.h>
#include <drivers/st/stm32mp_reset.h>
#include <lib/mmio.h>

#define DT_RNG_COMPAT		"st,stm32-rng"
#define RNG_CR			0x00U
#define RNG_SR			0x04U
#define RNG_DR			0x08U

#define RNG_CR_RNGEN		BIT(2)
#define RNG_CR_IE		BIT(3)
#define RNG_CR_CED		BIT(5)

#define RNG_SR_DRDY		BIT(0)
#define RNG_SR_CECS		BIT(1)
#define RNG_SR_SECS		BIT(2)
#define RNG_SR_CEIS		BIT(5)
#define RNG_SR_SEIS		BIT(6)

#define RNG_TIMEOUT_US		100000
#define RNG_TIMEOUT_STEP_US	10

#define TIMEOUT_US_1MS		U(1000)

struct stm32_rng_instance {
	uintptr_t base;
	unsigned long clock;
};

static struct stm32_rng_instance stm32_rng;

/*
 * stm32_rng_read - Read a number of random bytes from RNG
 * out: pointer to the output buffer
 * size: number of bytes to be read
 * Return 0 on success, non-0 on failure
 */
int stm32_rng_read(uint8_t *out, uint32_t size)
{
	uint8_t *buf = out;
	uint32_t len = size;
	int nb_tries;
	uint32_t data32;
	int rc = 0;
	int count;

	if (stm32_rng.base == 0U) {
		return -EPERM;
	}

	stm32mp_clk_enable(stm32_rng.clock);

	if ((mmio_read_32(stm32_rng.base + RNG_CR) & RNG_CR_RNGEN) == 0U) {
		mmio_write_32(stm32_rng.base + RNG_CR,
			      RNG_CR_RNGEN | RNG_CR_CED);
	}

	while (len != 0U) {
		nb_tries = RNG_TIMEOUT_US / RNG_TIMEOUT_STEP_US;
		do {
			uint32_t status = mmio_read_32(stm32_rng.base + RNG_SR);

			if ((status & (RNG_SR_SECS | RNG_SR_SEIS)) != 0U) {
				uint8_t i;

				/* Recommended by the SoC reference manual */
				mmio_clrbits_32(stm32_rng.base + RNG_SR,
						RNG_SR_SEIS);
				dmb();
				for (i = 12; i != 0; i--) {
					(void)mmio_read_32(stm32_rng.base +
							   RNG_DR);
				}
				dmb();

				if ((mmio_read_32(stm32_rng.base + RNG_SR) &
				     RNG_SR_SEIS) != 0U) {
					ERROR("RNG noise\n");
					panic();
				}
			}

			udelay(RNG_TIMEOUT_STEP_US);
			nb_tries--;
			if (nb_tries == 0) {
				rc = -ETIMEDOUT;
				goto bail;
			}
		} while ((mmio_read_32(stm32_rng.base + RNG_SR) &
			  RNG_SR_DRDY) == 0U);

		count = 4;
		while (len != 0U) {
			data32 = mmio_read_32(stm32_rng.base + RNG_DR);
			count--;

			memcpy(buf, &data32, MIN(len, sizeof(uint32_t)));
			buf += MIN(len, sizeof(uint32_t));
			len -= MIN(len, sizeof(uint32_t));

			if (count == 0) {
				break;
			}
		}
	}

bail:
	stm32mp_clk_disable(stm32_rng.clock);

	if (rc != 0) {
		memset(out, 0, buf - out);
	}

	return rc;
}

/*
 * stm32_rng_init: Initialize rng from DT
 * return 0 on success, negative value on failure
 */
int stm32_rng_init(void)
{
	void *fdt;
	struct dt_node_info dt_rng;
	int node;

	if (fdt_get_address(&fdt) == 0) {
		panic();
	}

	node = dt_get_node(&dt_rng, -1, DT_RNG_COMPAT);
	if (node < 0) {
		return 0;
	}

	if ((dt_rng.status & DT_SECURE) == 0) {
		return 0;
	}

	assert(dt_rng.base == RNG1_BASE);

	stm32_rng.base = dt_rng.base;

	if ((dt_rng.status & DT_NON_SECURE) == DT_NON_SECURE) {
		stm32mp_register_non_secure_periph_iomem(stm32_rng.base);
	} else {
		stm32mp_register_secure_periph_iomem(stm32_rng.base);
	}

	if (dt_rng.clock < 0) {
		panic();
	}
	stm32_rng.clock = (unsigned long)dt_rng.clock;

	stm32mp_clk_enable(stm32_rng.clock);
	stm32mp_clk_disable(stm32_rng.clock);

	if (dt_rng.reset >= 0) {
		if (stm32mp_reset_assert_to((unsigned long)dt_rng.reset,
					    TIMEOUT_US_1MS))
			panic();

		udelay(20);
		if (stm32mp_reset_deassert_to((unsigned long)dt_rng.reset,
					      TIMEOUT_US_1MS))
			panic();
	}

	VERBOSE("Init RNG done\n");

	return 0;
}
