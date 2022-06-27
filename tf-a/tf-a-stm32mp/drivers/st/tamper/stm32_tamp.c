/*
 * Copyright (c) 2018-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <libfdt.h>

#include <platform_def.h>

#include <drivers/st/stm32_rng.h>
#include <drivers/st/stm32_rtc.h>
#include <drivers/st/stm32_tamp.h>
#include <lib/mmio.h>

#define DT_TAMP_COMPAT			"st,stm32-tamp"
/* STM32 Registers */
#define STM32_TAMP_CR1			0x00U
#define STM32_TAMP_CR2			0x04U
#define STM32_TAMP_FLTCR		0x0CU
#define STM32_TAMP_ATCR			0x10U
#define STM32_TAMP_ATSEEDR		0x14U
#define STM32_TAMP_ATOR			0x18U
#define STM32_TAMP_SMCR			0x20U
#define STM32_TAMP_IER			0x2CU
#define STM32_TAMP_SR			0x30U
#define STM32_TAMP_SCR			0x3CU
#define STM32_TAMP_COUNTR		0x40U
#define STM32_TAMP_OR			0x50U
#define STM32_TAMP_HWCFGR2		0x3ECU
#define STM32_TAMP_HWCFGR1		0x3F0U
#define STM32_TAMP_VERR			0x3F4U
#define STM32_TAMP_IPIDR		0x3F8U
#define STM32_TAMP_SIDR			0x3FCU

/* STM32_TAMP_FLTCR bit fields */
#define STM32_TAMP_FLTCR_TAMPFREQ	GENMASK(2, 0)
#define STM32_TAMP_FLTCR_TAMPFLT	GENMASK(4, 3)
#define STM32_TAMP_FLTCR_TAMPPRCH	GENMASK(6, 5)
#define STM32_TAMP_FLTCR_TAMPPUDIS	BIT(7)

/* STM32_TAMP_ATCR bit fields */
#define STM32_TAMP_ATCR_ATCKSEL		GENMASK(18, 16)
#define STM32_TAMP_ATCR_ATPER		GENMASK(26, 24)
#define STM32_TAMP_ATCR_ATOSHARE	BIT(30)
#define STM32_TAMP_ATCR_FLTEN		BIT(31)

/* STM32_TAMP_ATOR bit fields */
#define STM32_TAMP_PRNG			GENMASK(7, 0)
#define STM32_TAMP_SEEDF		BIT(14)
#define STM32_TAMP_INITS		BIT(15)

/* STM32_TAMP_IER bit fields */
#define STM32_TAMP_IER_TAMPXIE_ALL	GENMASK(7, 0)
#define STM32_TAMP_IER_ITAMPXIE_ALL	GENMASK(31, 16)

/* STM32_TAMP_SR bit fields */
#define STM32_TAMP_SR_TAMPXF_MASK	GENMASK(7, 0)
#define STM32_TAMP_SR_ITAMPXF_MASK	GENMASK(31, 16)

/* STM32_TAMP_SMCR but fields */
#define STM32_TAMP_SMCR_DPROT		BIT(31)

/* STM32_TAMP_CFGR bit fields */
#define STM32_TAMP_OR_OUT3RMP		BIT(0)

/* STM32_TAMP_HWCFGR2 bit fields */
#define STM32_TAMP_HWCFGR2_TZ		GENMASK(11, 8)
#define STM32_TAMP_HWCFGR2_OR		GENMASK(7, 0)

/* STM32_TAMP_HWCFGR1 bit fields */
#define STM32_TAMP_HWCFGR1_BKPREG	GENMASK(7, 0)
#define STM32_TAMP_HWCFGR1_TAMPER	GENMASK(11, 8)
#define STM32_TAMP_HWCFGR1_ACTIVE	GENMASK(15, 12)
#define STM32_TAMP_HWCFGR1_INTERN	GENMASK(31, 16)

/* STM32_TAMP_VERR bit fields */
#define STM32_TAMP_VERR_MINREV		GENMASK(3, 0)
#define STM32_TAMP_VERR_MAJREV		GENMASK(7, 4)

#define STM32_TAMP_MAX_INTERNAL		16U
#define STM32_TAMP_MAX_EXTERNAL		8U

struct stm32_tamp_instance {
	uintptr_t base;
	uint32_t clock;
	uint32_t hwconf1;
	uint32_t hwconf2;
	uint16_t int_nbtamp;
	uint8_t ext_nbtamp;
	struct stm32_tamp_int *int_tamp;
	struct stm32_tamp_ext *ext_tamp;
};

static struct stm32_tamp_instance stm32_tamp;

static void stm32_tamp_set_secured(unsigned long base)
{
	mmio_clrbits_32(base + STM32_TAMP_SMCR, STM32_TAMP_SMCR_DPROT);
}

static void stm32_tamp_configure_or(unsigned long base, uint32_t out3)
{
	mmio_setbits_32(base + STM32_TAMP_OR, out3);
}

static int stm32_tamp_seed_init(unsigned long base)
{
	/*  Need RNG access */
	uint32_t timeout = 100;
	uint8_t idx;

	for (idx = 0; idx < 4U; idx++) {
		uint32_t rnd;

		if (stm32_rng_read((uint8_t *)&rnd, sizeof(uint32_t)) != 0) {
			return -1;
		}

		VERBOSE("Seed init %u\n", rnd);
		mmio_write_32(base + STM32_TAMP_ATSEEDR, rnd);
	}

	while (((mmio_read_32(base + STM32_TAMP_ATOR) &
		 STM32_TAMP_SEEDF) != 0U) &&
	       (timeout != 0U)) {
		timeout--;
	}

	if (timeout == 0U) {
		return -1;
	}

	return 0;
}

static void stm32_tamp_reset_register(unsigned long base)
{
	/* Disable all internal tamper */
	mmio_write_32(base + STM32_TAMP_CR1, 0U);

	/* Disable all external tamper */
	mmio_write_32(base + STM32_TAMP_CR2, 0U);

	/* Clean configuration registers */
	mmio_write_32(base + STM32_TAMP_FLTCR, 0U);
	mmio_write_32(base + STM32_TAMP_ATCR, 0U);
	mmio_clrbits_32(base + STM32_TAMP_SMCR, STM32_TAMP_SMCR_DPROT);

	/* Clean Tamper IT */
	mmio_write_32(base + STM32_TAMP_IER, 0U);
	mmio_write_32(base + STM32_TAMP_SCR, ~0U);

	mmio_clrbits_32(base + STM32_TAMP_OR, STM32_TAMP_OR_OUT3RMP);
}

void stm32_tamp_write_mcounter(void)
{
	mmio_write_32(stm32_tamp.base + STM32_TAMP_COUNTR, 1U);
}

void stm32_tamp_configure_internal(struct stm32_tamp_int *tamper_list,
				   uint16_t nb_tamper)
{
	uint16_t i;

	assert(nb_tamper < STM32_TAMP_MAX_INTERNAL);

	for (i = 0; i < nb_tamper; i++) {
		int id = tamper_list[i].id;
		uint32_t u_id;

		if (id == -1) {
			continue;
		}

		u_id = (uint32_t)id;

		if ((stm32_tamp.hwconf1 & BIT(u_id + 16U)) != 0U) {
			mmio_setbits_32(stm32_tamp.base + STM32_TAMP_CR1,
					BIT(u_id + 16U));
			mmio_setbits_32(stm32_tamp.base + STM32_TAMP_IER,
					BIT(u_id + 16U));
		}
	}

	stm32_tamp.int_tamp = tamper_list;
	stm32_tamp.int_nbtamp = nb_tamper;
}

void stm32_tamp_configure_external(struct stm32_tamp_ext *ext_tamper_list,
				   uint8_t nb_tamper, uint32_t passive_conf,
				   uint32_t active_conf)
{
	/* External configuration */
	uint8_t i, active_tamp = 0;

	assert(nb_tamper < STM32_TAMP_MAX_EXTERNAL);

	/*  Enable external Tamp */
	for (i = 0; i < nb_tamper; i++) {
		int id = ext_tamper_list[i].id;
		uint32_t reg = 0, u_id;

		if (id == -1) {
			continue;
		}

		u_id = (uint32_t)id;

		mmio_setbits_32(stm32_tamp.base + STM32_TAMP_CR1,
				BIT(u_id));

		if (ext_tamper_list[i].mode == TAMP_TRIG_ON) {
			reg |= BIT(u_id + 24U);
		}

		if (ext_tamper_list[i].mode == TAMP_ACTIVE) {
			active_tamp |= BIT(u_id);
		}

		if (ext_tamper_list[i].erase != 0U) {
			reg |= BIT(u_id);
		}

		if (ext_tamper_list[i].evt_mask != 0U) {
			reg |= BIT(u_id + 16U);
		} else {
			mmio_setbits_32(stm32_tamp.base + STM32_TAMP_IER,
					BIT(u_id));
		}

		mmio_setbits_32(stm32_tamp.base + STM32_TAMP_CR2, reg);
	}

	/*  Filter mode register set */
	mmio_write_32(stm32_tamp.base + STM32_TAMP_FLTCR, passive_conf);

	/*  Active mode configuration */
	if (active_tamp != 0U) {
		mmio_write_32(stm32_tamp.base + STM32_TAMP_ATCR,
			      active_conf | active_tamp);
		if (stm32_tamp_seed_init(stm32_tamp.base) != 0) {
			ERROR("Active tamper: SEED not initialized\n");
			panic();
		}
	}

	stm32_tamp.ext_tamp = ext_tamper_list;
	stm32_tamp.ext_nbtamp = nb_tamper;
}

void stm32_tamp_it_handler(void)
{
	uint32_t it = mmio_read_32(stm32_tamp.base + STM32_TAMP_SR);
	uint8_t tamp = 0;
	struct stm32_rtc_time tamp_ts;
	struct stm32_tamp_int *int_list = stm32_tamp.int_tamp;
	struct stm32_tamp_ext *ext_list = stm32_tamp.ext_tamp;

	if (stm32_rtc_is_timestamp_enable()) {
		stm32_rtc_get_timestamp(&tamp_ts);
		INFO("Tamper Event Occurred\n");
		INFO("Date : %u/%u\n \t Time : %u:%u:%u\n",
		     tamp_ts.day, tamp_ts.month, tamp_ts.hour,
		     tamp_ts.min, tamp_ts.sec);
	}

	/*  Internal tamper interrupt */
	if ((it & STM32_TAMP_IER_ITAMPXIE_ALL) == 0U) {
		goto tamp_ext;
	}

	while ((it != 0U) && (tamp < stm32_tamp.int_nbtamp)) {
		uint32_t int_id = (uint32_t)int_list[tamp].id;

		if ((it & BIT(int_id + 16U)) != 0U) {
			if (int_list[tamp].func != NULL) {
				int_list[tamp].func(int_id);
			}

			mmio_setbits_32(stm32_tamp.base + STM32_TAMP_SCR,
					BIT(int_id + 16U));
			it &= ~BIT(int_id + 16U);
		}
		tamp++;
	}

tamp_ext:
	tamp = 0;
	/* External tamper interrupt */
	if ((it == 0U) || ((it & STM32_TAMP_IER_TAMPXIE_ALL) == 0U)) {
		return;
	}

	while ((it != 0U) && (tamp < stm32_tamp.ext_nbtamp)) {
		uint32_t ext_id = (uint32_t)ext_list[tamp].id;

		if ((it & BIT(ext_id)) != 0U) {
			if (ext_list[tamp].func != NULL) {
				ext_list[tamp].func(ext_id);
			}

			mmio_setbits_32(stm32_tamp.base + STM32_TAMP_SCR,
					BIT(ext_id));
			it &= ~BIT(ext_id);
		}
		tamp++;
	}
}

int stm32_tamp_init(void)
{
	int node;
	struct dt_node_info dt_tamp;
	void *fdt;
	uint32_t rev __unused;

	if (fdt_get_address(&fdt) == 0) {
		return -EPERM;
	}

	node = dt_get_node(&dt_tamp, -1, DT_TAMP_COMPAT);
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	assert(dt_tamp.base != 0U);
	assert(dt_tamp.clock != -1);

	stm32_tamp.base = dt_tamp.base;
	stm32_tamp.clock = (uint32_t)dt_tamp.clock;

	/* Init Tamp clock */
	stm32mp_clk_enable(stm32_tamp.clock);

	/* Reset Tamp register without modifying backup registers conf */
	stm32_tamp_reset_register(stm32_tamp.base);

	/* Check if TAMP is enabled */
	if ((dt_tamp.status != DT_SECURE) &&
	    (dt_tamp.status != DT_SHARED)) {
		return 0;
	}

	stm32_tamp.hwconf1 = mmio_read_32(stm32_tamp.base + STM32_TAMP_HWCFGR1);
	stm32_tamp.hwconf2 = mmio_read_32(stm32_tamp.base + STM32_TAMP_HWCFGR2);

	rev = mmio_read_32(stm32_tamp.base + STM32_TAMP_VERR);
	VERBOSE("STM32 TAMPER V%u.%u\n", (rev & STM32_TAMP_VERR_MAJREV) >> 4,
		rev & STM32_TAMP_VERR_MINREV);

	if ((stm32_tamp.hwconf2 & STM32_TAMP_HWCFGR2_TZ) == 0U) {
		ERROR("Tamper IP doesn't support trustzone");
		return -EPERM;
	}

	stm32_tamp_set_secured(stm32_tamp.base);

	if (fdt_getprop(fdt, node, "st,out3-pc13", NULL) != NULL) {
		stm32_tamp_configure_or(stm32_tamp.base, 1);
	}

	if (stm32_gic_enable_spi(node, NULL) < 0) {
		panic();
	}

	if (fdt_getprop(fdt, node, "wakeup-source", NULL) != NULL) {
		mmio_setbits_32(EXTI_BASE + EXTI_TZENR1, EXTI_TZENR1_TZEN18);
		mmio_setbits_32(EXTI_BASE + EXTI_C1IMR1, EXTI_IMR1_IM18);
	}

	return 1;
}
