/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <string.h>

#include <libfdt.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/arm/gicv2.h>
#include <drivers/delay_timer.h>
#include <drivers/st/stm32_iwdg.h>
#include <drivers/st/stm32mp_clkfunc.h>
#include <lib/mmio.h>
#include <lib/utils.h>
#include <plat/common/platform.h>

#define IWDG_TIMEOUT_MS		U(100)

/* IWDG registers offsets */
#define IWDG_KR_OFFSET		0x00U
#define IWDG_PR_OFFSET		0x04U
#define IWDG_RLR_OFFSET		0x08U
#define IWDG_SR_OFFSET		0x0CU
#define IWDG_EWCR_OFFSET	0x14U

/* Registers values */
#define IWDG_KR_ACCESS_KEY	0x5555
#define IWDG_KR_RELOAD_KEY	0xAAAA
#define IWDG_KR_START_KEY	0xCCCC

#define IWDG_PR_DIV_4		0x00
#define IWDG_PR_DIV_256		0x06

#define IWDG_RLR_MAX_VAL	0xFFF

#define IWDG_SR_EWU		BIT(3)

#define IWDG_EWCR_EWIE		BIT(15)
#define IWDG_EWCR_EWIC		BIT(14)
#define IWDG_EWCR_EWIT_MASK	GENMASK(11, 0)

struct stm32_iwdg_instance {
	uintptr_t base;
	unsigned long clock;
	uint8_t flags;
	int num_irq;
};

static struct stm32_iwdg_instance stm32_iwdg[IWDG_MAX_INSTANCE];

static int stm32_iwdg_get_dt_node(struct dt_node_info *info, int offset)
{
	int node;

	node = dt_get_node(info, offset, DT_IWDG_COMPAT);
	if (node < 0) {
		if (offset == -1) {
			VERBOSE("%s: No IDWG found\n", __func__);
		}
		return -FDT_ERR_NOTFOUND;
	}

	return node;
}

#if defined(IMAGE_BL32)
void __dead2 stm32_iwdg_it_handler(int id)
{
	unsigned int cpu = plat_my_core_pos();
	struct stm32_iwdg_instance *iwdg;
	unsigned int instance;

	for (instance = 0; instance < IWDG_MAX_INSTANCE; instance++) {
		if (stm32_iwdg[instance].num_irq == id) {
			break;
		}
	}

	if (instance == IWDG_MAX_INSTANCE) {
		panic();
	}

	iwdg = &stm32_iwdg[instance];

	VERBOSE("CPU %x IT Watchdog %d\n", cpu, instance + 1);

	stm32_iwdg_refresh();

	stm32mp_clk_enable(iwdg->clock);

	mmio_setbits_32(iwdg->base + IWDG_EWCR_OFFSET, IWDG_EWCR_EWIC);

	stm32mp_clk_disable(iwdg->clock);

	/* Ack interrupt as we do not return from next call */
	gicv2_end_of_interrupt(id);

	stm32mp_plat_reset(cpu);
}

static int stm32_iwdg_get_secure_timeout(int node)
{
	void *fdt;
	const fdt32_t *cuint;

	if (fdt_get_address(&fdt) == 0) {
		return -1;
	}

	cuint = fdt_getprop(fdt, node, "secure-timeout-sec", NULL);
	if (cuint == NULL) {
		return -1;
	}

	return (int)fdt32_to_cpu(*cuint);
}

static int stm32_iwdg_conf_etimeout(int node, struct stm32_iwdg_instance *iwdg)
{
	int id_lsi;
	int dt_secure_timeout = stm32_iwdg_get_secure_timeout(node);
	uint32_t reload, status;
	unsigned int timeout = IWDG_TIMEOUT_MS;
	unsigned long long reload_ll;

	if (dt_secure_timeout < 0) {
		return 0;
	}

	if (dt_secure_timeout == 0) {
		return -EINVAL;
	}

	id_lsi = fdt_get_clock_id_by_name(node, "lsi");
	if (id_lsi < 0) {
		return -EINVAL;
	}

	/* Prescaler fix to 256 */
	reload_ll = (unsigned long long)dt_secure_timeout *
		   stm32mp_clk_get_rate(id_lsi);
	reload = ((uint32_t)(reload_ll >> 8) - 1U) & IWDG_EWCR_EWIT_MASK;

	stm32mp_clk_enable(iwdg->clock);

	mmio_write_32(iwdg->base + IWDG_KR_OFFSET, IWDG_KR_START_KEY);
	mmio_write_32(iwdg->base + IWDG_KR_OFFSET, IWDG_KR_ACCESS_KEY);
	mmio_write_32(iwdg->base + IWDG_PR_OFFSET, IWDG_PR_DIV_256);
	mmio_write_32(iwdg->base + IWDG_EWCR_OFFSET, IWDG_EWCR_EWIE | reload);

	do {
		status = mmio_read_32(iwdg->base + IWDG_SR_OFFSET) &
			 IWDG_SR_EWU;
		timeout--;
		mdelay(1);
	} while ((status != 0U) && (timeout != 0U));

	iwdg->num_irq = stm32_gic_enable_spi(node, NULL);
	if (iwdg->num_irq < 0) {
		panic();
	}

	stm32mp_clk_disable(iwdg->clock);

	return (timeout == 0U) ? -ETIMEDOUT : 0;
}
#endif

void stm32_iwdg_refresh(void)
{
	uint8_t i;

	for (i = 0U; i < IWDG_MAX_INSTANCE; i++) {
		struct stm32_iwdg_instance *iwdg = &stm32_iwdg[i];

		/* 0x00000000 is not a valid address for IWDG peripherals */
		if (iwdg->base != 0U) {
			stm32mp_clk_enable(iwdg->clock);

			mmio_write_32(iwdg->base + IWDG_KR_OFFSET,
				      IWDG_KR_RELOAD_KEY);

			stm32mp_clk_disable(iwdg->clock);
		}
	}
}

int stm32_iwdg_init(void)
{
	int node = -1;
	int __unused res;
	struct dt_node_info dt_info;
	void *fdt;
	uint32_t __unused count = 0;

	if (fdt_get_address(&fdt) == 0) {
		panic();
	}

	for (node = stm32_iwdg_get_dt_node(&dt_info, node);
	     node != -FDT_ERR_NOTFOUND;
	     node = stm32_iwdg_get_dt_node(&dt_info, node)) {
		struct stm32_iwdg_instance *iwdg;
		uint32_t hw_init;
		uint32_t idx;

		count++;

		idx = stm32_iwdg_get_instance(dt_info.base);
		iwdg = &stm32_iwdg[idx];
		iwdg->base = dt_info.base;
		iwdg->clock = (unsigned long)dt_info.clock;

		/* DT can specify low power cases */
		if (fdt_getprop(fdt, node, "stm32,enable-on-stop", NULL) ==
		    NULL) {
			iwdg->flags |= IWDG_DISABLE_ON_STOP;
		}

		if (fdt_getprop(fdt, node, "stm32,enable-on-standby", NULL) ==
		    NULL) {
			iwdg->flags |= IWDG_DISABLE_ON_STANDBY;
		}

		/* Explicit list of supported bit flags */
		hw_init = stm32_iwdg_get_otp_config(idx);

		if ((hw_init & IWDG_HW_ENABLED) != 0) {
			if (dt_info.status == DT_DISABLED) {
				ERROR("OTP enabled but iwdg%u DT-disabled\n",
				      idx + 1U);
				panic();
			}
			iwdg->flags |= IWDG_HW_ENABLED;
		}

		if (dt_info.status == DT_DISABLED) {
			zeromem((void *)iwdg,
				sizeof(struct stm32_iwdg_instance));
			continue;
		}

		if ((hw_init & IWDG_DISABLE_ON_STOP) != 0) {
			iwdg->flags |= IWDG_DISABLE_ON_STOP;
		}

		if ((hw_init & IWDG_DISABLE_ON_STANDBY) != 0) {
			iwdg->flags |= IWDG_DISABLE_ON_STANDBY;
		}

		VERBOSE("IWDG%u found, %ssecure\n", idx + 1U,
			((dt_info.status & DT_NON_SECURE) != 0) ?
			"non-" : "");

		if ((dt_info.status & DT_NON_SECURE) != 0) {
			stm32mp_register_non_secure_periph_iomem(iwdg->base);
		} else {
			stm32mp_register_secure_periph_iomem(iwdg->base);
		}

		stm32mp_clk_enable(iwdg->clock);
		stm32mp_clk_disable(iwdg->clock);

#if defined(IMAGE_BL32)
		res = stm32_iwdg_conf_etimeout(node, iwdg);
		if (res != 0) {
			ERROR("IWDG%x early timeout config failed (%d)\n",
			      idx + 1, res);
			return res;
		}
#endif
#if defined(IMAGE_BL2)
		if (stm32_iwdg_shadow_update(idx, iwdg->flags) != BSEC_OK) {
			return -1;
		}
#endif
	}

	VERBOSE("%u IWDG instance%s found\n", count, (count > 1U) ? "s" : "");

	return 0;
}
