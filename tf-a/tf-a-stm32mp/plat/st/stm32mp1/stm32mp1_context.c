/*
 * Copyright (c) 2017-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <string.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <context.h>
#include <drivers/st/stm32_rtc.h>
#include <drivers/st/stm32mp_clkfunc.h>
#include <drivers/st/stm32mp1_ddr_regs.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <lib/mmio.h>
#include <lib/utils.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <smccc_helpers.h>

#include <stm32mp1_context.h>

#include <boot_api.h>

#define TAMP_BOOT_ITF_BACKUP_REG_ID	U(20)
#define TAMP_BOOT_ITF_MASK		U(0x0000FF00)
#define TAMP_BOOT_ITF_SHIFT		8

#define TRAINING_AREA_SIZE		64

#ifdef AARCH32_SP_OPTEE
/*
 * OPTEE_MAILBOX_MAGIC relates to struct backup_data_s as defined
 *
 * OPTEE_MAILBOX_MAGIC_V1:
 * Context provides magic, resume entry, zq0cr0 zdata and DDR training buffer.
 *
 * OPTEE_MAILBOX_MAGIC_V2:
 * Context provides magic, resume entry, zq0cr0 zdata, DDR training buffer
 * and PLL1 dual OPP settings structure (86 bytes).
 */
#define OPTEE_MAILBOX_MAGIC_V1		(0x0001 << 16)
#define OPTEE_MAILBOX_MAGIC_V2		(0x0002 << 16)
#define OPTEE_MAILBOX_MAGIC		(OPTEE_MAILBOX_MAGIC_V2 | \
					 TRAINING_AREA_SIZE)

#define MAGIC_ID(magic)			((magic) & GENMASK_32(31, 16))
#define MAGIC_AREA_SIZE(magic)		((magic) & GENMASK_32(15, 0))

#if (PLAT_MAX_OPP_NB != 2) || (PLAT_MAX_PLLCFG_NB != 6)
#error OPTEE_MAILBOX_MAGIC_V1 does not support expected PLL1 settings
#endif
#endif

/* pll_settings structure size definitions (reference to clock driver) */
#define PLL1_SETTINGS_SIZE		(((PLAT_MAX_OPP_NB * \
					  (PLAT_MAX_PLLCFG_NB + 3)) + 1) * \
					 sizeof(uint32_t))

/* Set to 600 bytes to be a bit flexible but could be optimized if needed */
#define CLOCK_CONTEXT_SIZE		600

/* SCMI needs only 24 bits to save the state of the 24 exposed clocks */
#define SCMI_CONTEXT_SIZE		(sizeof(uint8_t) * 4)

struct backup_data_s {
#ifdef AARCH32_SP_OPTEE
	uint32_t magic;
	uint32_t core0_resume_hint;
	uint32_t zq0cr0_zdata;
	uint8_t ddr_training_backup[TRAINING_AREA_SIZE];
	uint8_t pll1_settings[PLL1_SETTINGS_SIZE];
#else
	smc_ctx_t saved_smc_context[PLATFORM_CORE_COUNT];
	cpu_context_t saved_cpu_context[PLATFORM_CORE_COUNT];
	uint32_t zq0cr0_zdata;
	struct stm32_rtc_calendar rtc;
	uint8_t ddr_training_backup[TRAINING_AREA_SIZE];
	uint8_t pll1_settings[PLL1_SETTINGS_SIZE];
	unsigned long long stgen;
	uint8_t clock_cfg[CLOCK_CONTEXT_SIZE];
	uint8_t scmi_context[SCMI_CONTEXT_SIZE];
#endif
};

#ifdef AARCH32_SP_OPTEE
uint32_t stm32_pm_get_optee_ep(void)
{
	struct backup_data_s *backup_data;
	uint32_t ep;

	stm32mp_clk_enable(BKPSRAM);

	/* Context & Data to be saved at the beginning of Backup SRAM */
	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	switch (MAGIC_ID(backup_data->magic)) {
	case OPTEE_MAILBOX_MAGIC_V1:
	case OPTEE_MAILBOX_MAGIC_V2:
		if (MAGIC_AREA_SIZE(backup_data->magic) != TRAINING_AREA_SIZE) {
			panic();
		}
		break;
	default:
		ERROR("PM context: bad magic\n");
		panic();
	}

	ep = backup_data->core0_resume_hint;

	stm32mp_clk_disable(BKPSRAM);

	return ep;
}
#else /*AARCH32_SP_OPTEE*/
void stm32_clean_context(void)
{
	stm32mp_clk_enable(BKPSRAM);

	zeromem((void *)STM32MP_BACKUP_RAM_BASE, sizeof(struct backup_data_s));

	stm32mp_clk_disable(BKPSRAM);
}

void stm32mp1_pm_save_clock_cfg(size_t offset, uint8_t *data, size_t size)
{
	struct backup_data_s *backup_data;

	if (offset + size > sizeof(backup_data->clock_cfg)) {
		panic();
	}

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	stm32mp_clk_enable(BKPSRAM);

	memcpy(backup_data->clock_cfg + offset, data, size);

	stm32mp_clk_disable(BKPSRAM);
}

void stm32mp1_pm_restore_clock_cfg(size_t offset, uint8_t *data, size_t size)
{
	struct backup_data_s *backup_data;

	if (offset + size > sizeof(backup_data->clock_cfg))
		panic();

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	stm32mp_clk_enable(BKPSRAM);

	memcpy(data, backup_data->clock_cfg + offset, size);

	stm32mp_clk_disable(BKPSRAM);
}

int stm32_save_context(uint32_t zq0cr0_zdata)
{
	void *smc_context;
	void *cpu_context;
	struct backup_data_s *backup_data;

	stm32mp1_clock_suspend();

	stm32mp_clk_enable(BKPSRAM);

	/* Context & Data to be saved at the beginning of Backup SRAM */
	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	/* Retrieve smc context struct address */
	smc_context = smc_get_ctx(NON_SECURE);

	/* Retrieve smc context struct address */
	cpu_context = cm_get_context(NON_SECURE);

	/* Save context in Backup SRAM */
	memcpy(&backup_data->saved_smc_context[0], smc_context,
	       sizeof(smc_ctx_t) * PLATFORM_CORE_COUNT);
	memcpy(&backup_data->saved_cpu_context[0], cpu_context,
	       sizeof(cpu_context_t) * PLATFORM_CORE_COUNT);

	backup_data->zq0cr0_zdata = zq0cr0_zdata;

	stm32_rtc_get_calendar(&backup_data->rtc);
	backup_data->stgen = stm32mp_stgen_get_counter();

	stm32mp1_clk_lp_save_opp_pll1_settings(backup_data->pll1_settings,
					sizeof(backup_data->pll1_settings));

	stm32mp1_pm_save_scmi_state(backup_data->scmi_context,
				    sizeof(backup_data->scmi_context));

	save_clock_pm_context();

	stm32mp_clk_disable(BKPSRAM);

	return 0;
}

int stm32_restore_context(void)
{
	void *smc_context;
	void *cpu_context;
	struct backup_data_s *backup_data;
	struct stm32_rtc_calendar current_calendar;
	unsigned long long stdby_time_in_ms;

	/* Context & Data to be saved at the beginning of Backup SRAM */
	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	/* Retrieve smc context struct address */
	smc_context = smc_get_ctx(NON_SECURE);

	/* Retrieve smc context struct address */
	cpu_context = cm_get_context(NON_SECURE);

	stm32mp_clk_enable(BKPSRAM);

	restore_clock_pm_context();

	stm32mp1_pm_restore_scmi_state(backup_data->scmi_context,
				       sizeof(backup_data->scmi_context));

	/* Restore data from Backup SRAM */
	memcpy(smc_context, backup_data->saved_smc_context,
	       sizeof(smc_ctx_t) * PLATFORM_CORE_COUNT);
	memcpy(cpu_context, backup_data->saved_cpu_context,
	       sizeof(cpu_context_t) * PLATFORM_CORE_COUNT);

	/* Restore STGEN counter with standby mode length */
	stm32_rtc_get_calendar(&current_calendar);
	stdby_time_in_ms = stm32_rtc_diff_calendar(&current_calendar,
						   &backup_data->rtc);
	stm32mp_stgen_restore_counter(backup_data->stgen, stdby_time_in_ms);

	stm32mp1_clk_lp_load_opp_pll1_settings(backup_data->pll1_settings,
					sizeof(backup_data->pll1_settings));

	stm32mp_clk_disable(BKPSRAM);

	stm32mp1_clock_resume();

	return 0;
}

unsigned long long stm32_get_stgen_from_context(void)
{
	struct backup_data_s *backup_data;
	unsigned long long stgen_cnt;

	stm32mp_clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	stgen_cnt = backup_data->stgen;

	stm32mp_clk_disable(BKPSRAM);

	return stgen_cnt;
}
#endif /*AARCH32_SP_OPTEE*/

uint32_t stm32_get_zdata_from_context(void)
{
	struct backup_data_s *backup_data;
	uint32_t zdata;

	stm32mp_clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	zdata = (backup_data->zq0cr0_zdata >> DDRPHYC_ZQ0CRN_ZDATA_SHIFT) &
		DDRPHYC_ZQ0CRN_ZDATA_MASK;

	stm32mp_clk_disable(BKPSRAM);

	return zdata;
}

#ifdef AARCH32_SP_OPTEE
static int pll1_settings_in_context(struct backup_data_s *backup_data)
{
	switch (MAGIC_ID(backup_data->magic)) {
	case OPTEE_MAILBOX_MAGIC_V1:
		return -ENOENT;
	case OPTEE_MAILBOX_MAGIC_V2:
		assert(MAGIC_AREA_SIZE(backup_data->magic) ==
		       TRAINING_AREA_SIZE);
		return 0;
	default:
		panic();
	}
}
#else
static int pll1_settings_in_context(struct backup_data_s *backup_data)
{
	return 0;
}
#endif

int stm32_get_pll1_settings_from_context(void)
{
	struct backup_data_s *backup_data;
	int ret;

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	stm32mp_clk_enable(BKPSRAM);

	ret = pll1_settings_in_context(backup_data);
	if (ret == 0) {
		uint8_t *data = (uint8_t *)backup_data->pll1_settings;
		size_t size = sizeof(backup_data->pll1_settings);

		stm32mp1_clk_lp_load_opp_pll1_settings(data, size);
	}

	stm32mp_clk_disable(BKPSRAM);

	return ret;
}

bool stm32_are_pll1_settings_valid_in_context(void)
{
	struct backup_data_s *backup_data;
	uint32_t *data;
	bool is_valid;

	stm32mp_clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;
	data = (uint32_t *)backup_data->pll1_settings;

	is_valid = (data[0] == PLL1_SETTINGS_VALID_ID);

	stm32mp_clk_disable(BKPSRAM);

	return is_valid;
}

int stm32_save_boot_interface(uint32_t interface, uint32_t instance)
{
	uint32_t bkpr_itf_idx = tamp_bkpr(TAMP_BOOT_ITF_BACKUP_REG_ID);

	stm32mp_clk_enable(RTCAPB);

	mmio_clrsetbits_32(bkpr_itf_idx,
			   TAMP_BOOT_ITF_MASK,
			   ((interface << 4) | (instance & 0xFU)) <<
			   TAMP_BOOT_ITF_SHIFT);

	stm32mp_clk_disable(RTCAPB);

	return 0;
}

int stm32_get_boot_interface(uint32_t *interface, uint32_t *instance)
{
	uint32_t backup_reg_itf;
	uint32_t bkpr_itf_idx = tamp_bkpr(TAMP_BOOT_ITF_BACKUP_REG_ID);

	stm32mp_clk_enable(RTCAPB);

	backup_reg_itf = (mmio_read_32(bkpr_itf_idx) &
			  TAMP_BOOT_ITF_MASK) >> TAMP_BOOT_ITF_SHIFT;

	stm32mp_clk_disable(RTCAPB);

	*interface = backup_reg_itf >> 4;
	*instance = backup_reg_itf & 0xFU;

	return 0;
}

#if defined(IMAGE_BL32)
/*
 * When returning from STANDBY, the 64 first bytes of DDR will be overwritten
 * during DDR DQS training. This area must then be saved before going to
 * standby, and will be restored after
 */
void stm32_save_ddr_training_area(void)
{
	struct backup_data_s *backup_data;
	int ret __unused;

	stm32mp_clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	ret = mmap_add_dynamic_region(STM32MP_DDR_BASE, STM32MP_DDR_BASE,
				      PAGE_SIZE, MT_MEMORY | MT_RW | MT_NS);
	assert(ret == 0);

	memcpy(&backup_data->ddr_training_backup,
	       (const uint32_t *)STM32MP_DDR_BASE,
	       TRAINING_AREA_SIZE);
	dsb();

	ret = mmap_remove_dynamic_region(STM32MP_DDR_BASE, PAGE_SIZE);
	assert(ret == 0);

	stm32mp_clk_disable(BKPSRAM);
}
#endif

void stm32_restore_ddr_training_area(void)
{
	struct backup_data_s *backup_data;

	stm32mp_clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	memcpy((uint32_t *)STM32MP_DDR_BASE,
	       &backup_data->ddr_training_backup,
	       TRAINING_AREA_SIZE);
	dsb();

	stm32mp_clk_disable(BKPSRAM);
}
