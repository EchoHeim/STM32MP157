/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <libfdt.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/arm/gic_common.h>
#include <drivers/arm/gicv2.h>
#include <drivers/delay_timer.h>
#include <drivers/st/stm32_iwdg.h>
#include <drivers/st/stm32_rtc.h>
#include <drivers/st/stm32mp_clkfunc.h>
#include <drivers/st/stm32mp_pmic.h>
#include <drivers/st/stm32mp1_ddr_helpers.h>
#include <drivers/st/stm32mp1_pwr.h>
#include <drivers/st/stm32mp1_rcc.h>
#include <drivers/st/stpmic1.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <dt-bindings/power/stm32mp1-power.h>
#include <lib/mmio.h>
#include <plat/common/platform.h>

#include <boot_api.h>
#include <stm32mp_common.h>
#include <stm32mp_dt.h>
#include <stm32mp1_context.h>
#include <stm32mp1_low_power.h>
#include <stm32mp1_private.h>

static unsigned int gicc_pmr;
static struct stm32_rtc_calendar sleep_time;
static bool enter_cstop_done;
static uint32_t int_stack[STM32MP_INT_STACK_SIZE];

extern void wfi_svc_int_enable(uintptr_t stack_addr);

struct pwr_lp_config {
	uint32_t pwr_cr1;
	uint32_t pwr_mpucr;
	const char *regul_suspend_node_name;
};

#define PWR_CR1_MASK	(PWR_CR1_LPDS | PWR_CR1_LPCFG | PWR_CR1_LVDS)
#define PWR_MPUCR_MASK	(PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF | PWR_MPUCR_PDDS)

static const struct pwr_lp_config config_pwr[STM32_PM_MAX_SOC_MODE] = {
	[STM32_PM_CSLEEP_RUN] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSSF,
		.regul_suspend_node_name = NULL,
	},
	[STM32_PM_CSTOP_ALLOW_STOP] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF,
		.regul_suspend_node_name = NULL,
	},
	[STM32_PM_CSTOP_ALLOW_LP_STOP] = {
		.pwr_cr1 = PWR_CR1_LPDS,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF,
		.regul_suspend_node_name = "lp-stop",
	},
	[STM32_PM_CSTOP_ALLOW_LPLV_STOP] = {
		.pwr_cr1 = PWR_CR1_LVDS | PWR_CR1_LPDS | PWR_CR1_LPCFG,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF,
		.regul_suspend_node_name = "lplv-stop",
	},
	[STM32_PM_CSTOP_ALLOW_STANDBY_DDR_SR] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF |
			PWR_MPUCR_PDDS,
		.regul_suspend_node_name = "standby-ddr-sr",
	},
	[STM32_PM_CSTOP_ALLOW_STANDBY_DDR_OFF] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF |
			PWR_MPUCR_PDDS,
		.regul_suspend_node_name = "standby-ddr-off",
	},
	[STM32_PM_SHUTDOWN] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = 0U,
		.regul_suspend_node_name = "standby-ddr-off",
	},
};

#define GICC_PMR_PRIORITY_8	U(0x8)

void stm32_apply_pmic_suspend_config(uint32_t mode)
{
	const char *node_name = config_pwr[mode].regul_suspend_node_name;

	assert(mode < ARRAY_SIZE(config_pwr));

	if (node_name != NULL) {
		if (!initialize_pmic_i2c()) {
			panic();
		}

		if (pmic_set_lp_config(node_name) < 0) {
			panic();
		}

		if (pmic_configure_boot_on_regulators() < 0) {
			panic();
		}
	}
}

/*
 * stm32_enter_cstop - Prepare CSTOP mode
 *
 * @mode - Target low power mode
 * @nsec_addr - Non secure resume entry point
 * Return 0 if succeed to suspend, non 0 else.
 */
static void enter_cstop(uint32_t mode, uint32_t nsec_addr)
{
	uint32_t zq0cr0_zdata;
	uint32_t bkpr_core1_addr =
		tamp_bkpr(BOOT_API_CORE1_BRANCH_ADDRESS_TAMP_BCK_REG_IDX);
	uint32_t bkpr_core1_magic =
		tamp_bkpr(BOOT_API_CORE1_MAGIC_NUMBER_TAMP_BCK_REG_IDX);
	uint32_t pwr_cr1 = config_pwr[mode].pwr_cr1;
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t rcc_base = stm32mp_rcc_base();

	stm32mp1_syscfg_disable_io_compensation();

	/* Switch to Software Self-Refresh mode */
	ddr_set_sr_mode(DDR_SSR_MODE);

	dcsw_op_all(DC_OP_CISW);

	stm32_clean_context();

	if (mode == STM32_PM_CSTOP_ALLOW_STANDBY_DDR_SR) {
		/*
		 * The first 64 bytes of DDR need to be saved for DDR DQS
		 * training
		 */
		stm32_save_ddr_training_area();
	}

	if (dt_pmic_status() > 0) {
		stm32_apply_pmic_suspend_config(mode);

		if (mode == STM32_PM_CSTOP_ALLOW_LP_STOP) {
			pwr_cr1 |= PWR_CR1_LPCFG;
		}
	}

	/* Clear RCC interrupt before enabling it */
	mmio_setbits_32(rcc_base + RCC_MP_CIFR, RCC_MP_CIFR_WKUPF);

	/* Enable RCC Wake-up */
	mmio_setbits_32(rcc_base + RCC_MP_CIER, RCC_MP_CIFR_WKUPF);

	/* Configure low power mode */
	mmio_clrsetbits_32(pwr_base + PWR_MPUCR, PWR_MPUCR_MASK,
			   config_pwr[mode].pwr_mpucr);
	mmio_clrsetbits_32(pwr_base + PWR_CR1, PWR_CR1_MASK,
			   pwr_cr1);

	/* Clear RCC pending interrupt flags */
	mmio_write_32(rcc_base + RCC_MP_CIFR, RCC_MP_CIFR_MASK);

	/* Request CSTOP mode to RCC */
	mmio_setbits_32(rcc_base + RCC_MP_SREQSETR,
			RCC_MP_SREQSETR_STPREQ_P0 | RCC_MP_SREQSETR_STPREQ_P1);

	stm32_iwdg_refresh();

	gicc_pmr = plat_ic_set_priority_mask(GICC_PMR_PRIORITY_8);

	/*
	 * Set DDR in Self-refresh, even if no return address is given.
	 * This is also the procedure awaited when switching off power supply.
	 */
	if (ddr_standby_sr_entry(&zq0cr0_zdata) != 0) {
		panic();
	}

	stm32mp_clk_enable(RTCAPB);

	mmio_write_32(bkpr_core1_addr, 0);
	mmio_write_32(bkpr_core1_magic, 0);

	stm32mp1_clock_stopmode_save();

	if (mode == STM32_PM_CSTOP_ALLOW_STANDBY_DDR_SR) {
		/*
		 * Save non-secure world entrypoint after standby in Backup
		 * register
		 */
		mmio_write_32(bkpr_core1_addr, nsec_addr);
		mmio_write_32(bkpr_core1_magic,
			      BOOT_API_A7_CORE0_MAGIC_NUMBER);

		if (stm32_save_context(zq0cr0_zdata) != 0) {
			panic();
		}

		/* Keep retention and backup RAM content in standby */
		mmio_setbits_32(pwr_base + PWR_CR2, PWR_CR2_BREN |
				PWR_CR2_RREN);
		while ((mmio_read_32(pwr_base + PWR_CR2) &
			(PWR_CR2_BRRDY | PWR_CR2_RRRDY)) == 0U) {
			;
		}
	}

	stm32mp_clk_disable(RTCAPB);

	stm32_rtc_get_calendar(&sleep_time);

	enter_cstop_done = true;
}

/*
 * stm32_exit_cstop - Exit from CSTOP mode
 */
void stm32_exit_cstop(void)
{
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t rcc_base = stm32mp_rcc_base();
	unsigned long long stdby_time_in_ms;
	struct stm32_rtc_calendar current_calendar;

	if (!enter_cstop_done) {
		return;
	}

	enter_cstop_done = false;

	if (ddr_sw_self_refresh_exit() != 0) {
		panic();
	}

	/* Switch to memorized Self-Refresh mode */
	ddr_restore_sr_mode();

	plat_ic_set_priority_mask(gicc_pmr);

	/* Disable RCC Wake-up */
	mmio_clrbits_32(rcc_base + RCC_MP_CIER, RCC_MP_CIFR_WKUPF);

	/* Disable STOP request */
	mmio_setbits_32(rcc_base + RCC_MP_SREQCLRR,
			RCC_MP_SREQSETR_STPREQ_P0 | RCC_MP_SREQSETR_STPREQ_P1);

	dsb();
	isb();

	/* Disable retention and backup RAM content after stop */
	mmio_clrbits_32(pwr_base + PWR_CR2, PWR_CR2_BREN | PWR_CR2_RREN);

	/* Update STGEN counter with low power mode duration */
	stm32_rtc_get_calendar(&current_calendar);

	stdby_time_in_ms = stm32_rtc_diff_calendar(&current_calendar,
						   &sleep_time);
	stm32mp_stgen_restore_counter(stm32_get_stgen_from_context(),
				      stdby_time_in_ms);

	stm32mp1_syscfg_enable_io_compensation();

	if (stm32mp1_clock_stopmode_resume() != 0) {
		panic();
	}
}

static void enter_shutdown(void)
{
	/* Set DDR in Self-refresh before shutting down the platform */
	if (ddr_standby_sr_entry(NULL) != 0) {
		WARN("DDR can't be set in Self-refresh mode\n");
	}

	if (dt_pmic_status() > 0) {
		if (!initialize_pmic_i2c()) {
			panic();
		}

		stpmic1_switch_off();

		udelay(100);

		/* Shouldn't be reached */
		panic();
	}
}

static void enter_csleep(void)
{
	uintptr_t pwr_base = stm32mp_pwr_base();

	mmio_clrsetbits_32(pwr_base + PWR_MPUCR, PWR_MPUCR_MASK,
			   config_pwr[STM32_PM_CSLEEP_RUN].pwr_mpucr);
	mmio_clrsetbits_32(pwr_base + PWR_CR1, PWR_CR1_MASK,
			   config_pwr[STM32_PM_CSLEEP_RUN].pwr_cr1);

	stm32_pwr_down_wfi();
}

void stm32_enter_low_power(uint32_t mode, uint32_t nsec_addr)
{
	switch (mode) {
	case STM32_PM_SHUTDOWN:
		enter_shutdown();
		break;

	case STM32_PM_CSLEEP_RUN:
		enter_csleep();
		break;

	default:
		enter_cstop(mode, nsec_addr);
		break;
	}
}

void stm32_pwr_down_wfi(void)
{
	uint32_t interrupt = GIC_SPURIOUS_INTERRUPT;

	stm32mp1_calib_set_wakeup(false);

	while (interrupt == GIC_SPURIOUS_INTERRUPT &&
	       !stm32mp1_calib_get_wakeup()) {
		wfi_svc_int_enable((uintptr_t)&int_stack[0]);

		interrupt = gicv2_acknowledge_interrupt();

		if (interrupt != GIC_SPURIOUS_INTERRUPT) {
			gicv2_end_of_interrupt(interrupt);
		}

		stm32_iwdg_refresh();
	}
}
