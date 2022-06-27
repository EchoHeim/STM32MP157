/*
 * Copyright (c) 2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <common/debug.h>
#include <drivers/st/stm32mp1_clk.h>
#include <drivers/st/stpmic1.h>
#include <lib/mmio.h>

#include <stm32mp_common.h>
#include <stm32mp_dt.h>
#include <stm32mp1_private.h>

/*
 * SYSCFG register offsets (base relative)
 */
#define SYSCFG_BOOTR				0x00U
#define SYSCFG_IOCTRLSETR			0x18U
#define SYSCFG_ICNR				0x1CU
#define SYSCFG_CMPCR				0x20U
#define SYSCFG_CMPENSETR			0x24U

/*
 * SYSCFG_BOOTR Register
 */
#define SYSCFG_BOOTR_BOOT_MASK			GENMASK(2, 0)
#define SYSCFG_BOOTR_BOOTPD_MASK		GENMASK(6, 4)
#define SYSCFG_BOOTR_BOOTPD_SHIFT		4
/*
 * SYSCFG_IOCTRLSETR Register
 */
#define SYSCFG_IOCTRLSETR_HSLVEN_TRACE		BIT(0)
#define SYSCFG_IOCTRLSETR_HSLVEN_QUADSPI	BIT(1)
#define SYSCFG_IOCTRLSETR_HSLVEN_ETH		BIT(2)
#define SYSCFG_IOCTRLSETR_HSLVEN_SDMMC		BIT(3)
#define SYSCFG_IOCTRLSETR_HSLVEN_SPI		BIT(4)

/*
 * SYSCFG_ICNR Register
 */
#define SYSCFG_ICNR_AXI_M9			BIT(9)

/*
 * SYSCFG_CMPCR Register
 */
#define SYSCFG_CMPCR_SW_CTRL			BIT(1)
#define SYSCFG_CMPCR_READY			BIT(8)
#define SYSCFG_CMPCR_RANSRC			GENMASK(19, 16)
#define SYSCFG_CMPCR_RANSRC_SHIFT		16
#define SYSCFG_CMPCR_RAPSRC			GENMASK(23, 20)
#define SYSCFG_CMPCR_ANSRC_SHIFT		24

#define SYSCFG_CMPCR_READY_TIMEOUT_US		10000U

/*
 * SYSCFG_CMPENSETR Register
 */
#define SYSCFG_CMPENSETR_MPU_EN			BIT(0)

void stm32mp1_syscfg_init(void)
{
	uint32_t bootr;
	uint32_t otp_value;
	uint32_t vdd_voltage;
	bool product_below_2v5;

	/*
	 * Interconnect update : select master using the port 1.
	 * LTDC = AXI_M9.
	 */
	mmio_write_32(SYSCFG_BASE + SYSCFG_ICNR, SYSCFG_ICNR_AXI_M9);

	/* Disable Pull-Down for boot pin connected to VDD */
	bootr = mmio_read_32(SYSCFG_BASE + SYSCFG_BOOTR) &
		SYSCFG_BOOTR_BOOT_MASK;
	mmio_clrsetbits_32(SYSCFG_BASE + SYSCFG_BOOTR, SYSCFG_BOOTR_BOOTPD_MASK,
			   bootr << SYSCFG_BOOTR_BOOTPD_SHIFT);

	/*
	 * High Speed Low Voltage Pad mode Enable for SPI, SDMMC, ETH, QSPI
	 * and TRACE. Needed above ~50MHz and conditioned by AFMUX selection.
	 * It could be disabled for low frequencies or if AFMUX is selected
	 * but the function is not used, typically for TRACE.
	 * If high speed low voltage pad mode is node enable, platform will
	 * over consume.
	 *
	 * WARNING:
	 *   Enabling High Speed mode while VDD > 2.7V
	 *   with the OTP product_below_2v5 (OTP 18, BIT 13)
	 *   erroneously set to 1 can damage the SoC!
	 *   => TF-A enables the low power mode only if VDD < 2.7V (in DT)
	 *      but this value needs to be consistent with board design.
	 */
	if (stm32_get_otp_value(HW2_OTP, &otp_value) != 0) {
		panic();
	}

	product_below_2v5 = (otp_value & HW2_OTP_PRODUCT_BELOW_2V5) != 0U;

	/* Get VDD supply */
	vdd_voltage = dt_get_pwr_vdd_voltage();

	/* Check if VDD is Low Voltage */
	if (vdd_voltage == 0U) {
		WARN("VDD unknown");
	} else if (vdd_voltage < 2700000U) {
		mmio_write_32(SYSCFG_BASE + SYSCFG_IOCTRLSETR,
			      SYSCFG_IOCTRLSETR_HSLVEN_TRACE |
			      SYSCFG_IOCTRLSETR_HSLVEN_QUADSPI |
			      SYSCFG_IOCTRLSETR_HSLVEN_ETH |
			      SYSCFG_IOCTRLSETR_HSLVEN_SDMMC |
			      SYSCFG_IOCTRLSETR_HSLVEN_SPI);

		if (!product_below_2v5) {
			INFO("Product_below_2v5=0: HSLVEN protected by HW\n");
		}
	} else {
		if (product_below_2v5) {
			ERROR("Product_below_2v5=1:\n");
			ERROR("\tHSLVEN update is destructive,\n");
			ERROR("\tno update as VDD > 2.7V\n");
			panic();
		}
	}

	stm32mp1_syscfg_enable_io_compensation();
}

void stm32mp1_syscfg_enable_io_compensation(void)
{
	uint64_t start;

	/*
	 * Activate automatic I/O compensation.
	 * Warning: need to ensure CSI enabled and ready in clock driver.
	 * Enable non-secure clock, we assume non-secure is suspended.
	 */
	stm32mp1_clk_force_enable(SYSCFG);

	mmio_setbits_32(SYSCFG_BASE + SYSCFG_CMPENSETR,
			SYSCFG_CMPENSETR_MPU_EN);

	start = timeout_init_us(SYSCFG_CMPCR_READY_TIMEOUT_US);

	while ((mmio_read_32(SYSCFG_BASE + SYSCFG_CMPCR) &
		SYSCFG_CMPCR_READY) == 0U) {
		if (timeout_elapsed(start)) {
			/*
			 * Failure on IO compensation enable is not a issue:
			 * warn only.
			 */
			WARN("IO compensation cell not ready\n");
			break;
		}
	}

	mmio_clrbits_32(SYSCFG_BASE + SYSCFG_CMPCR, SYSCFG_CMPCR_SW_CTRL);
}

void stm32mp1_syscfg_disable_io_compensation(void)
{
	uint32_t value;

	/*
	 * Deactivate automatic I/O compensation.
	 * Warning: CSI is disabled automatically in STOP if not
	 * requested for other usages and always OFF in STANDBY.
	 * Disable non-secure SYSCFG clock, we assume non-secure is suspended.
	 */
	value = mmio_read_32(SYSCFG_BASE + SYSCFG_CMPCR) >>
	      SYSCFG_CMPCR_ANSRC_SHIFT;

	mmio_clrbits_32(SYSCFG_BASE + SYSCFG_CMPCR,
			SYSCFG_CMPCR_RANSRC | SYSCFG_CMPCR_RAPSRC);

	value = mmio_read_32(SYSCFG_BASE + SYSCFG_CMPCR) |
		(value << SYSCFG_CMPCR_RANSRC_SHIFT);

	mmio_write_32(SYSCFG_BASE + SYSCFG_CMPCR, value | SYSCFG_CMPCR_SW_CTRL);

	mmio_clrbits_32(SYSCFG_BASE + SYSCFG_CMPENSETR,
			SYSCFG_CMPENSETR_MPU_EN);

	stm32mp1_clk_force_disable(SYSCFG);
}
