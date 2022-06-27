/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <drivers/st/stm32mp1_ddr.h>
#include <drivers/st/stm32mp1_ddr_helpers.h>
#include <drivers/st/stm32mp1_ddr_regs.h>
#include <lib/mmio.h>

#define TIMEOUT_500US	500U

static enum stm32mp1_ddr_sr_mode saved_ddr_sr_mode;

void ddr_enable_clock(void)
{
	stm32mp1_clk_rcc_regs_lock();

	mmio_setbits_32(stm32mp_rcc_base() + RCC_DDRITFCR,
			RCC_DDRITFCR_DDRC1EN |
			RCC_DDRITFCR_DDRC2EN |
			RCC_DDRITFCR_DDRPHYCEN |
			RCC_DDRITFCR_DDRPHYCAPBEN |
			RCC_DDRITFCR_DDRCAPBEN);

	stm32mp1_clk_rcc_regs_unlock();
}

static void do_sw_handshake(void)
{
	uintptr_t ddrctrl_base = stm32mp_ddrctrl_base();

	mmio_clrbits_32(ddrctrl_base + DDRCTRL_SWCTL, DDRCTRL_SWCTL_SW_DONE);
}

static void do_sw_ack(void)
{
	uint64_t timeout;
	uintptr_t ddrctrl_base = stm32mp_ddrctrl_base();

	mmio_setbits_32(ddrctrl_base + DDRCTRL_SWCTL, DDRCTRL_SWCTL_SW_DONE);

	timeout = timeout_init_us(TIMEOUT_500US);
	while ((mmio_read_32(ddrctrl_base + DDRCTRL_SWSTAT) &
		DDRCTRL_SWSTAT_SW_DONE_ACK) == 0U) {
		if (timeout_elapsed(timeout)) {
			panic();
		}
	}
}

static int ddr_sw_self_refresh_in(void)
{
	uint64_t timeout;
	uint32_t stat;
	uint32_t operating_mode;
	uint32_t selref_type;
	uint8_t op_mode_changed = 0;
	uintptr_t rcc_base = stm32mp_rcc_base();
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t ddrctrl_base = stm32mp_ddrctrl_base();
	uintptr_t ddrphyc_base = stm32mp_ddrphyc_base();

	stm32mp1_clk_rcc_regs_lock();

	mmio_clrbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_AXIDCGEN);

	stm32mp1_clk_rcc_regs_unlock();

	/* Blocks AXI ports from taking anymore transactions */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PCTRL_0,
			DDRCTRL_PCTRL_N_PORT_EN);
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PCTRL_1,
			DDRCTRL_PCTRL_N_PORT_EN);

	/* Waits unit all AXI ports are idle
	 * Poll PSTAT.rd_port_busy_n = 0
	 * Poll PSTAT.wr_port_busy_n = 0
	 */
	timeout = timeout_init_us(TIMEOUT_500US);
	while (mmio_read_32(ddrctrl_base + DDRCTRL_PSTAT)) {
		if (timeout_elapsed(timeout)) {
			goto pstat_failed;
		}
	}
	/* SW Self-Refresh entry */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_SELFREF_SW);

	/* Wait operating mode change in self-refresh mode
	 * with STAT.operating_mode[1:0]==11.
	 * Ensure transition to self-refresh was due to software
	 * by checking also that STAT.selfref_type[1:0]=2.
	 */
	timeout = timeout_init_us(TIMEOUT_500US);
	while (!timeout_elapsed(timeout)) {
		stat = mmio_read_32(ddrctrl_base + DDRCTRL_STAT);
		operating_mode = stat & DDRCTRL_STAT_OPERATING_MODE_MASK;
		selref_type = stat & DDRCTRL_STAT_SELFREF_TYPE_MASK;

		if ((operating_mode == DDRCTRL_STAT_OPERATING_MODE_SR) &&
		    (selref_type == DDRCTRL_STAT_SELFREF_TYPE_SR)) {
			op_mode_changed = 1;
			break;
		}
	}

	if (op_mode_changed == 0U)
		goto selfref_sw_failed;

	/* IOs powering down (PUBL registers) */
	mmio_setbits_32(ddrphyc_base + DDRPHYC_ACIOCR, DDRPHYC_ACIOCR_ACPDD);

	mmio_setbits_32(ddrphyc_base + DDRPHYC_ACIOCR, DDRPHYC_ACIOCR_ACPDR);

	mmio_clrsetbits_32(ddrphyc_base + DDRPHYC_ACIOCR,
			   DDRPHYC_ACIOCR_CKPDD_MASK,
			   DDRPHYC_ACIOCR_CKPDD_0);

	mmio_clrsetbits_32(ddrphyc_base + DDRPHYC_ACIOCR,
			   DDRPHYC_ACIOCR_CKPDR_MASK,
			   DDRPHYC_ACIOCR_CKPDR_0);

	mmio_clrsetbits_32(ddrphyc_base + DDRPHYC_ACIOCR,
			   DDRPHYC_ACIOCR_CSPDD_MASK,
			   DDRPHYC_ACIOCR_CSPDD_0);

	/* Disable command/address output driver */
	mmio_clrbits_32(ddrphyc_base + DDRPHYC_ACIOCR, DDRPHYC_ACIOCR_ACOE);

	mmio_setbits_32(ddrphyc_base + DDRPHYC_DXCCR, DDRPHYC_DXCCR_DXPDD);

	mmio_setbits_32(ddrphyc_base + DDRPHYC_DXCCR, DDRPHYC_DXCCR_DXPDR);

	mmio_clrsetbits_32(ddrphyc_base + DDRPHYC_DSGCR,
			   DDRPHYC_DSGCR_ODTPDD_MASK,
			   DDRPHYC_DSGCR_ODTPDD_0);

	mmio_setbits_32(ddrphyc_base + DDRPHYC_DSGCR, DDRPHYC_DSGCR_NL2PD);

	mmio_clrsetbits_32(ddrphyc_base + DDRPHYC_DSGCR,
			   DDRPHYC_DSGCR_CKEPDD_MASK,
			   DDRPHYC_DSGCR_CKEPDD_0);

	/* Disable PZQ cell (PUBL register) */
	mmio_setbits_32(ddrphyc_base + DDRPHYC_ZQ0CR0, DDRPHYC_ZQ0CRN_ZQPD);

	/* Set latch */
	mmio_clrbits_32(ddrphyc_base + DDRPHYC_DSGCR, DDRPHYC_DSGCR_CKOE);

	/* Additional delay to avoid early latch */
	udelay(10);

	/* Activate sw retention in PWRCTRL */
	stm32mp_pwr_regs_lock();
	mmio_setbits_32(pwr_base + PWR_CR3, PWR_CR3_DDRRETEN);
	stm32mp_pwr_regs_unlock();

	/* Switch controller clocks (uMCTL2/PUBL) to DLL ref clock */
	stm32mp1_clk_rcc_regs_lock();
	mmio_setbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_GSKPCTRL);
	stm32mp1_clk_rcc_regs_unlock();

	/* Disable all DLLs: GLITCH window */
	mmio_setbits_32(ddrphyc_base + DDRPHYC_ACDLLCR,
			DDRPHYC_ACDLLCR_DLLDIS);

	mmio_setbits_32(ddrphyc_base + DDRPHYC_DX0DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_setbits_32(ddrphyc_base + DDRPHYC_DX1DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_setbits_32(ddrphyc_base + DDRPHYC_DX2DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_setbits_32(ddrphyc_base + DDRPHYC_DX3DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	stm32mp1_clk_rcc_regs_lock();

	/* Switch controller clocks (uMCTL2/PUBL) to DLL output clock */
	mmio_clrbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_GSKPCTRL);

	/* Deactivate all DDR clocks */
	mmio_clrbits_32(rcc_base + RCC_DDRITFCR,
			RCC_DDRITFCR_DDRC1EN |
			RCC_DDRITFCR_DDRC2EN |
			RCC_DDRITFCR_DDRCAPBEN |
			RCC_DDRITFCR_DDRPHYCAPBEN);

	stm32mp1_clk_rcc_regs_unlock();

	return 0;

selfref_sw_failed:
	/* This bit should be cleared to restore DDR in its previous state */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_SELFREF_SW);

pstat_failed:
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PCTRL_0,
			DDRCTRL_PCTRL_N_PORT_EN);
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PCTRL_1,
			DDRCTRL_PCTRL_N_PORT_EN);

	return -1;
}

int ddr_sw_self_refresh_exit(void)
{
	uint64_t timeout;
	uintptr_t rcc_base = stm32mp_rcc_base();
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t ddrctrl_base = stm32mp_ddrctrl_base();
	uintptr_t ddrphyc_base = stm32mp_ddrphyc_base();

	/* Enable all clocks */
	ddr_enable_clock();

	do_sw_handshake();

	/* Mask dfi_init_complete_en */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_DFIMISC,
			DDRCTRL_DFIMISC_DFI_INIT_COMPLETE_EN);

	do_sw_ack();

	/* Switch controller clocks (uMCTL2/PUBL) to DLL ref clock */
	stm32mp1_clk_rcc_regs_lock();
	mmio_setbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_GSKPCTRL);
	stm32mp1_clk_rcc_regs_unlock();

	/* Enable all DLLs: GLITCH window */
	mmio_clrbits_32(ddrphyc_base + DDRPHYC_ACDLLCR,
			DDRPHYC_ACDLLCR_DLLDIS);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_DX0DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_DX1DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_DX2DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_DX3DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	/* Additional delay to avoid early DLL clock switch */
	udelay(50);

	/* Switch controller clocks (uMCTL2/PUBL) to DLL ref clock */
	stm32mp1_clk_rcc_regs_lock();
	mmio_clrbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_GSKPCTRL);
	stm32mp1_clk_rcc_regs_unlock();

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_ACDLLCR,
			DDRPHYC_ACDLLCR_DLLSRST);

	udelay(10);

	mmio_setbits_32(ddrphyc_base + DDRPHYC_ACDLLCR,
			DDRPHYC_ACDLLCR_DLLSRST);

	/* PHY partial init: (DLL lock and ITM reset) */
	mmio_write_32(ddrphyc_base + DDRPHYC_PIR,
		      DDRPHYC_PIR_DLLSRST | DDRPHYC_PIR_DLLLOCK |
		      DDRPHYC_PIR_ITMSRST | DDRPHYC_PIR_INIT);

	/* Need to wait at least 10 clock cycles before accessing PGSR */
	udelay(1);

	/* Pool end of init */
	timeout = timeout_init_us(TIMEOUT_500US);

	while ((mmio_read_32(ddrphyc_base + DDRPHYC_PGSR) &
		DDRPHYC_PGSR_IDONE) == 0U) {
		if (timeout_elapsed(timeout)) {
			return -1;
		}
	}

	do_sw_handshake();

	/* Unmask dfi_init_complete_en to uMCTL2 */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_DFIMISC,
			DDRCTRL_DFIMISC_DFI_INIT_COMPLETE_EN);

	do_sw_ack();

	/* Deactivate sw retention in PWR */
	stm32mp_pwr_regs_lock();
	mmio_clrbits_32(pwr_base + PWR_CR3, PWR_CR3_DDRRETEN);
	stm32mp_pwr_regs_unlock();

	/* Enable PZQ cell (PUBL register) */
	mmio_clrbits_32(ddrphyc_base + DDRPHYC_ZQ0CR0, DDRPHYC_ZQ0CRN_ZQPD);

	/* Enable pad drivers */
	mmio_clrbits_32(ddrphyc_base + DDRPHYC_ACIOCR, DDRPHYC_ACIOCR_ACPDD);

	/* Enable command/address output driver */
	mmio_setbits_32(ddrphyc_base + DDRPHYC_ACIOCR, DDRPHYC_ACIOCR_ACOE);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_ACIOCR,
			DDRPHYC_ACIOCR_CKPDD_MASK);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_ACIOCR,
			DDRPHYC_ACIOCR_CSPDD_MASK);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_DXCCR, DDRPHYC_DXCCR_DXPDD);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_DXCCR, DDRPHYC_DXCCR_DXPDR);

	/* Release latch */
	mmio_setbits_32(ddrphyc_base + DDRPHYC_DSGCR, DDRPHYC_DSGCR_CKOE);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_DSGCR,
			DDRPHYC_DSGCR_ODTPDD_MASK);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_DSGCR, DDRPHYC_DSGCR_NL2PD);

	mmio_clrbits_32(ddrphyc_base + DDRPHYC_DSGCR,
			DDRPHYC_DSGCR_CKEPDD_MASK);

	/* Remove selfrefresh */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_SELFREF_SW);

	/* Wait operating_mode == normal */
	timeout = timeout_init_us(TIMEOUT_500US);
	while ((mmio_read_32(ddrctrl_base + DDRCTRL_STAT) &
		DDRCTRL_STAT_OPERATING_MODE_MASK) !=
	       DDRCTRL_STAT_OPERATING_MODE_NORMAL) {
		if (timeout_elapsed(timeout)) {
			return -1;
		}
	}

	/* AXI ports are no longer blocked from taking transactions */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PCTRL_0,
			DDRCTRL_PCTRL_N_PORT_EN);
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PCTRL_1,
			DDRCTRL_PCTRL_N_PORT_EN);

	stm32mp1_clk_rcc_regs_lock();

	mmio_setbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_AXIDCGEN);

	stm32mp1_clk_rcc_regs_unlock();

	return 0;
}

int ddr_standby_sr_entry(uint32_t *zq0cr0_zdata)
{
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t ddrphyc_base = stm32mp_ddrphyc_base();

	/* Save IOs calibration values */
	if (zq0cr0_zdata != NULL) {
		*zq0cr0_zdata = mmio_read_32(ddrphyc_base + DDRPHYC_ZQ0CR0) &
				DDRPHYC_ZQ0CRN_ZDATA_MASK;
	}

	/* Put DDR in Self-Refresh */
	if (ddr_sw_self_refresh_in() != 0) {
		return -1;
	}

	/* Enable I/O retention mode in standby */
	stm32mp_pwr_regs_lock();
	mmio_setbits_32(pwr_base + PWR_CR3, PWR_CR3_DDRSREN);
	stm32mp_pwr_regs_unlock();

	return 0;
}

static void ddr_sr_mode_ssr(void)
{
	uintptr_t rcc_ddritfcr = stm32mp_rcc_base() + RCC_DDRITFCR;
	uintptr_t ddrctrl_base = stm32mp_ddrctrl_base();

	stm32mp1_clk_rcc_regs_lock();

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC1LPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC2LPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC1EN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC2EN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRCAPBLPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRPHYCAPBLPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRCAPBEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRPHYCAPBEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRPHYCEN);

	mmio_clrbits_32(rcc_ddritfcr, RCC_DDRITFCR_AXIDCGEN);

	mmio_clrbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRCKMOD_MASK);

	stm32mp1_clk_rcc_regs_unlock();

	/* Disable HW LP interface of uMCTL2 */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_HWLPCTL,
			DDRCTRL_HWLPCTL_HW_LP_EN);

	/* Configure Automatic LP modes of uMCTL2 */
	mmio_clrsetbits_32(ddrctrl_base + DDRCTRL_PWRTMG,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_MASK,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_0);

	/*
	 * Disable Clock disable with LP modes
	 * (used in RUN mode for LPDDR2 with specific timing).
	 */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE);

	/* Disable automatic Self-Refresh mode */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_SELFREF_EN);
}

static void ddr_sr_mode_asr(void)
{
	uintptr_t rcc_ddritfcr = stm32mp_rcc_base() + RCC_DDRITFCR;
	uintptr_t ddrctrl_base = stm32mp_ddrctrl_base();

	stm32mp1_clk_rcc_regs_lock();

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_AXIDCGEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC1LPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC2LPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRPHYCLPEN);

	mmio_clrsetbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRCKMOD_MASK,
			   RCC_DDRITFCR_DDRCKMOD_ASR1);

	stm32mp1_clk_rcc_regs_unlock();

	/* Enable HW LP interface of uMCTL2 */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_HWLPCTL,
			DDRCTRL_HWLPCTL_HW_LP_EN);

	/* Configure Automatic LP modes of uMCTL2 */
	mmio_clrsetbits_32(ddrctrl_base + DDRCTRL_PWRTMG,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_MASK,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_0);

	/*
	 * Enable Clock disable with LP modes
	 * (used in RUN mode for LPDDR2 with specific timing).
	 */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE);

	/* Enable automatic Self-Refresh for ASR mode */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_SELFREF_EN);
}

static void ddr_sr_mode_hsr(void)
{
	uintptr_t rcc_ddritfcr = stm32mp_rcc_base() + RCC_DDRITFCR;
	uintptr_t ddrctrl_base = stm32mp_ddrctrl_base();

	stm32mp1_clk_rcc_regs_lock();

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_AXIDCGEN);

	mmio_clrbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC1LPEN);

	mmio_clrbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC2LPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRPHYCLPEN);

	mmio_clrsetbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRCKMOD_MASK,
			   RCC_DDRITFCR_DDRCKMOD_HSR1);

	stm32mp1_clk_rcc_regs_unlock();

	/* Enable HW LP interface of uMCTL2 */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_HWLPCTL,
			DDRCTRL_HWLPCTL_HW_LP_EN);

	/* Configure Automatic LP modes of uMCTL2 */
	mmio_clrsetbits_32(ddrctrl_base + DDRCTRL_PWRTMG,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_MASK,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_0);

	/*
	 * Enable Clock disable with LP modes
	 * (used in RUN mode for LPDDR2 with specific timing).
	 */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE);
}

enum stm32mp1_ddr_sr_mode ddr_read_sr_mode(void)
{
	uint32_t pwrctl = mmio_read_32(stm32mp_ddrctrl_base() + DDRCTRL_PWRCTL);

	switch (pwrctl & (DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE |
			  DDRCTRL_PWRCTL_SELFREF_EN)) {
	case 0U:
		return DDR_SSR_MODE;

	case DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE:
		return DDR_HSR_MODE;

	case DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE | DDRCTRL_PWRCTL_SELFREF_EN:
		return DDR_ASR_MODE;

	default:
		return DDR_SR_MODE_INVALID;
	}
}

void ddr_set_sr_mode(enum stm32mp1_ddr_sr_mode mode)
{
	switch (mode) {
	case DDR_SSR_MODE:
		ddr_sr_mode_ssr();
		break;

	case DDR_HSR_MODE:
		ddr_sr_mode_hsr();
		break;

	case DDR_ASR_MODE:
		ddr_sr_mode_asr();
		break;

	default:
		ERROR("Unknown Self Refresh mode\n");
		panic();
	}
}

void ddr_save_sr_mode(void)
{
	saved_ddr_sr_mode = ddr_read_sr_mode();
}

void ddr_restore_sr_mode(void)
{
	ddr_set_sr_mode(saved_ddr_sr_mode);
}

bool ddr_is_nonsecured_area(uintptr_t address, uint32_t length)
{
	uint64_t pa;

	write_ats1cpw(address);

	isb();

	pa = read64_par();

	if  ((((pa >> PAR_NS_SHIFT) & PAR_NS_MASK) != PAR_NS_MASK) ||
	     (((pa >> PAR_F_SHIFT) & PAR_F_MASK) == PAR_F_MASK)) {
		return false;
	}

	write_ats1cpw(address + length - 1U);

	isb();

	pa = read64_par();

	if  ((((pa >> PAR_NS_SHIFT) & PAR_NS_MASK) == PAR_NS_MASK) &&
	     (((pa >> PAR_F_SHIFT) & PAR_F_MASK) != PAR_F_MASK)) {
		return true;
	}

	return false;
}
