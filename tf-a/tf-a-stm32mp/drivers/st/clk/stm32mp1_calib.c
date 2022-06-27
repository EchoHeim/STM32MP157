/*
 * Copyright (C) 2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>

#include <libfdt.h>

#include <platform_def.h>

#include <arch.h>
#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <drivers/generic_delay_timer.h>
#include <drivers/st/stm32_timer.h>
#include <drivers/st/stm32mp_clkfunc.h>
#include <lib/mmio.h>
#include <lib/spinlock.h>
#include <lib/utils_def.h>
#include <plat/common/platform.h>

#define TIMEOUT_10MS	10000
#define CALIB_TIMEOUT	TIMEOUT_10MS

struct stm32mp1_trim_boundary_t {
	/* Max boundary trim value around forbidden value */
	unsigned int x1;
	/* Min boundary trim value around forbidden value */
	unsigned int x2;
};

struct stm32mp1_clk_cal {
	uint16_t *fbv;
	unsigned int cal_ref;
	int trim_max;
	int trim_min;
	unsigned int boundary_max;
	unsigned long ref_freq;
	unsigned int freq_margin;
	unsigned long (*get_freq)(void);
	void (*set_trim)(unsigned int cal);
	unsigned int (*get_trim)(void);
	struct stm32mp1_trim_boundary_t boundary[16];
};

/* RCC Wakeup status */
static bool rcc_wakeup;

/* List of forbiden values for HSI */
static uint16_t fbv_hsi[] = {
	512,
	480,
	448,
	416,
	384,
	352,
	320,
	288,
	256,
	224,
	192,
	160,
	128,
	96,
	64,
	32,
	0
};

/* List of forbiden values for CSI */
static uint16_t fbv_csi[] = {
	256,
	240,
	224,
	208,
	192,
	176,
	160,
	144,
	128,
	112,
	96,
	80,
	64,
	48,
	32,
	16,
	0
};

static void hsi_set_trim(unsigned int cal);
static unsigned int hsi_get_trimed_cal(void);
static void csi_set_trim(unsigned int cal);
static unsigned int csi_get_trimed_cal(void);

static struct stm32mp1_clk_cal stm32mp1_clk_cal_hsi = {
	.fbv = fbv_hsi,
	.trim_max = 63,
	.trim_min = -64,
	.ref_freq = 0,
	.freq_margin = 5,
	.set_trim = hsi_set_trim,
	.get_trim = hsi_get_trimed_cal,
};

static struct stm32mp1_clk_cal stm32mp1_clk_cal_csi = {
	.fbv = fbv_csi,
	.trim_max = 15,
	.trim_min = -16,
	.ref_freq = 0,
	.freq_margin = 8,
	.set_trim = csi_set_trim,
	.get_trim = csi_get_trimed_cal,
};

static uint32_t timer_val;

/*
 * HSI Calibration part
 */
static int get_signed_value(uint8_t val)
{
	return ((int8_t)(val << 1)) >> 1;
}

static void hsi_set_trim(unsigned int cal)
{
	int clk_trim = (int)cal - (int)stm32mp1_clk_cal_hsi.cal_ref;
	uint32_t trim = ((uint32_t)clk_trim << RCC_HSICFGR_HSITRIM_SHIFT) &
			RCC_HSICFGR_HSITRIM_MASK;

	mmio_clrsetbits_32(stm32mp_rcc_base() + RCC_HSICFGR,
			   RCC_HSICFGR_HSITRIM_MASK, trim);
}

static unsigned int hsi_get_trimed_cal(void)
{
	uint32_t utrim = (mmio_read_32(stm32mp_rcc_base() + RCC_HSICFGR) &
			  RCC_HSICFGR_HSITRIM_MASK) >>
			 RCC_HSICFGR_HSITRIM_SHIFT;

	int trim = get_signed_value((uint8_t)utrim);

	if (trim + (int)stm32mp1_clk_cal_hsi.cal_ref < 0) {
		return 0;
	}

	return stm32mp1_clk_cal_hsi.cal_ref + trim;
}

static void csi_set_trim(unsigned int cal)
{
	int clk_trim = (int)cal - (int)stm32mp1_clk_cal_csi.cal_ref +
		       stm32mp1_clk_cal_csi.trim_max + 1;
	uint32_t trim = ((uint32_t)clk_trim << RCC_CSICFGR_CSITRIM_SHIFT) &
			RCC_CSICFGR_CSITRIM_MASK;

	mmio_clrsetbits_32(stm32mp_rcc_base() + RCC_CSICFGR,
			   RCC_CSICFGR_CSITRIM_MASK, trim);
}

static unsigned int csi_get_trimed_cal(void)
{
	uint32_t trim = (mmio_read_32(stm32mp_rcc_base() + RCC_CSICFGR) &
			 RCC_CSICFGR_CSITRIM_MASK) >>
			RCC_CSICFGR_CSITRIM_SHIFT;

	return (int)trim - stm32mp1_clk_cal_csi.trim_max +
		(int)stm32mp1_clk_cal_csi.cal_ref - 1;
}

static unsigned int trim_increase(struct stm32mp1_clk_cal *clk_cal,
				  unsigned int cal)
{
	struct stm32mp1_trim_boundary_t *boundary;
	unsigned int new_cal;
	int i;

	/* By default: last calibration value */
	new_cal = cal;

	/* Start from Lowest cal value */
	for (i = (int)clk_cal->boundary_max - 1; i >= 0; i--) {
		boundary = &clk_cal->boundary[i];

		if (cal < boundary->x2) {
			new_cal = boundary->x2;
			break;
		}

		if ((cal >= boundary->x2) && (cal < boundary->x1)) {
			new_cal = cal + 1;
			break;
		}
	}

	return new_cal;
}

static unsigned int trim_decrease(struct stm32mp1_clk_cal *clk_cal,
				  unsigned int cal)
{
	struct stm32mp1_trim_boundary_t *boundary;
	unsigned int new_cal;
	unsigned int i;

	/* By default: last calibration value */
	new_cal = cal;

	/* Start from Highest cal value */
	for (i = 0; i < clk_cal->boundary_max; i++) {
		boundary = &clk_cal->boundary[i];

		if (cal > boundary->x1) {
			new_cal = boundary->x1;
			break;
		}

		if ((cal > boundary->x2) && (cal <= boundary->x1)) {
			new_cal = cal - 1;
			break;
		}
	}

	return new_cal;
}

static void rcc_calibration(struct stm32mp1_clk_cal *clk_cal)
{
	unsigned long freq = clk_cal->get_freq();
	unsigned long min = clk_cal->ref_freq -
		((clk_cal->ref_freq * clk_cal->freq_margin) / 1000);
	unsigned long max = clk_cal->ref_freq +
		((clk_cal->ref_freq * clk_cal->freq_margin) / 1000);
	int trim, new_trim;
	unsigned long conv;
	unsigned long min_conv = ULONG_MAX;
	uint64_t start;

	if ((freq >= min) && (freq <= max)) {
		return;
	}

	trim = clk_cal->get_trim();
	start = timeout_init_us(CALIB_TIMEOUT);
	do {
		if (freq < clk_cal->ref_freq) {
			new_trim = trim_increase(clk_cal, trim);
		} else {
			new_trim = trim_decrease(clk_cal, trim);
		}

		clk_cal->set_trim(new_trim);
		freq = clk_cal->get_freq();
		if (freq == 0U) {
			/* Calibration will be stopped */
			clk_cal->ref_freq = 0U;
			return;
		}
		conv = (clk_cal->ref_freq < freq) ?
			freq - clk_cal->ref_freq : clk_cal->ref_freq - freq;
		if (conv < min_conv) {
			min_conv = conv;
			trim = new_trim;
		}

		if (timeout_elapsed(start)) {
			break;
		}

	} while (conv == min_conv);

	clk_cal->set_trim(trim);
	freq = clk_cal->get_freq();

	if ((freq < min) || (freq > max)) {
		ERROR("%s Calibration : Freq %lu, trim %i\n",
		      (clk_cal->set_trim == hsi_set_trim) ? "HSI" : "CSI",
		      freq, trim);
#if DEBUG
		/*
		 * Show the steps around the selected trim value
		 * to correct the margin if needed
		 */
		new_trim = trim_decrease(clk_cal, trim);
		clk_cal->set_trim(new_trim);
		ERROR("%s Calibration : Freq %lu, trim %i\n",
		      (clk_cal->set_trim == hsi_set_trim) ?
		      "HSI" : "CSI", clk_cal->get_freq(), new_trim);

		new_trim = trim_increase(clk_cal, trim);
		clk_cal->set_trim(new_trim);
		ERROR("%s Calibration : Freq %lu, trim %i\n",
		      (clk_cal->set_trim == hsi_set_trim) ?
		      "HSI" : "CSI", clk_cal->get_freq(), new_trim);
#endif
	}
}

static void save_trim(struct stm32mp1_clk_cal *clk_cal,
		      unsigned int i, unsigned int x1, unsigned int x2)
{
	clk_cal->boundary[i].x1 = x1;
	clk_cal->boundary[i].x2 = x2;
}

static int trim_find_prev_boundary(struct stm32mp1_clk_cal *clk_cal,
				   unsigned int x1)
{
	unsigned int x = x1;
	unsigned long freq;

	clk_cal->set_trim(x1 + 1);
	freq = clk_cal->get_freq();

	while (x >= (clk_cal->cal_ref + clk_cal->trim_min)) {
		x--;
		clk_cal->set_trim(x);

		if (clk_cal->get_freq() <= freq) {
			break;
		}
	};

	return x;
}

static void trim_table_init(struct stm32mp1_clk_cal *clk_cal)
{
	uint16_t *trim_fbv = clk_cal->fbv;
	unsigned int min;
	unsigned int max;
	int boundary = 0;
	int i = 0;

	max = clk_cal->cal_ref + clk_cal->trim_max;
	min = clk_cal->cal_ref + clk_cal->trim_min;

	while (trim_fbv[i]) {
		unsigned int x;
		unsigned int x1 = trim_fbv[i];
		unsigned int x2 = trim_fbv[i + 1];

		if ((max <= x2) || (min >= x1)) {
			i++;
			if (boundary != 0) {
				goto out;
			}
			continue;
		}

		/* Take forbiden value + 1 */
		x2 = x2 + 1;
		if (x2 < min) {
			x2 = min;
		}

		if (boundary == 0) {
			/* Save first boundary */
			save_trim(clk_cal, boundary, max, x2);
			boundary++;
			i++;
			continue;
		}

		x = trim_find_prev_boundary(clk_cal, x1);
		/* Save boundary values */
		save_trim(clk_cal, boundary, x - 1, x2);
		boundary++;
		i++;
	};
out:
	clk_cal->boundary_max = boundary;
}

bool stm32mp1_calib_get_wakeup(void)
{
	return rcc_wakeup;
}

void stm32mp1_calib_set_wakeup(bool state)
{
	rcc_wakeup = state;
}

void stm32mp1_calib_it_handler(uint32_t id)
{
	uintptr_t rcc_base = stm32mp_rcc_base();

	switch (id) {
	case STM32MP1_IRQ_RCC_WAKEUP:
		plat_ic_set_priority_mask(GIC_HIGHEST_NS_PRIORITY);
		mmio_setbits_32(rcc_base + RCC_MP_CIFR, RCC_MP_CIFR_WKUPF);
		stm32mp1_calib_set_wakeup(true);
		return;

	case STM32MP1_IRQ_MCU_SEV:
		stm32mp1_calib_set_wakeup(false);
		if ((mmio_read_32(EXTI_BASE + EXTI_RPR3) &
		     EXTI_RPR3_RPIF65) != 0U) {
			mmio_setbits_32(EXTI_BASE + EXTI_RPR3,
					EXTI_RPR3_RPIF65);
		}

		if ((mmio_read_32(EXTI_BASE + EXTI_FPR3) &
		     EXTI_FPR3_FPIF65) != 0U) {
			mmio_setbits_32(EXTI_BASE + EXTI_FPR3,
					EXTI_FPR3_FPIF65);
		}

		break;

	case ARM_IRQ_SEC_PHY_TIMER:
	default:
		break;
	}

	if (stm32mp1_clk_cal_hsi.ref_freq != 0U) {
		rcc_calibration(&stm32mp1_clk_cal_hsi);
	}

	if (stm32mp1_clk_cal_csi.ref_freq != 0U) {
		rcc_calibration(&stm32mp1_clk_cal_csi);
	}

	if (timer_val != 0U) {
		write_cntp_tval(timer_val);
	}
}

int stm32mp1_calib_start_hsi_cal(void)
{
	if (stm32mp1_clk_cal_hsi.ref_freq == 0U) {
		return -ENOENT;
	}

	rcc_calibration(&stm32mp1_clk_cal_hsi);
	return 0;
}

int stm32mp1_calib_start_csi_cal(void)
{
	if (stm32mp1_clk_cal_csi.ref_freq == 0U) {
		return -ENOENT;
	}

	rcc_calibration(&stm32mp1_clk_cal_csi);
	return 0;
}

static void init_hsi_cal(void)
{
	int len;

	if (fdt_rcc_read_prop("st,hsi-cal", &len) == NULL) {
		return;
	}

	stm32_timer_freq_func(&stm32mp1_clk_cal_hsi.get_freq, HSI_CAL);
	if (stm32mp1_clk_cal_hsi.get_freq == NULL) {
		return;
	}

	stm32mp1_clk_cal_hsi.ref_freq = stm32mp_clk_get_rate(CK_HSI);

	/* Read initial value */
	stm32mp1_clk_cal_hsi.cal_ref =
		((mmio_read_32(stm32mp_rcc_base() + RCC_HSICFGR)
		  & RCC_HSICFGR_HSICAL_MASK) >> RCC_HSICFGR_HSICAL_SHIFT);

	trim_table_init(&stm32mp1_clk_cal_hsi);

	stm32mp1_clk_cal_hsi.set_trim(stm32mp1_clk_cal_hsi.cal_ref);

	rcc_calibration(&stm32mp1_clk_cal_hsi);
}

static void init_csi_cal(void)
{
	int len;

	if (fdt_rcc_read_prop("st,csi-cal", &len) == NULL) {
		return;
	}

	stm32_timer_freq_func(&stm32mp1_clk_cal_csi.get_freq, CSI_CAL);
	if (stm32mp1_clk_cal_csi.get_freq == NULL) {
		return;
	}

	stm32mp1_clk_cal_csi.ref_freq = stm32mp_clk_get_rate(CK_CSI);

	/* Read initial value */
	stm32mp1_clk_cal_csi.cal_ref =
		((mmio_read_32(stm32mp_rcc_base() + RCC_CSICFGR) &
		  RCC_CSICFGR_CSICAL_MASK) >> RCC_CSICFGR_CSICAL_SHIFT);

	trim_table_init(&stm32mp1_clk_cal_csi);

	stm32mp1_clk_cal_csi.set_trim(stm32mp1_clk_cal_csi.cal_ref);

	rcc_calibration(&stm32mp1_clk_cal_csi);
}

void stm32mp1_calib_init(void)
{
	init_hsi_cal();
	init_csi_cal();

	timer_val = fdt_rcc_read_uint32_default("st,cal-sec", 0) *
		plat_get_syscnt_freq2();

	if (timer_val != 0U) {
		/* Load & enable timer */
		write_cntp_tval(timer_val);
		write_cntp_ctl(BIT(0));
	};

	if (fdt_rcc_enable_it("mcu_sev") < 0) {
		VERBOSE("No MCU calibration\n");
	}
}
