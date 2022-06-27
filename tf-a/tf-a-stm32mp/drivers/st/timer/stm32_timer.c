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

#include <drivers/delay_timer.h>
#include <drivers/st/stm32_timer.h>
#include <lib/mmio.h>

#define TIM_CR1			0x00U		/* Control Register 1      */
#define TIM_CR2			0x04U		/* Control Register 2      */
#define TIM_SMCR		0x08U		/* Slave mode control reg  */
#define TIM_DIER		0x0CU		/* DMA/interrupt register  */
#define TIM_SR			0x10U		/* Status register	   */
#define TIM_EGR			0x14U		/* Event Generation Reg    */
#define TIM_CCMR1		0x18U		/* Capt/Comp 1 Mode Reg    */
#define TIM_CCMR2		0x1CU		/* Capt/Comp 2 Mode Reg    */
#define TIM_CCER		0x20U		/* Capt/Comp Enable Reg    */
#define TIM_CNT			0x24U		/* Counter		   */
#define TIM_PSC			0x28U		/* Prescaler               */
#define TIM_ARR			0x2CU		/* Auto-Reload Register    */
#define TIM_CCR1		0x34U		/* Capt/Comp Register 1    */
#define TIM_CCR2		0x38U		/* Capt/Comp Register 2    */
#define TIM_CCR3		0x3CU		/* Capt/Comp Register 3    */
#define TIM_CCR4		0x40U		/* Capt/Comp Register 4    */
#define TIM_BDTR		0x44U		/* Break and Dead-Time Reg */
#define TIM_DCR			0x48U		/* DMA control register    */
#define TIM_DMAR		0x4CU		/* DMA transfer register   */
#define TIM_AF1			0x60U		/* Alt Function Reg 1      */
#define TIM_AF2			0x64U		/* Alt Function Reg 2      */
#define TIM_TISEL		0x68U		/* Input Selection         */

#define TIM_CR1_CEN		BIT(0)
#define TIM_SMCR_SMS		GENMASK(2, 0)	/* Slave mode selection */
#define TIM_SMCR_TS		GENMASK(6, 4)	/* Trigger selection */
#define TIM_CCMR_CC1S_TI1	BIT(0)		/* IC1/IC3 selects TI1/TI3 */
#define TIM_CCMR_CC1S_TI2	BIT(1)		/* IC1/IC3 selects TI2/TI4 */
#define TIM_CCMR_CC2S_TI2	BIT(8)		/* IC2/IC4 selects TI2/TI4 */
#define TIM_CCMR_CC2S_TI1	BIT(9)		/* IC2/IC4 selects TI1/TI3 */
#define TIM_CCER_CC1E		BIT(0)		/* Capt/Comp 1  out Ena    */
#define TIM_CCER_CC1P		BIT(1)		/* Capt/Comp 1  Polarity   */
#define TIM_CCER_CC1NP		BIT(3)		/* Capt/Comp 1N Polarity   */
#define TIM_SR_UIF		BIT(0)		/* UIF interrupt flag      */
#define TIM_SR_CC1IF		BIT(1)		/* CC1 interrupt flag      */
#define TIM_TISEL_TI1SEL_MASK	GENMASK(3, 0)
#define TIM_SMCR_SMS_RESET	0x4U
#define TIM_SMCR_TS_SHIFT	4U
#define TIM_SMCR_TS_TI1FP1	0x5U

#define TIM_COMPAT		"st,stm32-timers"
#define TIM_TIMEOUT_US		100000
#define TIM_TIMEOUT_STEP_US	10
#define TIM_PRESCAL_HSI		10U
#define TIM_PRESCAL_CSI		7U
#define TIM_MIN_FREQ_CALIB	50000000U

struct stm32_timer_instance {
	uintptr_t base;
	unsigned long clk;
	unsigned long freq;
	uint8_t cal_input;
};

static struct stm32_timer_instance stm32_timer[TIM_MAX_INSTANCE];

static int stm32_timer_get_dt_node(struct dt_node_info *info, int offset)
{
	int node;

	node = dt_get_node(info, offset, TIM_COMPAT);
	if (node < 0) {
		if (offset == -1) {
			WARN("%s: No TIMER found\n", __func__);
		}
		return -FDT_ERR_NOTFOUND;
	}

	return node;
}

static int stm32_timer_config(struct stm32_timer_instance *timer)
{
	stm32mp_clk_enable(timer->clk);

	timer->freq = stm32mp_clk_timer_get_rate(timer->clk);

	if (timer->freq < TIM_MIN_FREQ_CALIB) {
		WARN("Timer is not accurate enough for calibration\n");
		stm32mp_clk_disable(timer->clk);
		return -EINVAL;
	}

	if ((mmio_read_32(timer->base + TIM_TISEL) & TIM_TISEL_TI1SEL_MASK) !=
	    timer->cal_input) {
		mmio_clrsetbits_32(timer->base + TIM_CCMR1,
				   TIM_CCMR_CC1S_TI1 | TIM_CCMR_CC1S_TI2,
				   TIM_CCMR_CC1S_TI1);

		mmio_clrbits_32(timer->base + TIM_CCER,
				TIM_CCER_CC1P | TIM_CCER_CC1NP);

		mmio_clrsetbits_32(timer->base + TIM_SMCR,
				   TIM_SMCR_TS | TIM_SMCR_SMS,
				   (TIM_SMCR_TS_TI1FP1 << TIM_SMCR_TS_SHIFT) |
				   TIM_SMCR_SMS_RESET);

		mmio_write_32(timer->base + TIM_TISEL, timer->cal_input);
		mmio_setbits_32(timer->base + TIM_CR1, TIM_CR1_CEN);
		mmio_setbits_32(timer->base + TIM_CCER, TIM_CCER_CC1E);
	}

	stm32mp_clk_disable(timer->clk);

	return 0;
}

static uint32_t stm32_timer_start_capture(struct stm32_timer_instance *timer)
{
	uint32_t timeout = TIM_TIMEOUT_US / TIM_TIMEOUT_STEP_US;
	uint32_t counter = 0U;
	uint32_t old_counter = 0U;
	int twice = 0;

	if (stm32_timer_config(timer) < 0) {
		return 0U;
	}

	stm32mp_clk_enable(timer->clk);

	mmio_write_32(timer->base + TIM_SR, 0U);
	while (((mmio_read_32(timer->base + TIM_SR) &
		 TIM_SR_UIF) == 0U) && (timeout != 0U)) {
		udelay(TIM_TIMEOUT_STEP_US);
		timeout--;
	}

	if (timeout == 0U) {
		goto out;
	}

	mmio_write_32(timer->base + TIM_SR, 0U);

	while ((twice < 2) || (old_counter != counter)) {
		timeout = TIM_TIMEOUT_US / TIM_TIMEOUT_STEP_US;

		while (((mmio_read_32(timer->base + TIM_SR) &
			 TIM_SR_CC1IF) == 0U) && (timeout != 0U)) {
			udelay(TIM_TIMEOUT_STEP_US);
			timeout--;
		}
		if (timeout == 0U) {
			goto out;
		}

		old_counter = counter;
		counter = mmio_read_32(timer->base + TIM_CCR1);
		twice++;
	}

out:
	stm32mp_clk_disable(timer->clk);

	if (timeout == 0U) {
		return 0U;
	}

	return counter;
}

unsigned long stm32_timer_hsi_freq(void)
{
	struct stm32_timer_instance *timer = &stm32_timer[HSI_CAL];
	unsigned long hsi_freq;
	uint32_t counter;

	if (timer->base == 0) {
		return 0;
	}

	counter = stm32_timer_start_capture(timer);
	VERBOSE("Counter value %i\n", counter);

	if (counter == 0U) {
		return 0;
	}

	hsi_freq = (timer->freq / counter) << TIM_PRESCAL_HSI;

	return hsi_freq;
}

unsigned long stm32_timer_csi_freq(void)
{
	struct stm32_timer_instance *timer = &stm32_timer[CSI_CAL];
	unsigned long csi_freq;
	uint32_t counter;

	if (timer->base == 0) {
		return 0;
	}

	counter = stm32_timer_start_capture(timer);
	VERBOSE("Counter value %i\n", counter);

	if (counter == 0U) {
		return 0;
	}

	csi_freq = (timer->freq / counter) << TIM_PRESCAL_CSI;

	return csi_freq;
}

/*
 * Get the timer frequence callback function for a target clock calibration
 * @timer_freq_cb - Output callback function
 * @type - Target clock calibration ID
 */
void stm32_timer_freq_func(unsigned long (**timer_freq_cb)(void),
			   enum timer_cal type)
{
	switch (type) {
	case HSI_CAL:
		if (stm32_timer[HSI_CAL].base != 0) {
			*timer_freq_cb = stm32_timer_hsi_freq;
		}
		break;

	case CSI_CAL:
		if (stm32_timer[CSI_CAL].base != 0) {
			*timer_freq_cb = stm32_timer_csi_freq;
		}
		break;

	default:
		panic();
	}
}

/*
 * Initialize timer from DT
 * return 0 if disabled, 1 if enabled, else < 0
 */
int stm32_timer_init(void)
{
	void *fdt;
	struct dt_node_info dt_timer;
	int node = -1;
	uint8_t nb_timer = 0;

	if (fdt_get_address(&fdt) == 0) {
		return -EPERM;
	}

	for (node = stm32_timer_get_dt_node(&dt_timer, node);
	     node != -FDT_ERR_NOTFOUND;
	     node = stm32_timer_get_dt_node(&dt_timer, node)) {

		if (dt_timer.status == DT_SECURE) {
			struct stm32_timer_instance *timer;
			const fdt32_t *cuint;

			nb_timer++;

			cuint = fdt_getprop(fdt, node, "st,hsi-cal-input",
					    NULL);
			if (cuint != NULL) {
				timer = &stm32_timer[HSI_CAL];
				timer->base = dt_timer.base;
				timer->clk = dt_timer.clock;
				timer->freq =
					stm32mp_clk_timer_get_rate(timer->clk);
				timer->cal_input =
					(uint8_t)fdt32_to_cpu(*cuint);
				if (stm32_timer_config(timer) < 0) {
					timer->base = 0;
					continue;
				}
			}

			cuint = fdt_getprop(fdt, node, "st,csi-cal-input",
					    NULL);
			if (cuint != NULL) {
				timer = &stm32_timer[CSI_CAL];
				timer->base = dt_timer.base;
				timer->clk = dt_timer.clock;
				timer->freq =
					stm32mp_clk_timer_get_rate(timer->clk);
				timer->cal_input =
					(uint8_t)fdt32_to_cpu(*cuint);
				if (stm32_timer_config(timer) < 0) {
					timer->base = 0;
					continue;
				}
			}
		}
	}

	VERBOSE("%u TIMER instance%s found\n", nb_timer,
		nb_timer > 1 ? "s" : "");

	if (nb_timer == 0U) {
		return -FDT_ERR_NOTFOUND;
	}

	return 0;
}
