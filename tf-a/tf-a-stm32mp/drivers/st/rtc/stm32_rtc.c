/*
 * Copyright (c) 2018-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/st/stm32_rtc.h>
#include <drivers/st/stm32mp_clkfunc.h>
#include <lib/mmio.h>
#include <lib/spinlock.h>

#define RTC_COMPAT		"st,stm32mp1-rtc"

#define RTC_TR_SU_MASK		GENMASK(3, 0)
#define RTC_TR_ST_MASK		GENMASK(6, 4)
#define RTC_TR_ST_SHIFT		4
#define RTC_TR_MNU_MASK		GENMASK(11, 8)
#define RTC_TR_MNU_SHIFT	8
#define RTC_TR_MNT_MASK		GENMASK(14, 12)
#define RTC_TR_MNT_SHIFT	12
#define RTC_TR_HU_MASK		GENMASK(19, 16)
#define RTC_TR_HU_SHIFT		16
#define RTC_TR_HT_MASK		GENMASK(21, 20)
#define RTC_TR_HT_SHIFT		20
#define RTC_TR_PM		BIT(22)

#define RTC_DR_DU_MASK		GENMASK(3, 0)
#define RTC_DR_DT_MASK		GENMASK(5, 4)
#define RTC_DR_DT_SHIFT		4
#define RTC_DR_MU_MASK		GENMASK(11, 8)
#define RTC_DR_MU_SHIFT		8
#define RTC_DR_MT		BIT(12)
#define RTC_DR_MT_SHIFT		12
#define RTC_DR_WDU_MASK		GENMASK(15, 13)
#define RTC_DR_WDU_SHIFT	13
#define RTC_DR_YU_MASK		GENMASK(19, 16)
#define RTC_DR_YU_SHIFT		16
#define RTC_DR_YT_MASK		GENMASK(23, 20)
#define RTC_DR_YT_SHIFT		20

#define RTC_SSR_SS_MASK		GENMASK(15, 0)

#define RTC_ICSR_ALRAWF		BIT(0)
#define RTC_ICSR_RSF		BIT(5)

#define RTC_PRER_PREDIV_S_MASK	GENMASK(14, 0)

#define RTC_CR_BYPSHAD		BIT(5)
#define RTC_CR_BYPSHAD_SHIFT	5
#define RTC_CR_ALRAE		BIT(8)
#define RTC_CR_ALRAIE		BIT(12)
#define RTC_CR_TAMPTS		BIT(25)

#define RTC_SMCR_TS_DPROT	BIT(3)

#define RTC_TSDR_DU_MASK	GENMASK(3, 0)
#define RTC_TSDR_DU_SHIFT	0
#define RTC_TSDR_DT_MASK	GENMASK(5, 4)
#define RTC_TSDR_DT_SHIFT	4
#define RTC_TSDR_MU_MASK	GENMASK(11, 8)
#define RTC_TSDR_MU_SHIFT	8

#define RTC_ALRMAR_DU_SHIFT	24

#define RTC_SR_TSF		BIT(3)
#define RTC_SR_TSOVF		BIT(4)

#define RTC_SCR_CTSF		BIT(3)
#define RTC_SCR_CTSOVF		BIT(4)

#define RTC_WPR_KEY1		0xCA
#define RTC_WPR_KEY2		0x53
#define RTC_WPR_KEY_LOCK	0xFF

static struct dt_node_info rtc_dev;

static struct spinlock lock;

void stm32_rtc_regs_lock(void)
{
	if (stm32mp_lock_available()) {
		spin_lock(&lock);
	}
}

void stm32_rtc_regs_unlock(void)
{
	if (stm32mp_lock_available()) {
		spin_unlock(&lock);
	}
}

static void stm32_rtc_write_unprotect(void)
{
	mmio_write_32(rtc_dev.base + RTC_WPR, RTC_WPR_KEY1);
	mmio_write_32(rtc_dev.base + RTC_WPR, RTC_WPR_KEY2);
}

static void stm32_rtc_write_protect(void)
{
	mmio_write_32(rtc_dev.base + RTC_WPR, RTC_WPR_KEY_LOCK);
}

/*******************************************************************************
 * This function gets the BYPSHAD bit value of the RTC_CR register.
 * It will determine if we need to reset RTC_ISCR.RSF after each RTC calendar
 * read, and also wait for RTC_ISCR.RSF=1 before next read.
 * Returns true or false depending on the bit value.
 ******************************************************************************/
static bool stm32_rtc_get_bypshad(void)
{
	return ((mmio_read_32(rtc_dev.base + RTC_CR) & RTC_CR_BYPSHAD) >>
		RTC_CR_BYPSHAD_SHIFT) != 0U;
}

/*******************************************************************************
 * This function reads the RTC calendar register values.
 * If shadow registers are not bypassed, then a reset/poll is done.
 ******************************************************************************/
static void stm32_rtc_read_calendar(struct stm32_rtc_calendar *calendar)
{
	bool bypshad = stm32_rtc_get_bypshad();

	if (!bypshad) {
		mmio_clrbits_32((uint32_t)(rtc_dev.base + RTC_ICSR),
				RTC_ICSR_RSF);
		while ((mmio_read_32(rtc_dev.base + RTC_ICSR) & RTC_ICSR_RSF) !=
		       RTC_ICSR_RSF) {
			;
		}
	}

	calendar->ssr = mmio_read_32(rtc_dev.base + RTC_SSR);
	calendar->tr = mmio_read_32(rtc_dev.base + RTC_TR);
	calendar->dr = mmio_read_32(rtc_dev.base + RTC_DR);
}

/*******************************************************************************
 * This function fill the rtc_time structure based on rtc_calendar register.
 ******************************************************************************/
static void stm32_rtc_get_time(struct stm32_rtc_calendar *cal,
			       struct stm32_rtc_time *tm)
{
	assert(cal != NULL);
	assert(tm != NULL);

	tm->hour = (((cal->tr & RTC_TR_HT_MASK) >> RTC_TR_HT_SHIFT) * 10U) +
		((cal->tr & RTC_TR_HU_MASK) >> RTC_TR_HU_SHIFT);

	if ((cal->tr & RTC_TR_PM) != 0U) {
		tm->hour += 12U;
	}

	tm->min = (((cal->tr & RTC_TR_MNT_MASK) >> RTC_TR_MNT_SHIFT) * 10U) +
		  ((cal->tr & RTC_TR_MNU_MASK) >> RTC_TR_MNU_SHIFT);
	tm->sec = (((cal->tr & RTC_TR_ST_MASK) >> RTC_TR_ST_SHIFT) * 10U) +
		  (cal->tr & RTC_TR_SU_MASK);
}

/*******************************************************************************
 * This function fill the rtc_time structure with the given date register.
 ******************************************************************************/
static void stm32_rtc_get_date(struct stm32_rtc_calendar *cal,
			       struct stm32_rtc_time *tm)
{
	assert(cal != NULL);
	assert(tm != NULL);

	tm->wday = (((cal->dr & RTC_DR_WDU_MASK) >> RTC_DR_WDU_SHIFT));

	tm->day = (((cal->dr & RTC_DR_DT_MASK) >> RTC_DR_DT_SHIFT) * 10U) +
		  (cal->dr & RTC_DR_DU_MASK);

	tm->month = (((cal->dr & RTC_DR_MT) >> RTC_DR_MT_SHIFT) * 10U) +
		    ((cal->dr & RTC_DR_MU_MASK) >> RTC_DR_MU_SHIFT);

	tm->year = (((cal->dr & RTC_DR_YT_MASK) >> RTC_DR_YT_SHIFT) * 10U) +
		   ((cal->dr & RTC_DR_YU_MASK) >> RTC_DR_YU_SHIFT) + 2000U;
}

/*******************************************************************************
 * This function reads the RTC timestamp register values and update time
 * structure with the corresponding value.
 ******************************************************************************/
static void stm32_rtc_read_timestamp(struct stm32_rtc_time *time)
{
	assert(time != NULL);

	struct stm32_rtc_calendar cal_tamp;

	cal_tamp.tr = mmio_read_32(rtc_dev.base + RTC_TSTR);
	cal_tamp.dr = mmio_read_32(rtc_dev.base + RTC_TSDR);
	stm32_rtc_get_time(&cal_tamp, time);
	stm32_rtc_get_date(&cal_tamp, time);
}

/*******************************************************************************
 * This function gets the RTC calendar register values.
 * It takes into account the need of reading twice or not, depending on
 * frequencies previously setted, and the bypass or not of the shadow
 * registers. This service is exposed externally.
 ******************************************************************************/
void stm32_rtc_get_calendar(struct stm32_rtc_calendar *calendar)
{
	bool read_twice = stm32mp1_rtc_get_read_twice();

	stm32_rtc_regs_lock();
	stm32mp_clk_enable(rtc_dev.clock);

	stm32_rtc_read_calendar(calendar);

	if (read_twice) {
		uint32_t tr_save = calendar->tr;

		stm32_rtc_read_calendar(calendar);

		if (calendar->tr != tr_save) {
			stm32_rtc_read_calendar(calendar);
		}
	}

	stm32mp_clk_disable(rtc_dev.clock);
	stm32_rtc_regs_unlock();
}

/*******************************************************************************
 * This function computes the second fraction in milliseconds.
 * The returned value is a uint32_t between 0 and 1000.
 ******************************************************************************/
static uint32_t stm32_rtc_get_second_fraction(struct stm32_rtc_calendar *cal)
{
	uint32_t prediv_s = mmio_read_32(rtc_dev.base + RTC_PRER) &
			    RTC_PRER_PREDIV_S_MASK;
	uint32_t ss = cal->ssr & RTC_SSR_SS_MASK;

	return ((prediv_s - ss) * 1000U) / (prediv_s + 1U);
}

/*******************************************************************************
 * This function computes the fraction difference between two timestamps.
 * Here again the returned value is in milliseconds.
 ******************************************************************************/
static signed long long stm32_rtc_diff_frac(struct stm32_rtc_calendar *cur,
					    struct stm32_rtc_calendar *ref)
{
	return (signed long long)stm32_rtc_get_second_fraction(cur) -
	       (signed long long)stm32_rtc_get_second_fraction(ref);
}

/*******************************************************************************
 * This function computes the time difference between two timestamps.
 * It includes seconds, minutes and hours.
 * Here again the returned value is in milliseconds.
 ******************************************************************************/
static signed long long stm32_rtc_diff_time(struct stm32_rtc_time *current,
					    struct stm32_rtc_time *ref)
{
	signed long long curr_s;
	signed long long ref_s;

	curr_s = (signed long long)current->sec +
		 (((signed long long)current->min +
		  (((signed long long)current->hour * 60))) * 60);

	ref_s = (signed long long)ref->sec +
		(((signed long long)ref->min +
		 (((signed long long)ref->hour * 60))) * 60);

	return (curr_s - ref_s) * 1000;
}

/*******************************************************************************
 * This function determines if the year is leap or not.
 * Returned value is true or false.
 ******************************************************************************/
static bool stm32_is_a_leap_year(uint32_t year)
{
	return ((year % 4U) == 0U) &&
	       (((year % 100U) != 0U) || ((year % 400U) == 0U));
}

/*******************************************************************************
 * This function computes the date difference between two timestamps.
 * It includes days, months, years, with exceptions.
 * Here again the returned value is in milliseconds.
 ******************************************************************************/
static signed long long stm32_rtc_diff_date(struct stm32_rtc_time *current,
					    struct stm32_rtc_time *ref)
{
	uint32_t diff_in_days = 0;
	uint32_t m;
	static const uint8_t month_len[NB_MONTHS] = {
		31, 28, 31, 30, 31, 30,
		31, 31, 30, 31, 30, 31
	};

	/* Get the number of non-entire month days */
	if (current->day >= ref->day) {
		diff_in_days += current->day - ref->day;
	} else {
		diff_in_days += (uint32_t)month_len[ref->month - 1U] -
				ref->day + current->day;
	}

	/* Get the number of entire months, and compute the related days */
	if (current->month > (ref->month + 1U)) {
		for (m = (ref->month + 1U); (m < current->month) &&
		     (m < 12U); m++) {
			diff_in_days += (uint32_t)month_len[m - 1U];
		}
	}

	if (current->month < (ref->month - 1U)) {
		for (m = 1U; (m < current->month) && (m < 12U); m++) {
			diff_in_days += (uint32_t)month_len[m - 1U];
		}

		for (m = (ref->month + 1U); m < 12U; m++) {
			diff_in_days += (uint32_t)month_len[m - 1U];
		}
	}

	/* Get complete years */
	if (current->year > (ref->year + 1U)) {
		diff_in_days += (current->year - ref->year - 1U) * 365U;
	}

	/* Particular cases: leap years (one day more) */
	if (diff_in_days > 0U) {
		if (current->year == ref->year) {
			if (stm32_is_a_leap_year(current->year)) {
				if ((ref->month <= 2U) &&
				    (current->month >= 3U) &&
				    (current->day <= 28U)) {
					diff_in_days++;
				}
			}
		} else {
			uint32_t y;

			/* Ref year is leap */
			if ((stm32_is_a_leap_year(ref->year)) &&
			    (ref->month <= 2U) && (ref->day <= 28U)) {
				diff_in_days++;
			}

			/* Current year is leap */
			if ((stm32_is_a_leap_year(current->year)) &&
			    (current->month >= 3U)) {
				diff_in_days++;
			}

			/* Interleaved years are leap */
			for (y = ref->year + 1U; y < current->year; y++) {
				if (stm32_is_a_leap_year(y)) {
					diff_in_days++;
				}
			}
		}
	}

	return (24 * 60 * 60 * 1000) * (signed long long)diff_in_days;
}

/*******************************************************************************
 * This function computes the date difference between two rtc value.
 * Here again the returned value is in milliseconds.
 ******************************************************************************/
unsigned long long stm32_rtc_diff_calendar(struct stm32_rtc_calendar *cur,
					   struct stm32_rtc_calendar *ref)
{
	signed long long diff_in_ms = 0;
	struct stm32_rtc_time curr_t;
	struct stm32_rtc_time ref_t;

	stm32mp_clk_enable(rtc_dev.clock);

	stm32_rtc_get_date(cur, &curr_t);
	stm32_rtc_get_date(ref, &ref_t);
	stm32_rtc_get_time(cur, &curr_t);
	stm32_rtc_get_time(ref, &ref_t);

	diff_in_ms += stm32_rtc_diff_frac(cur, ref);
	diff_in_ms += stm32_rtc_diff_time(&curr_t, &ref_t);
	diff_in_ms += stm32_rtc_diff_date(&curr_t, &ref_t);

	stm32mp_clk_disable(rtc_dev.clock);

	return (unsigned long long)diff_in_ms;
}

/*******************************************************************************
 * This function fill the RTC timestamp structure.
 ******************************************************************************/
void stm32_rtc_get_timestamp(struct stm32_rtc_time *tamp_ts)
{
	stm32_rtc_regs_lock();
	stm32mp_clk_enable(rtc_dev.clock);

	if ((mmio_read_32(rtc_dev.base + RTC_SR) & RTC_SR_TSF) != 0U) {
		/* Print timestamp for tamper event */
		stm32_rtc_read_timestamp(tamp_ts);
		mmio_setbits_32(rtc_dev.base + RTC_SCR, RTC_SCR_CTSF);
		if ((mmio_read_32(rtc_dev.base + RTC_SR) & RTC_SR_TSOVF) !=
		    0U) {
			/* Overflow detected */
			mmio_setbits_32(rtc_dev.base + RTC_SCR, RTC_SCR_CTSOVF);
		}
	}

	stm32mp_clk_disable(rtc_dev.clock);
	stm32_rtc_regs_unlock();
}

/*******************************************************************************
 * This function enable the timestamp bit for tamper and secure timestamp
 * access.
 ******************************************************************************/
void stm32_rtc_set_tamper_timestamp(void)
{
	stm32_rtc_regs_lock();
	stm32mp_clk_enable(rtc_dev.clock);

	stm32_rtc_write_unprotect();

	/* Enable tamper timestamper */
	mmio_setbits_32(rtc_dev.base + RTC_CR, RTC_CR_TAMPTS);

	/* Secure Timestamp bit */
	mmio_clrbits_32(rtc_dev.base + RTC_SMCR, RTC_SMCR_TS_DPROT);

	stm32_rtc_write_protect();

	stm32mp_clk_disable(rtc_dev.clock);
	stm32_rtc_regs_unlock();
}

/*******************************************************************************
 * This function return state of tamper timestamp.
 ******************************************************************************/
bool stm32_rtc_is_timestamp_enable(void)
{
	bool ret;

	stm32mp_clk_enable(rtc_dev.clock);

	ret = (mmio_read_32(rtc_dev.base + RTC_CR) & RTC_CR_TAMPTS) != 0U;

	stm32mp_clk_disable(rtc_dev.clock);

	return ret;
}

/*******************************************************************************
 * RTC initialisation function.
 ******************************************************************************/
int stm32_rtc_init(void)
{
	int node;

	node = dt_get_node(&rtc_dev, -1, RTC_COMPAT);
	if (node < 0) {
		return node;
	}

	if (rtc_dev.status == DT_SECURE) {
		stm32mp_register_secure_periph_iomem(rtc_dev.base);
	} else {
		stm32mp_register_non_secure_periph_iomem(rtc_dev.base);
	}

	return 0;
}
