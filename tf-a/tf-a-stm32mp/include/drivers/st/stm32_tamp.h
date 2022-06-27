/*
 * Copyright (c) 2014-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32_TAMP_H
#define STM32_TAMP_H

/* Internal Tamper */
enum stm32_tamp_int_id {
	ITAMP1 = 0,
	ITAMP2,
	ITAMP3,
	ITAMP4,
	ITAMP5,
	ITAMP6,
	ITAMP7,
	ITAMP8,
	ITAMP9,
	ITAMP10,
	ITAMP11,
	ITAMP12,
	ITAMP13,
	ITAMP14,
	ITAMP15,
	ITAMP16
};

/* External Tamper */
enum stm32_tamp_ext_id {
	EXT_TAMP1 = 0,
	EXT_TAMP2,
	EXT_TAMP3,
	EXT_TAMP4,
	EXT_TAMP5,
	EXT_TAMP6,
	EXT_TAMP7,
	EXT_TAMP8
};

enum stm32_tamp_state {
	TAMP_DISABLE = 0,
	TAMP_ENABLE
};

#define TAMP_UNUSED	{.id = -1}

/* define TAMPER modes */
#define TAMP_TRIG_OFF		0x0U
#define TAMP_TRIG_ON		0x1U
#define TAMP_ACTIVE		0x2U
#define TAMP_ERASE		0x0U
#define TAMP_NOERASE		0x1U
#define TAMP_NO_EVT_MASK	0x0U
#define TAMP_EVT_MASK		0x1U

/* define Passive FILTER mode */
#define TAMP_FILTER_PRECHARGE		0x0U
#define TAMP_FILTER_PULL_UP_DISABLE	0x1U
#define TAMP_FILTER_DURATION_1_CYCLE	0x0U
#define TAMP_FILTER_DURATION_2_CYCLES	0x1U
#define TAMP_FILTER_DURATION_3_CYCLES	0x2U
#define TAMP_FILTER_DURATION_4_CYCLES	0x3U
#define TAMP_FILTER_COUNT_1		0x0U
#define TAMP_FILTER_COUNT_2		0x1U
#define TAMP_FILTER_COUNT_4		0x2U
#define TAMP_FILTER_COUNT_8		0x3U
#define TAMP_FILTER_SAMPLING_32768	0x0U
#define TAMP_FILTER_SAMPLING_16384	0x1U
#define TAMP_FILTER_SAMPLING_8192	0x2U
#define TAMP_FILTER_SAMPLING_4096	0x3U
#define TAMP_FILTER_SAMPLING_2048	0x4U
#define TAMP_FILTER_SAMPLING_1024	0x5U
#define TAMP_FILTER_SAMPLING_512	0x6U
#define TAMP_FILTER_SAMPLING_256	0x7U

/*  define active filter */
#define TAMP_ACTIVE_FILTER_OFF		0x0U
#define TAMP_ACTIVE_FILTER_ON		0x1U
#define TAMP_ACTIVE_ATO_DEDICTED	0x0U
#define TAMP_ACTIVE_ATO_TAMPOUTSEL	0x1U
#define TAMP_ACTIVE_APER_1_OUTPUT	0x0U
#define TAMP_ACTIVE_APER_2_OUTPUTS	0x1U
#define TAMP_ACTIVE_APER_3_4_OUTPUTS	0x2U
#define TAMP_ACTIVE_APER_5_OUTPUTS	0x3U
#define TAMP_ACTIVE_CKSEL_DIV_0		0x0U
#define TAMP_ACTIVE_CKSEL_DIV_2		0x1U
#define TAMP_ACTIVE_CKSEL_DIV_4		0x2U
#define TAMP_ACTIVE_CKSEL_DIV_8		0x3U
#define TAMP_ACTIVE_CKSEL_DIV_16	0x4U
#define TAMP_ACTIVE_CKSEL_DIV_32	0x5U
#define TAMP_ACTIVE_CKSEL_DIV_64	0x6U
#define TAMP_ACTIVE_CKSEL_DIV_128	0x7U
#define TAMP_ACTIVE_ATOSEL_OUT1_(X)	(0x0U << ((X) * 2))
#define TAMP_ACTIVE_ATOSEL_OUT2_(X)	(0x1U << ((X) * 2))
#define TAMP_ACTIVE_ATOSEL_OUT3_(X)	(0x2U << ((X) * 2))
#define TAMP_ACTIVE_ATOSEL_OUT4_(X)	(0x3U << ((X) * 2))

#define TAMP_EXT(tamp_id, trig, erase, mask) (((tamp_id) << 3) | ((trig) << 2)\
					      | ((erase) << 1) | (mask))

#define TAMP_FLTCR(precharge, duration, count, sample) (((precharge) << 7) |\
							((duration) << 5) |\
							((count) << 3) |\
							(sample))

#define TAMP_ACT(filter, ato, aper, atcksel, atosel) (((filter) << 31) |\
						      ((ato) << 30) |\
						      ((aper) << 24) |\
						      ((atcksel) << 16) |\
						      (atosel) << 8)

struct stm32_tamp_int {
	int id;
	void (*func)(int id);
};

struct stm32_tamp_ext {
	int id;
	uint8_t mode;
	uint8_t erase;
	uint8_t evt_mask;
	void (*func)(int id);
};

/*
 * stm32_tamp_write_mcounter : Increase monotonic counter
 */
void stm32_tamp_write_mcounter(void);

/*
 * stm32_tamp_it_handler : Interrupt handler
 */
void stm32_tamp_it_handler(void);

/*
 * stm32_tamp_configure_internal: Configure internal tamper
 * tamper_list: List of tamper to enable
 * nb_tamper: Number of tamper in list
 */
void stm32_tamp_configure_internal(struct stm32_tamp_int *tamper_list,
				   uint16_t nb_tamper);

/*
 * stm32_tamp_configure_external: Configure external tamper and associated
 * configuration for filtering
 * ext_tamp_list: List of external tamper to configure
 * nb_tamper: Number of tamper in list
 * passive_conf: Filter configuration
 * active_conf: Configuration for active tamper
 */
void stm32_tamp_configure_external(struct stm32_tamp_ext *ext_tamper_list,
				   uint8_t nb_tamper, uint32_t passive_conf,
				   uint32_t active_conf);

/*
 * stm32_tamp_init: Initialize tamper from DT
 * return 0 if disabled, 1 if enabled, else < 0
 */
int stm32_tamp_init(void);

#endif /* STM32_TAMP_H */
