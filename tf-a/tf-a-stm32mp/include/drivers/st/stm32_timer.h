/*
 * Copyright (c) 2018-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32_TIMER_H
#define STM32_TIMER_H

enum timer_cal {
	HSI_CAL = 0,
	CSI_CAL
};

unsigned long stm32_timer_hsi_freq(void);
unsigned long stm32_timer_csi_freq(void);
void stm32_timer_freq_func(unsigned long (**timer_freq_cb)(void),
			   enum timer_cal type);
int stm32_timer_init(void);

#endif /* STM32_TIMER_H */
