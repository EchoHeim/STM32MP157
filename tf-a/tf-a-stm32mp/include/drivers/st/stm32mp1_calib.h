/*
 * Copyright (c) 2018-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_CALIB_H
#define STM32MP1_CALIB_H

#include <stdbool.h>
#include <stdint.h>

bool stm32mp1_calib_get_wakeup(void);
void stm32mp1_calib_set_wakeup(bool state);
void stm32mp1_calib_it_handler(uint32_t id);
int stm32mp1_calib_start_hsi_cal(void);
int stm32mp1_calib_start_csi_cal(void);
void stm32mp1_calib_init(void);

#endif /* STM32MP1_CLK_H */
