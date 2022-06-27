/*
 * Copyright (c) 2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32_RNG_H
#define STM32_RNG_H

int stm32_rng_read(uint8_t *out, uint32_t size);
int stm32_rng_init(void);

#endif /* STM32_RNG_H */
