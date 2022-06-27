/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef RCC_SVC_H
#define RCC_SVC_H

uint32_t rcc_scv_handler(uint32_t x1, uint32_t x2, uint32_t x3);
uint32_t rcc_cal_scv_handler(uint32_t x1);
uint32_t rcc_opp_scv_handler(uint32_t x1, uint32_t x2, uint32_t *res);

#endif /* RCC_SVC_H */
