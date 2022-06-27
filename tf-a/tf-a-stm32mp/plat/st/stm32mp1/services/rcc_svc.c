/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <limits.h>

#include <platform_def.h>

#include <common/debug.h>
#include <drivers/st/stm32_gpio.h>
#include <drivers/st/stm32mp1_clk.h>
#include <drivers/st/stm32mp1_rcc.h>
#include <lib/mmio.h>

#include <stm32mp_common.h>
#include <stm32mp_shres_helpers.h>
#include <stm32mp1_shared_resources.h>
#include <stm32mp1_smc.h>

#include "rcc_svc.h"

static bool offset_is_clear_register(uint32_t __unused offset)
{
	/* All currently allowed registers are non set/clear registers */
	return false;
}

static void access_allowed_mask(uint32_t request, uint32_t offset,
				uint32_t value, uint32_t allowed_mask)
{
	uint32_t addr = stm32mp_rcc_base() + offset;
	uint32_t masked_value = value & allowed_mask;

	switch (request) {
	case STM32_SMC_REG_WRITE:
		if (offset_is_clear_register(offset)) {
			mmio_write_32(addr, masked_value);
		} else {
			stm32mp_mmio_clrsetbits_32_shregs(addr, allowed_mask,
							  masked_value);
		}
		VERBOSE("wrt 0x%x = 0x%x => 0x%x\n", offset, value,
			mmio_read_32(addr));
		break;

	case STM32_SMC_REG_SET:
		if (offset_is_clear_register(offset)) {
			mmio_write_32(addr, masked_value);
		} else {
			stm32mp_mmio_setbits_32_shregs(addr, masked_value);
		}
		VERBOSE("set 0x%x = 0x%x => 0x%x\n", offset, value,
			mmio_read_32(addr));
		break;

	case STM32_SMC_REG_CLEAR:
		if (offset_is_clear_register(offset)) {
			/* Nothing to do on CLR registers */
		} else {
			stm32mp_mmio_clrbits_32_shregs(addr, masked_value);
		}
		VERBOSE("clear 0x%x = 0x%x => 0x%x\n", offset, value,
			mmio_read_32(addr));
		break;

	default:
		break;
	}
}

static void raw_allowed_access_request(uint32_t request,
				       uint32_t offset, uint32_t value)
{
	uint32_t allowed_mask = 0;

	switch (offset) {
	case RCC_MP_CIER:
	case RCC_MP_CIFR:
		allowed_mask = RCC_MP_CIFR_WKUPF;
		break;
	case RCC_MP_GCR:
		allowed_mask = RCC_MP_GCR_BOOT_MCU;
		break;
	default:
		panic();
	}

	if (allowed_mask != 0U) {
		access_allowed_mask(request, offset, value, allowed_mask);
	}
}

uint32_t rcc_scv_handler(uint32_t x1, uint32_t x2, uint32_t x3)
{
	uint32_t request = x1;
	uint32_t offset = x2;
	uint32_t value = x3;

	/*
	 * x2 may be either the RCC register offset or the register
	 * full physical address.
	 */
	if ((offset & ~RCC_OFFSET_MASK) != 0) {
		if ((offset & ~RCC_OFFSET_MASK) != stm32mp_rcc_base()) {
			return STM32_SMC_INVALID_PARAMS;
		}

		offset &= RCC_OFFSET_MASK;
	}

	raw_allowed_access_request(request, offset, value);

	return STM32_SMC_OK;
}

uint32_t rcc_cal_scv_handler(uint32_t x1)
{
	uint32_t ret = STM32_SMC_FAILED;

	switch (x1) {
	case CK_CSI:
		if (stm32mp1_calib_start_csi_cal() ==  0) {
			ret = STM32_SMC_OK;
		}
		break;

	case CK_HSI:
		if (stm32mp1_calib_start_hsi_cal() == 0) {
			ret = STM32_SMC_OK;
		}
		break;

	default:
		ret = STM32_SMC_INVALID_PARAMS;
		break;
	}

	return ret;
}

uint32_t rcc_opp_scv_handler(uint32_t x1, uint32_t x2, uint32_t *res)
{
	uint32_t cmd = x1;
	uint32_t opp = x2 / 1000U; /* KHz */

	switch (cmd) {
	case STM32_SMC_RCC_OPP_SET:
		if (stm32mp1_set_opp_khz(opp) != 0) {
			return STM32_SMC_FAILED;
		}
		break;

	case STM32_SMC_RCC_OPP_ROUND:
		if (stm32mp1_round_opp_khz(&opp) != 0) {
			return STM32_SMC_FAILED;
		}

		if (opp > (UINT32_MAX / 1000U)) {
			return STM32_SMC_FAILED;
		}

		*res = opp * 1000U;
		break;

	default:
		return STM32_SMC_INVALID_PARAMS;
	}

	return STM32_SMC_OK;
}
