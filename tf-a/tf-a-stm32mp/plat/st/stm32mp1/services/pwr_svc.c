/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <common/debug.h>
#include <drivers/st/stm32mp1_pwr.h>
#include <lib/mmio.h>

#include <stm32mp_common.h>
#include <stm32mp1_smc.h>

#include "pwr_svc.h"

static void access_allowed_mask(uint32_t request, uint32_t offset,
				uint32_t value, uint32_t allowed_mask)
{
	uint32_t addr = stm32mp_pwr_base() + offset;
	uint32_t masked_value = value & allowed_mask;

	stm32mp_pwr_regs_lock();

	switch (request) {
	case STM32_SMC_REG_WRITE:
		mmio_clrsetbits_32(addr, allowed_mask, masked_value);
		VERBOSE("wrt 0x%x = 0x%x => 0x%x\n", offset, value,
			mmio_read_32(addr));
		break;

	case STM32_SMC_REG_SET:
		mmio_setbits_32(addr, masked_value);
		VERBOSE("set 0x%x = 0x%x => 0x%x\n", offset, value,
			mmio_read_32(addr));
		break;

	case STM32_SMC_REG_CLEAR:
		mmio_clrbits_32(addr, masked_value);
		VERBOSE("clear 0x%x = 0x%x => 0x%x\n", offset, value,
			mmio_read_32(addr));
		break;

	default:
		break;
	}

	stm32mp_pwr_regs_unlock();
}

static void raw_allowed_access_request(uint32_t request,
				       uint32_t offset, uint32_t value)
{
	uint32_t allowed_mask = 0;

	switch (offset) {
	case PWR_CR3:
		allowed_mask |= PWR_CR3_VBE | PWR_CR3_VBRS | PWR_CR3_USB33DEN |
				PWR_CR3_REG18EN | PWR_CR3_REG11EN;
		break;

	case PWR_WKUPCR:
		allowed_mask |= PWR_WKUPCR_MASK;
		break;

	case PWR_MPUWKUPENR:
		allowed_mask |= PWR_MPUWKUPENR_MASK;
		break;

	default:
		return;
	}

	if (allowed_mask != 0U) {
		access_allowed_mask(request, offset, value, allowed_mask);
	}
}

uint32_t pwr_scv_handler(uint32_t x1, uint32_t x2, uint32_t x3)
{
	uint32_t request = x1;
	uint32_t offset = x2;
	uint32_t value = x3;

	/*
	 * x2 may be either the PWR register offset or the register
	 * full physical address.
	 */
	if ((offset & ~PWR_OFFSET_MASK) != 0) {
		if ((offset & ~PWR_OFFSET_MASK) != stm32mp_pwr_base()) {
			return STM32_SMC_INVALID_PARAMS;
		}

		offset &= PWR_OFFSET_MASK;
	}

	/* PWR controls for non secure resource may be accessed straight */
	raw_allowed_access_request(request, offset, value);

	return STM32_SMC_OK;
}
