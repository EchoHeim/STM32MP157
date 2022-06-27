/*
 * Copyright (c) 2016-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <common/bl_common.h>
#include <common/desc_image_load.h>
#include <lib/mmio.h>
#include <plat/common/platform.h>

/*******************************************************************************
 * This function flushes the data structures so that they are visible
 * in memory for the next BL image.
 ******************************************************************************/
void plat_flush_next_bl_params(void)
{
	flush_bl_params_desc();
}

#ifdef AARCH32_SP_OPTEE
static bool addr_inside_backupsram(uintptr_t addr)
{
	return (addr >= STM32MP_BACKUP_RAM_BASE) &&
		(addr < (STM32MP_BACKUP_RAM_BASE + STM32MP_BACKUP_RAM_SIZE));
}
#endif

/*******************************************************************************
 * This function returns the list of loadable images.
 ******************************************************************************/
bl_load_info_t *plat_get_bl_image_load_info(void)
{
	boot_api_context_t *boot_context =
		(boot_api_context_t *)stm32mp_get_boot_ctx_address();
#ifdef AARCH32_SP_OPTEE
	bl_mem_params_node_t *bl32 = get_bl_mem_params_node(BL32_IMAGE_ID);
#endif
	bl_mem_params_node_t *bl33 = get_bl_mem_params_node(BL33_IMAGE_ID);
	uint32_t rstsr = mmio_read_32(stm32mp_rcc_base() + RCC_MP_RSTSCLRR);
	uint32_t bkpr_core1_addr =
		tamp_bkpr(BOOT_API_CORE1_BRANCH_ADDRESS_TAMP_BCK_REG_IDX);
	uintptr_t pwr_base = stm32mp_pwr_base();

	/*
	 * If going back from CSTANDBY / STANDBY and DDR was in Self-Refresh,
	 * BL33 must not be loaded as it would overwrite the code already
	 * in DDR. For this, the BL33 part of the bl_mem_params_desc_ptr
	 * struct should be modified to skip its loading
	 */
	if (((boot_context->boot_action ==
	      BOOT_API_CTX_BOOT_ACTION_WAKEUP_CSTANDBY) ||
	     (boot_context->boot_action ==
	      BOOT_API_CTX_BOOT_ACTION_WAKEUP_STANDBY)) &&
	    ((mmio_read_32(pwr_base + PWR_CR3) & PWR_CR3_DDRSREN) != 0U) &&
	    ((rstsr & RCC_MP_RSTSCLRR_PADRSTF) == 0U)) {
		stm32mp_clk_enable(RTCAPB);

		if (mmio_read_32(bkpr_core1_addr) != 0U) {
			bl33->image_info.h.attr |= IMAGE_ATTRIB_SKIP_LOADING;

#ifdef AARCH32_SP_OPTEE
			bl32->image_info.h.attr |= IMAGE_ATTRIB_SKIP_LOADING;
			bl32->ep_info.pc = stm32_pm_get_optee_ep();

			if (addr_inside_backupsram(bl32->ep_info.pc)) {
				stm32mp_clk_enable(BKPSRAM);
			}
#else
			/*
			 * Set ep_info PC to 0, to inform BL32 it is a reset
			 * after STANDBY
			 */
			bl33->ep_info.pc = 0;
#endif
		}

		stm32mp_clk_disable(RTCAPB);
	}

	/* Max size is non-secure DDR end address minus image_base */
	bl33->image_info.image_max_size = STM32MP_DDR_BASE +
					  dt_get_ddr_size() -
					  STM32MP_DDR_S_SIZE -
					  STM32MP_DDR_SHMEM_SIZE -
					  bl33->image_info.image_base;

	return get_bl_load_info_from_mem_params_desc();
}

/*******************************************************************************
 * This function returns the list of executable images.
 ******************************************************************************/
bl_params_t *plat_get_next_bl_params(void)
{
	return get_next_bl_params_from_mem_params_desc();
}
