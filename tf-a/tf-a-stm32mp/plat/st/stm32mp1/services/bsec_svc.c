/*
 * Copyright (c) 2016-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <platform_def.h>

#include <arch.h>
#include <arch_helpers.h>
#include <common/debug.h>
#include <common/runtime_svc.h>
#include <drivers/st/bsec.h>
#include <drivers/st/bsec2_reg.h>
#include <drivers/st/stm32mp_pmic.h>
#include <drivers/st/stm32mp1_ddr_helpers.h>
#include <drivers/st/stpmic1.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <services/std_svc.h>

#include <boot_api.h>
#include <stm32mp1_dbgmcu.h>
#include <stm32mp1_smc.h>

#include "bsec_svc.h"

#define SSP_OTP_REQ		BIT(BOOT_API_OTP_SSP_REQ_BIT_POS)
#define SSP_OTP_SUCCESS		BIT(BOOT_API_OTP_SSP_SUCCESS_BIT_POS)
#define SSP_OTP_MASK		(SSP_OTP_REQ | SSP_OTP_SUCCESS)

enum bsec_ssp_status {
	BSEC_NO_SSP = 0,
	BSEC_SSP_SET,
	BSEC_SSP_ERROR
};

struct otp_exchange {
	uint32_t version;
	uint32_t configuration;
	uint32_t reserved;
	uint32_t status;
	uint32_t general_lock;
	uint32_t debug_conf;
	uint32_t reserved1[2];
	uint32_t otp_disturb[3];
	uint32_t reserved2[3];
	uint32_t error_status[3];
	uint32_t reserved3[3];
	uint32_t permanent_lock[3];
	uint32_t reserved4[3];
	uint32_t programming_lock[3];
	uint32_t reserved5[3];
	uint32_t shadow_write_lock[3];
	uint32_t reserved6[3];
	uint32_t shadow_read_lock[3];
	uint32_t reserved7[3];
	uint32_t otp_value[STM32MP1_OTP_MAX_ID + 1];
	uint32_t reserved8[112];
	uint32_t bsec_hw_conf;
	uint32_t ip_version;
	uint32_t ip_id;
	uint32_t ip_magic_id;
};

static enum bsec_ssp_status bsec_check_ssp(uint32_t otp, uint32_t update)
{
	boot_api_context_t *boot_context =
		(boot_api_context_t *)BOOT_PARAM_ADDR;

	/* No SSP update or SSP already done*/
	if ((((otp & SSP_OTP_MASK) == 0U) && ((update & SSP_OTP_MASK) == 0U)) ||
	    (((otp & SSP_OTP_MASK) == SSP_OTP_MASK) &&
	     ((update & SSP_OTP_MASK) == SSP_OTP_MASK))) {
		return BSEC_NO_SSP;
	}

	/* SSP update */
	if ((update & SSP_OTP_MASK) != 0U) {
		if ((update & SSP_OTP_SUCCESS) != 0U) {
			return BSEC_SSP_ERROR;
		}

		/* SSP boot process */
		boot_context->p_ssp_config->ssp_cmd =
			BOOT_API_CTX_SSP_CMD_CALC_CHIP_PUBK;
#ifndef DCACHE_OFF
		flush_dcache_range((uintptr_t)boot_context->p_ssp_config,
				   sizeof(boot_api_ssp_config_t));
#endif
		if (dt_pmic_status() > 0) {
			const char *name;

			initialize_pmic();

			name = stm32mp_get_cpu_supply_name();
			if (name == NULL) {
				return BSEC_SSP_ERROR;
			}

			stpmic1_regulator_mask_reset_set(name);
		}

		return BSEC_SSP_SET;
	}
	return BSEC_NO_SSP;
}

#if STM32MP_USB_PROGRAMMER || STM32MP_UART_PROGRAMMER
static uint32_t bsec_read_all_bsec(struct otp_exchange *exchange)
{
	uint32_t i;
	uint32_t result;

	if (exchange == NULL) {
		return BSEC_ERROR;
	}

	exchange->version = BSEC_SERVICE_VERSION;

	for (i = 0U; i <= STM32MP1_OTP_MAX_ID; i++) {
		if (bsec_check_nsec_access_rights(i) == BSEC_OK) {
			result = bsec_shadow_register(i);
			if (result != BSEC_OK) {
				return result;
			}

			result = bsec_read_otp(&exchange->otp_value[i], i);
			if (result != BSEC_OK) {
				return result;
			}
		}
	}

	exchange->configuration = mmio_read_32(bsec_get_base() +
					       BSEC_OTP_CONF_OFF);

	exchange->status = mmio_read_32(bsec_get_base() + BSEC_OTP_STATUS_OFF);

	exchange->general_lock = mmio_read_32(bsec_get_base() +
					      BSEC_OTP_LOCK_OFF);

	exchange->debug_conf = mmio_read_32(bsec_get_base() + BSEC_DEN_OFF);

	exchange->otp_disturb[0] = mmio_read_32(bsec_get_base() +
						BSEC_DISTURBED_OFF);

	exchange->otp_disturb[1] = mmio_read_32(bsec_get_base() +
						BSEC_DISTURBED1_OFF);

	exchange->otp_disturb[2] = mmio_read_32(bsec_get_base() +
						BSEC_DISTURBED2_OFF);

	exchange->error_status[0] = mmio_read_32(bsec_get_base() +
						 BSEC_ERROR_OFF);

	exchange->error_status[1] = mmio_read_32(bsec_get_base() +
						 BSEC_ERROR1_OFF);

	exchange->error_status[2] = mmio_read_32(bsec_get_base() +
						 BSEC_ERROR2_OFF);

	exchange->permanent_lock[0] = mmio_read_32(bsec_get_base() +
						   BSEC_WRLOCK_OFF);

	exchange->permanent_lock[1] = mmio_read_32(bsec_get_base() +
						   BSEC_WRLOCK1_OFF);

	exchange->permanent_lock[2] = mmio_read_32(bsec_get_base() +
						   BSEC_WRLOCK2_OFF);

	exchange->programming_lock[0] = mmio_read_32(bsec_get_base() +
						     BSEC_SPLOCK_OFF);

	exchange->programming_lock[1] = mmio_read_32(bsec_get_base() +
						     BSEC_SPLOCK1_OFF);

	exchange->programming_lock[2] = mmio_read_32(bsec_get_base() +
						     BSEC_SPLOCK2_OFF);

	exchange->shadow_write_lock[0] = mmio_read_32(bsec_get_base() +
						      BSEC_SWLOCK_OFF);

	exchange->shadow_write_lock[1] = mmio_read_32(bsec_get_base() +
						      BSEC_SWLOCK1_OFF);

	exchange->shadow_write_lock[2] = mmio_read_32(bsec_get_base() +
						      BSEC_SWLOCK2_OFF);

	exchange->shadow_read_lock[0] = mmio_read_32(bsec_get_base() +
						     BSEC_SRLOCK_OFF);

	exchange->shadow_read_lock[1] = mmio_read_32(bsec_get_base() +
						     BSEC_SRLOCK1_OFF);

	exchange->shadow_read_lock[2] = mmio_read_32(bsec_get_base() +
						     BSEC_SRLOCK2_OFF);

	exchange->bsec_hw_conf = mmio_read_32(bsec_get_base() +
					      BSEC_IPHW_CFG_OFF);

	exchange->ip_version = mmio_read_32(bsec_get_base() + BSEC_IPVR_OFF);

	exchange->ip_id = mmio_read_32(bsec_get_base() + BSEC_IP_ID_OFF);

	exchange->ip_magic_id = mmio_read_32(bsec_get_base() +
					     BSEC_IP_MAGIC_ID_OFF);

	return BSEC_OK;
}

static uint32_t bsec_write_all_bsec(struct otp_exchange *exchange,
				    uint32_t *ret_otp_value)
{
	uint32_t i;
	uint32_t j;
	uint32_t start_otp = 0U;
	uint32_t value = 0U;
	uint32_t ret;
	struct bsec_config config_param;

	*ret_otp_value = 0U;

	if (exchange == NULL) {
		return BSEC_ERROR;
	}

	if (exchange->version != BSEC_SERVICE_VERSION) {
		return BSEC_ERROR;
	}

	for (i = start_otp; i <= STM32MP1_OTP_MAX_ID; i++) {
		if (bsec_check_nsec_access_rights(i) != BSEC_OK) {
			continue;
		}

		ret = bsec_shadow_register(i);
		if (ret != BSEC_OK) {
			return ret;
		}

		ret = bsec_read_otp(&value, i);
		if (ret != BSEC_OK) {
			return ret;
		}

		if ((value ==  exchange->otp_value[i]) &&
		    (i != BOOT_API_OTP_SSP_WORD_NB)) {
			continue;
		}

		if (i == BOOT_API_OTP_SSP_WORD_NB) {
			*ret_otp_value = (uint32_t)bsec_check_ssp(value,
							exchange->otp_value[i]);
			VERBOSE("Result OTP SSP %d\n", *ret_otp_value);
			if (*ret_otp_value == (uint32_t)BSEC_SSP_ERROR) {
				continue;
			}
		}

		ret = bsec_program_otp(exchange->otp_value[i], i);
		if (ret != BSEC_OK) {
			return ret;
		}

		ret = bsec_write_otp(exchange->otp_value[i], i);
		if (ret != BSEC_OK) {
			return ret;
		}
	}

	ret = bsec_write_debug_conf(exchange->debug_conf);
	if (ret != BSEC_OK) {
		return ret;
	}

	for (j = 0U; j < 3U; j++) {
		if (exchange->permanent_lock[j] == 0U) {
			continue;
		}

		for (i = 0U; i < 32U; i++) {
			if (bsec_check_nsec_access_rights((32U * j) + i) !=
			    BSEC_OK) {
				continue;
			}

			value = (exchange->permanent_lock[j] >> i) & 1U;
			if (value != 0U) {
				ret = bsec_permanent_lock_otp((32U * j) + i);
				if (ret != BSEC_OK) {
					return ret;
				}
			}
		}
	}

	for (j = 0U; j < 3U; j++) {
		if (exchange->programming_lock[j] == 0U) {
			continue;
		}

		for (i = 0U; i < 32U; i++) {
			if (bsec_check_nsec_access_rights((32U * j) + i) !=
			    BSEC_OK) {
				continue;
			}

			value = (exchange->programming_lock[j] >> i) & 1U;
			if (value != 0U) {
				if (bsec_set_sp_lock((32U * j) + i) !=
				    BSEC_OK) {
					return BSEC_ERROR;
				}
			}
		}
	}

	for (j = 0U; j < 3U; j++) {
		if (exchange->shadow_write_lock[j] == 0U) {
			continue;
		}

		for (i = 0U; i < 32U; i++) {
			if (bsec_check_nsec_access_rights((32U * j) + i) !=
			    BSEC_OK) {
				continue;
			}

			value = (exchange->shadow_write_lock[j] >> i) & 1U;
			if (value != 0U) {
				if (bsec_set_sw_lock((32U * j) + i) !=
				    BSEC_OK) {
					return BSEC_ERROR;
				}
			}
		}
	}

	for (j = 0U; j < 3U; j++) {
		if (exchange->shadow_read_lock[j] == 0U) {
			continue;
		}

		for (i = 0U; i < 32U; i++) {
			if (bsec_check_nsec_access_rights((32U * j) + i) !=
			    BSEC_OK) {
				continue;
			}

			value = (exchange->shadow_read_lock[j] >> i) & 1U;
			if (value != 0U) {
				if (bsec_set_sr_lock((32U * j) + i) !=
				    BSEC_OK) {
					return BSEC_ERROR;
				}
			}
		}
	}

	ret = bsec_get_config(&config_param);
	if (ret != BSEC_OK) {
		return ret;
	}

	config_param.power =
		(uint8_t)(exchange->configuration & BSEC_CONF_POWER_UP_MASK) >>
		BSEC_CONF_POWER_UP_SHIFT;
	config_param.freq =
		(uint8_t)(exchange->configuration & BSEC_CONF_FRQ_MASK) >>
		BSEC_CONF_FRQ_SHIFT;
	config_param.pulse_width =
		(uint8_t)(exchange->configuration & BSEC_CONF_PRG_WIDTH_MASK) >>
		BSEC_CONF_PRG_WIDTH_SHIFT;
	config_param.tread =
		(uint8_t)((exchange->configuration & BSEC_CONF_TREAD_MASK) >>
		BSEC_CONF_TREAD_SHIFT);
	config_param.den_lock =
		(uint8_t)(exchange->general_lock & DENREG_LOCK_MASK) >>
		DENREG_LOCK_SHIFT;
	config_param.prog_lock =
		(uint8_t)(exchange->general_lock & GPLOCK_LOCK_MASK) >>
		GPLOCK_LOCK_SHIFT;

	config_param.upper_otp_lock =
		(uint8_t)(exchange->general_lock & UPPER_OTP_LOCK_MASK) >>
		 UPPER_OTP_LOCK_SHIFT;

	ret = bsec_set_config(&config_param);
	if (ret != BSEC_OK) {
		return ret;
	}

	INFO("write all otp succeed\n");

	return BSEC_OK;
}
#endif /* STM32MP_USB_PROGRAMMER || STM32MP_UART_PROGRAMMER */

uint32_t bsec_main(uint32_t x1, uint32_t x2, uint32_t x3,
		   uint32_t *ret_otp_value)
{
	uint32_t result;
	uint32_t tmp_data = 0U;
	struct otp_exchange *otp_exch __unused;
	uintptr_t map_begin __unused;
	size_t map_size __unused = PAGE_SIZE;
	int ret __unused;

	if ((x1 != STM32_SMC_READ_ALL) && (x1 != STM32_SMC_WRITE_ALL) &&
	    (bsec_check_nsec_access_rights(x2) != BSEC_OK)) {
		return STM32_SMC_INVALID_PARAMS;
	}

#if STM32MP_USB_PROGRAMMER || STM32MP_UART_PROGRAMMER
	otp_exch = NULL;
	map_begin = 0U;

	if ((x1 == STM32_SMC_READ_ALL) || (x1 == STM32_SMC_WRITE_ALL)) {
		map_begin = round_down(x2, PAGE_SIZE);

		if (round_down(x2 + sizeof(struct otp_exchange), PAGE_SIZE) !=
		    map_begin) {
			/*
			 * Buffer end is in the next page, 2 pages need to be
			 * mapped.
			 */
			map_size += PAGE_SIZE;
		}

		ret = mmap_add_dynamic_region(map_begin,
					      map_begin,
					      map_size,
					      MT_MEMORY | MT_RW | MT_NS);
		assert(ret == 0);

		if (!ddr_is_nonsecured_area(map_begin, map_size)) {
			ret = mmap_remove_dynamic_region(map_begin, map_size);
			assert(ret == 0);

			return STM32_SMC_INVALID_PARAMS;
		}

		otp_exch = (struct otp_exchange *)(uintptr_t)x2;
	}
#endif

	switch (x1) {
	case STM32_SMC_READ_SHADOW:
		result = bsec_read_otp(ret_otp_value, x2);
		break;
	case STM32_SMC_PROG_OTP:
		*ret_otp_value = 0U;
		if (x2 == BOOT_API_OTP_SSP_WORD_NB) {
			result = bsec_read_otp(&tmp_data, x2);
			if (result != BSEC_OK) {
				break;
			}

			*ret_otp_value = (uint32_t)bsec_check_ssp(tmp_data, x3);
			if (*ret_otp_value == (uint32_t)BSEC_SSP_ERROR) {
				result = BSEC_OK;
				break;
			}
		}
		result = bsec_program_otp(x3, x2);
		break;
	case STM32_SMC_WRITE_SHADOW:
		*ret_otp_value = 0;
		result = bsec_write_otp(x3, x2);
		break;
	case STM32_SMC_READ_OTP:
		*ret_otp_value = 0;
		result = bsec_read_otp(&tmp_data, x2);
		if (result != BSEC_OK) {
			break;
		}

		result = bsec_shadow_register(x2);
		if (result != BSEC_OK) {
			break;
		}

		result = bsec_read_otp(ret_otp_value, x2);
		if (result != BSEC_OK) {
			break;
		}

		result = bsec_write_otp(tmp_data, x2);
		break;
#if STM32MP_USB_PROGRAMMER || STM32MP_UART_PROGRAMMER
	case STM32_SMC_READ_ALL:
		result = bsec_read_all_bsec(otp_exch);
		break;
	case STM32_SMC_WRITE_ALL:
		result = bsec_write_all_bsec(otp_exch, ret_otp_value);
		break;
#endif
	case STM32_SMC_WRLOCK_OTP:
		result = bsec_permanent_lock_otp(x2);
		break;
	default:
		return STM32_SMC_INVALID_PARAMS;
	}

#if STM32MP_USB_PROGRAMMER || STM32MP_UART_PROGRAMMER
	if ((x1 == STM32_SMC_READ_ALL) || (x1 == STM32_SMC_WRITE_ALL)) {
		ret = mmap_remove_dynamic_region(map_begin, map_size);
		assert(ret == 0);
	}
#endif

	return (result == BSEC_OK) ? STM32_SMC_OK : STM32_SMC_FAILED;
}
