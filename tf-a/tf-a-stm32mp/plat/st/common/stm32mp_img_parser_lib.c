/*
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>

#include <platform_def.h>

#include <common/debug.h>
#include <drivers/auth/img_parser_mod.h>
#include <plat/common/platform.h>

static void img_lib_init(void)
{
}

static int img_check_integrity(void *img, unsigned int img_len)
{
	return stm32mp_check_header(stm32mp_get_loaded_header(),
				    (uintptr_t)img);
}

static int img_get_auth_param(const auth_param_type_desc_t *type_desc,
			      void *img, unsigned int img_len, void **param,
			      unsigned int *param_len)
{
	boot_api_image_header_t *image_header = stm32mp_get_loaded_header();

	switch (type_desc->type) {
	case AUTH_PARAM_SIG:
		*param_len = sizeof(image_header->image_signature);
		*param = &image_header->image_signature;
		break;
	case AUTH_PARAM_SIG_ALG:
		*param_len = sizeof(image_header->option_flags) +
			sizeof(image_header->ecc_algo_type);
		*param = &image_header->option_flags;
		/*
		 * Store option_flags and ecc_alog_type in same param
		 * structure because they both have the same header fields.
		 */
		break;
	case AUTH_PARAM_PUB_KEY:
		*param_len = sizeof(image_header->ecc_pubk);
		*param = &image_header->ecc_pubk;
		break;
	case AUTH_PARAM_RAW_DATA:
		if (type_desc->cookie == NULL) {
			*param_len = image_header->image_length;
			*param = img;
		} else {
			return -EINVAL;
		}
		break;
	case AUTH_PARAM_NV_CTR:
		if (type_desc->cookie == NULL) {
			*param_len = sizeof(image_header->image_version);
			*param = &image_header->image_version;
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

REGISTER_IMG_PARSER_LIB(IMG_PLAT, "stm32_img_parser_lib",
			img_lib_init,
			img_check_integrity,
			img_get_auth_param);
