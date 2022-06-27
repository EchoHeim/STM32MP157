/*
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <drivers/auth/auth_mod.h>
#include <plat/common/platform.h>

static auth_param_type_desc_t stm32_header_pk =
	AUTH_PARAM_TYPE_DESC(AUTH_PARAM_PUB_KEY, 0);
static auth_param_type_desc_t stm32_header_sig =
	AUTH_PARAM_TYPE_DESC(AUTH_PARAM_SIG, 0);
static auth_param_type_desc_t stm32_header_sig_alg =
	AUTH_PARAM_TYPE_DESC(AUTH_PARAM_SIG_ALG, 0);
static auth_param_type_desc_t stm32_load =
	AUTH_PARAM_TYPE_DESC(AUTH_PARAM_RAW_DATA, 0);

#if defined(AARCH32_SP_OPTEE)
static const auth_img_desc_t bl32_image = {
	.img_id = BL32_IMAGE_ID,
	.img_type = IMG_PLAT,
	.parent = NULL,
	.img_auth_methods = (const auth_method_desc_t[AUTH_METHOD_NUM]) {
		[0] = {
			.type = AUTH_METHOD_SIG,
			.param.sig = {
				.pk = &stm32_header_pk,
				.sig = &stm32_header_sig,
				.alg = &stm32_header_sig_alg,
				.data = &stm32_load
			}
		},
	},
};

static const auth_img_desc_t bl32_extra1_image = {
	.img_id = BL32_EXTRA1_IMAGE_ID,
	.img_type = IMG_PLAT,
	.parent = NULL,
	.img_auth_methods = (const auth_method_desc_t[AUTH_METHOD_NUM]) {
		[0] = {
			.type = AUTH_METHOD_SIG,
			.param.sig = {
				.pk = &stm32_header_pk,
				.sig = &stm32_header_sig,
				.alg = &stm32_header_sig_alg,
				.data = &stm32_load
			}
		},
	},
};

static const auth_img_desc_t bl32_extra2_image = {
	.img_id = BL32_EXTRA2_IMAGE_ID,
	.img_type = IMG_PLAT,
	.parent = NULL,
	.img_auth_methods = (const auth_method_desc_t[AUTH_METHOD_NUM]) {
		[0] = {
			.type = AUTH_METHOD_SIG,
			.param.sig = {
				.pk = &stm32_header_pk,
				.sig = &stm32_header_sig,
				.alg = &stm32_header_sig_alg,
				.data = &stm32_load
			}
		},
	},
};
#else
static const auth_img_desc_t bl32_image = {
	.img_id = BL32_IMAGE_ID,
	.img_type = IMG_RAW,
	.parent = NULL,
	.img_auth_methods = (const auth_method_desc_t[AUTH_METHOD_NUM]) {
		[0] = {
			.type = AUTH_METHOD_NONE, /* Already verified by BL1
						   * as loaded in the same time
						   * as BL2
						   */
		}
	},
};
#endif

static const auth_img_desc_t bl33_image = {
	.img_id = BL33_IMAGE_ID,
	.img_type = IMG_PLAT,
	.parent = NULL,
	.img_auth_methods = (const auth_method_desc_t[AUTH_METHOD_NUM]) {
		[0] = {
			.type = AUTH_METHOD_SIG,
			.param.sig = {
				.pk = &stm32_header_pk,
				.sig = &stm32_header_sig,
				.alg = &stm32_header_sig_alg,
				.data = &stm32_load
			}
		},
	},
};

static const auth_img_desc_t * const cot_desc[] = {
	[BL32_IMAGE_ID] = &bl32_image,
#if defined(AARCH32_SP_OPTEE)
	[BL32_EXTRA1_IMAGE_ID] = &bl32_extra1_image,
	[BL32_EXTRA2_IMAGE_ID] = &bl32_extra2_image,
#endif
	[BL33_IMAGE_ID] = &bl33_image,
};

REGISTER_COT(cot_desc);
