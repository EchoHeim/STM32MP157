/*
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <platform_def.h>

#include <common/debug.h>
#include <drivers/auth/crypto_mod.h>
#include <drivers/io/io_storage.h>
#include <drivers/st/bsec.h>
#include <drivers/st/stm32_hash.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

struct stm32mp_auth_ops {
	uint32_t (*check_key)(uint8_t *pubkey_in, uint8_t *pubkey_out);
	uint32_t (*verify_signature)(uint8_t *hash_in, uint8_t *pubkey_in,
				     uint8_t *signature, uint32_t ecc_algo);
};

static struct stm32mp_auth_ops auth_ops;

static void crypto_lib_init(void)
{
	boot_api_context_t *boot_context =
		(boot_api_context_t *)stm32mp_get_boot_ctx_address();

	if (!stm32mp_is_auth_supported()) {
		return;
	}

	auth_ops.check_key = boot_context->bootrom_ecdsa_check_key;
	auth_ops.verify_signature =
		boot_context->bootrom_ecdsa_verify_signature;

	if (stm32_hash_register() != 0) {
		panic();
	}
}

static int crypto_verify_signature(void *data_ptr, unsigned int data_len,
				   void *sig_ptr, unsigned int sig_len,
				   void *sig_alg, unsigned int sig_alg_len,
				   void *pk_ptr, unsigned int pk_len)
{
	uint8_t image_hash[BOOT_API_SHA256_DIGEST_SIZE_IN_BYTES];
	uint32_t option_flags;
	uint32_t ecc_algo_type;
	uint32_t header_len;
	int result;
	boot_api_image_header_t *header = stm32mp_get_loaded_header();

	header_len = sizeof(boot_api_image_header_t) - sizeof(header->magic) -
		     sizeof(header->image_signature) -
		     sizeof(header->payload_checksum);

	if ((((size_t)sig_alg % __alignof__(uint32_t)) != 0) ||
	    (sig_alg_len != sizeof(option_flags) + sizeof(ecc_algo_type))) {
		return -EINVAL;
	}

	option_flags = ((uint32_t *)sig_alg)[0];
	ecc_algo_type = ((uint32_t *)sig_alg)[1];

	/* Check security status */
	if (!stm32mp_is_closed_device()) {
		if (option_flags != 0U) {
			WARN("Skip signature check (header option)\n");
			stm32mp_delete_loaded_header();
			return 0;
		}
		INFO("Check signature on Open device\n");
	}

	/* Check key/sign size */
	if ((pk_len != BOOT_API_ECDSA_PUB_KEY_LEN_IN_BYTES) ||
	    (sig_len != BOOT_API_ECDSA_SIGNATURE_LEN_IN_BYTES)) {
		return -EINVAL;
	}

	result = mmap_add_dynamic_region(STM32MP_ROM_BASE, STM32MP_ROM_BASE,
					 STM32MP_ROM_SIZE, MT_CODE | MT_SECURE);
	if (result != 0) {
		return result;
	}

	if (!stm32mp_is_closed_device()) {
		/*
		 * Check public key here in case of non-secure device
		 * It is done in the generic framework in case of close
		 * device.
		 */
		if (auth_ops.check_key(pk_ptr, NULL) != BOOT_API_RETURN_OK) {
			ERROR("ROTPK verification failed\n");
			result = -EINVAL;
			goto out;
		} else {
			NOTICE("ROTPK verification forced and checked OK\n");
		}
	}

	/* Compute hash for the data covered by the signature */
	stm32_hash_init(HASH_SHA256);

	result = stm32_hash_update((void *)&header->header_version, header_len);
	if (result != 0) {
		VERBOSE("Hash of header failed, %i\n", result);
		goto out;
	}

	result = stm32_hash_final_update((uint8_t *)data_ptr,
					 data_len, image_hash);
	if (result != 0) {
		VERBOSE("Hash of payload failed, %i\n", result);
		goto out;
	}

	/* Verify signature */
	if (auth_ops.verify_signature(image_hash, pk_ptr, sig_ptr,
				      ecc_algo_type) != BOOT_API_RETURN_OK) {
		result = -EAUTH;
	}

out:
	mmap_remove_dynamic_region(STM32MP_ROM_BASE, STM32MP_ROM_SIZE);
	if (result != 0) {
		stm32mp_delete_loaded_header();
	}

	return result;
}

static int crypto_verify_hash(void *data_ptr, unsigned int data_len,
			      void *digest_info_ptr,
			      unsigned int digest_info_len)
{
	int ret;
	uint8_t calc_hash[BOOT_API_SHA256_DIGEST_SIZE_IN_BYTES];

	if (digest_info_len != BOOT_API_SHA256_DIGEST_SIZE_IN_BYTES) {
		VERBOSE("%s: unexpected digest_len\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	stm32_hash_init(HASH_SHA256);
	ret = stm32_hash_final_update(data_ptr, data_len, calc_hash);
	if (ret != 0) {
		VERBOSE("%s: hash failed\n", __func__);
		goto out;
	}

	ret = memcmp(calc_hash, digest_info_ptr, digest_info_len);
	if (ret != 0) {
		VERBOSE("%s: not expected digest\n", __func__);
		ret = -EAUTH;
	}

out:
	/* Clean header as no more used */
	stm32mp_delete_loaded_header();

	return ret;
}

REGISTER_CRYPTO_LIB("stm32_crypto_lib",
		    crypto_lib_init,
		    crypto_verify_signature,
		    crypto_verify_hash);
