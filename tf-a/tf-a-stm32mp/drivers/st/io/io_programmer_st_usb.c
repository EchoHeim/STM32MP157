/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <string.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/io/io_driver.h>
#include <drivers/io/io_storage.h>
#include <drivers/st/io_programmer.h>
#include <drivers/st/io_programmer_st_usb.h>
#include <drivers/st/io_stm32image.h>
#include <drivers/st/stm32_iwdg.h>
#include <lib/usb/usb_st_dfu.h>

#define USB_STATE_READY		0
#define USB_STATE_WRITTEN	1

#define IO_USB_TIMEOUT_10_SEC	U(10000000)
#define DETACH_TIMEOUT		U(0x100)
#define USB_DFU_MAX_XFER_SIZE	1024

static uint8_t first_usb_buffer[USB_DFU_MAX_XFER_SIZE + 1] __aligned(4);
static usb_dfu_media_t usb_dfu_fops;
static uint8_t checksum_is_wrong;
static uint8_t usb_status;

/* usb device functions */
static int usb_dev_open(const uintptr_t init_params,
			io_dev_info_t **dev_info);
static int usb_block_open(io_dev_info_t *dev_info, const uintptr_t spec,
			  io_entity_t *entity);
static int usb_dev_init(io_dev_info_t *dev_info,
			const uintptr_t init_params);
static int usb_partition_size(io_entity_t *entity, size_t *length);
static int usb_block_seek(io_entity_t *entity, int mode,
			  signed long long offset);
static int usb_block_read(io_entity_t *entity, uintptr_t buffer,
			  size_t length, size_t *length_read);
static int usb_block_close(io_entity_t *entity);
static int usb_dev_close(io_dev_info_t *dev_info);
static io_type_t device_type_usb(void);

static const io_dev_connector_t usb_dev_connector = {
	.dev_open = usb_dev_open
};

static const io_dev_funcs_t usb_dev_funcs = {
	.type = device_type_usb,
	.open = usb_block_open,
	.seek = usb_block_seek,
	.size = usb_partition_size,
	.read = usb_block_read,
	.write = NULL,
	.close = usb_block_close,
	.dev_init = usb_dev_init,
	.dev_close = usb_dev_close,
};

static io_dev_info_t usb_dev_info = {
	.funcs = &usb_dev_funcs,
	.info = (uintptr_t)0,
};

/* Identify the device type as usb */
static io_type_t device_type_usb(void)
{
	return IO_TYPE_USB;
}

/* Callback to notify that data has been written in memory
 * ( set by USBD_DFU_SetDownloadAddr)
 */
static uint16_t usb_callback_write_done(uint32_t *written_in, uint32_t len)
{
	VERBOSE("%s Written_in 0x%lx len %i\n", __func__, (uintptr_t)written_in,
		len);

	/* Update SRAM state machine when block writing is finished */
	usb_status = USB_STATE_WRITTEN;

	return 0;
}

/* Call back to notify that a read memory is requested */
static uint8_t *usb_callback_read(uint8_t *src, uint8_t *dest, uint32_t len)
{
	ERROR("%s read is not supported src 0x%lx dest 0x%lx len %i\n",
	      __func__, (uintptr_t)src, (uintptr_t)dest, len);

	/* Return a valid address to avoid HardFault */
	return (uint8_t *)(dest);
}

/* Get the status to know if written operation has been checked */
static uint16_t usb_callback_get_status(void)
{
	uint16_t  status;

	/* According to SRAM state machine */
	switch (usb_status) {
	case USB_STATE_WRITTEN:
		/* The SRAM bloc writing has been done, change state machine
		 * to set SRAM in SRAM_STATE_READY
		 */
		usb_status = USB_STATE_READY;

		/* Notice caller that SRAM block writing is finished */
		status = DFU_MEDIA_STATE_WRITTEN;

		/* Checks checksum calculation result */
		if (checksum_is_wrong == 1) {
			status = DFU_MEDIA_STATE_ERROR;
			checksum_is_wrong = 0;
		}
		break;

	case USB_STATE_READY:
		/* Notice caller that SRAM is ready to be written */
		status = DFU_MEDIA_STATE_READY;
		break;

	default:
		status = DFU_MEDIA_STATE_ERROR;
		ERROR("USB unknown state\n");
		break;
	}
	VERBOSE("usb_callback_GetStatus status : %i\n", status);
	return status;
}

/* Open a connection to the usb device */
static int usb_dev_open(const uintptr_t init_params,
			io_dev_info_t **dev_info)
{
	usb_handle_t *usb_core_handle = (usb_handle_t *)init_params;

	assert(dev_info);
	*dev_info = &usb_dev_info;

	usb_dfu_fops.write_done = usb_callback_write_done;
	usb_dfu_fops.read = usb_callback_read;
	usb_dfu_fops.get_status = usb_callback_get_status;
	usb_status = USB_STATE_READY;
	checksum_is_wrong = 0;

	usb_core_handle->user_data = &usb_dfu_fops;

	usb_dev_info.info = (uintptr_t)usb_core_handle;

	return 0;
}

static int usb_dev_init(io_dev_info_t *dev_info, const uintptr_t init_params)
{
	return 0;
}

/* Close a connection to the usb device */
static int usb_dev_close(io_dev_info_t *dev_info)
{
	return 0;
}

/* Open a file on the usb device */
static int usb_block_open(io_dev_info_t *dev_info, const  uintptr_t spec,
			  io_entity_t *entity)
{
	int result;
	uint32_t length = 0;
	boot_api_image_header_t *header =
		(boot_api_image_header_t *)first_usb_buffer;

	const struct stm32image_part_info *partition_spec =
		(struct stm32image_part_info *)spec;

	/* Use PHASE_FSBL1 like init value*/
	if (current_phase.phase_id == PHASE_FSBL1) {
		assert(partition_spec);
		assert(entity);

		current_phase.current_packet = 0;

		if (!strcmp(partition_spec->name, BL33_IMAGE_NAME)) {
			/* read flash layout first for U-boot */
			current_phase.phase_id = PHASE_FLASHLAYOUT;
			current_phase.keep_header = 1;

			usb_dfu_set_phase_id(PHASE_FLASHLAYOUT);
			usb_dfu_set_download_addr((uintptr_t)
						 &first_usb_buffer[0]);

			header->magic = 0;

			while (((header->magic !=
				 BOOT_API_IMAGE_HEADER_MAGIC_NB) ||
				usb_dfu_get_current_req() == DFU_DNLOAD)) {
				usb_core_handle_it((usb_handle_t *)
						   usb_dev_info.info);
			}
			result = usb_block_read(NULL,
						FLASHLAYOUT_BASE,
						0,
						&length);
			if (result != 0) {
				return result;
			}

			flush_dcache_range((unsigned long)FLASHLAYOUT_BASE,
					   header->image_length +
					   sizeof(boot_api_image_header_t));

			current_phase.current_packet = 0;
			current_phase.keep_header = 0;
			current_phase.phase_id = PHASE_SSBL;
			current_phase.max_size = dt_get_ddr_size();
		}
		entity->info = (uintptr_t)&current_phase;
		result = 0;
	} else {
		WARN("A UART device is already active. Close first.\n");
		result = -EIO;
	}

	return result;
}

/* Return the size of a partition */
static int usb_partition_size(io_entity_t *entity, size_t *length)
{
	boot_api_image_header_t *header =
			(boot_api_image_header_t *)first_usb_buffer;
	int result = 0;

	usb_dfu_set_phase_id(current_phase.phase_id);
	usb_dfu_set_download_addr((uintptr_t)&first_usb_buffer[0]);

	header->magic = 0;

	while ((header->magic != BOOT_API_IMAGE_HEADER_MAGIC_NB) ||
	       (usb_dfu_get_current_req() == DFU_DNLOAD)) {
		usb_core_handle_it((usb_handle_t *)usb_dev_info.info);
	}

	if (header->image_length > current_phase.max_size)
		result = -EIO;
	else
		*length = header->image_length;

	INFO("%s: partition size : 0x%x\n", __func__,
	     header->image_length);

	return result;
}

/* Seek to a particular file offset on the usb device */
static int usb_block_seek(io_entity_t *entity, int mode,
			  signed long long offset)
{
	return 0;
}

/* Read data from a file on the usb device */
static int usb_block_read(io_entity_t *entity, uintptr_t buffer,
			  size_t length, size_t *length_read)
{
	uint8_t *local_ptr = (uint8_t *)buffer;
	int result = 0;
	boot_api_image_header_t *header =
		(boot_api_image_header_t *)first_usb_buffer;

	INFO("Start Download partition %i to address 0x%lx length %i\n",
	     current_phase.phase_id, buffer, length);

	if (current_phase.keep_header) {
		memcpy((uint8_t *)local_ptr,
		       (uint8_t *)&first_usb_buffer[0],
		       USB_DFU_MAX_XFER_SIZE);

		usb_dfu_set_download_addr((uintptr_t)
					  &local_ptr[USB_DFU_MAX_XFER_SIZE]);
	} else {
#if TRUSTED_BOARD_BOOT
		stm32mp_save_loaded_header(header);
#endif
		memcpy((uint8_t *)local_ptr,
		       (uint8_t *)
		       &first_usb_buffer[sizeof(boot_api_image_header_t)],
		       USB_DFU_MAX_XFER_SIZE -
		       sizeof(boot_api_image_header_t));

		usb_dfu_set_download_addr((uintptr_t)
					 &local_ptr[USB_DFU_MAX_XFER_SIZE -
					 sizeof(boot_api_image_header_t)]);
	}

	while (!usb_dfu_download_is_completed()) {
		/* Reload watchdog */
		stm32_iwdg_refresh();

		usb_core_handle_it((usb_handle_t *)usb_dev_info.info);
	}

	usb_core_handle_it((usb_handle_t *)usb_dev_info.info);
	usb_core_handle_it((usb_handle_t *)usb_dev_info.info);

	if (current_phase.keep_header)
		local_ptr += sizeof(boot_api_image_header_t);

	/* Verify header and checksum payload */
	result = stm32mp_check_header(header, (uintptr_t)local_ptr);
	if (result) {
		ERROR("Header check failed\n");
		return result;
	}

	/* Wait Detach in case of bl33 */
	if (current_phase.phase_id == PHASE_SSBL) {
		uint64_t timeout;
		uint32_t detach_timeout = DETACH_TIMEOUT;

		usb_dfu_set_phase_id(0x0);
		usb_dfu_set_download_addr(UNDEFINE_DOWN_ADDR);
		usb_dfu_request_detach();
		timeout = timeout_init_us(IO_USB_TIMEOUT_10_SEC);

		while (detach_timeout != 0U) {
			usb_core_handle_it((usb_handle_t *)
					   usb_dev_info.info);

			if (usb_dfu_detach_req() == 0U) {
				/*
				 * Continue to handle usb core IT to assure
				 * complete data transmission
				 */
				detach_timeout--;
			}

			if (timeout_elapsed(timeout)) {
				return -EIO;
			}
		}

		/* STOP the USB Handler */
		usb_core_stop((usb_handle_t *)usb_dev_info.info);
	}

	*length_read = length;

	return 0;
}

/* Close a file on the usb device */
static int usb_block_close(io_entity_t *entity)
{
	current_phase.phase_id = PHASE_FSBL1;

	return 0;
}

/* Exported functions */

/* Register the usb driver with the IO abstraction */
int register_io_dev_usb(const io_dev_connector_t **dev_con)
{
	int result;

	assert(dev_con);

	result = io_register_device(&usb_dev_info);
	if (!result)
		*dev_con = &usb_dev_connector;

	return result;
}
