/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_USB_DESC_H
#define STM32MP1_USB_DESC_H

#include <lib/usb/usb_core.h>

/* Max DFU Packet Size = 1024 bytes */
#define USBD_DFU_XFER_SIZE	1024

#define TRANSFER_SIZE_BYTES(size) \
	((uint8_t)((size) & 0xFF)), /* XFERSIZEB0 */\
	((uint8_t)((size) >> 8))    /* XFERSIZEB1 */

/* Descriptor of DFU interface 0 Alternate setting n */
#define USBD_DFU_IF_DESC(n)	0x09, /* Interface Descriptor size */\
				USB_DESC_TYPE_INTERFACE, /* descriptor type */\
				0x00, /* Number of Interface */\
				(n), /* Alternate setting */\
				0x00, /* bNumEndpoints*/\
				0xFE, /* Application Specific Class Code */\
				0x01, /* Device Firmware Upgrade Code */\
				0x02, /* DFU mode protocol */ \
				USBD_IDX_INTERFACE_STR + (n) + 1 /* iInterface:
								  * Index of
								  * string
								  * descriptor
								  */
/* DFU1.1 Standard only supported */
#define USB_DFU_VERSION			0x0110
#define USBD_DESC_MAX_ITF_NUM		0x6
#define USB_DFU_CONFIG_DESC_SIZ		72
#define USB_DFU_DESC_SIZ		9
/*  String size (1 byte) + type (1 byte) + 24 UTF16 characters */
/*  (2 bytes per character) */
#define USB_SIZ_STRING_SERIAL		(1 + 1 + (24 * 2))
#define USBD_MAX_STR_DESC_SIZ		0x100
#define USBD_VID			0x0483
#define USBD_PID			0xDF11
#define USBD_LANGID_STRING		0x409
#define USBD_MANUFACTURER_STRING	"STMicroelectronics"
#define USBD_PRODUCT_HS_STRING		"DFU in HS Mode @Device ID /0x500, @Revision ID /0x0000"
#define USBD_PRODUCT_FS_STRING		"DFU in FS Mode @Device ID /0x500, @Revision ID /0x0000"
#define USBD_CONFIGURATION_HS_STRING	"DFU Config"
#define USBD_INTERFACE_HS_STRING	"DFU Interface"
#define USBD_CONFIGURATION_FS_STRING	"DFU Config"
#define USBD_INTERFACE_FS_STRING	"DFU Interface"

void stm32mp_usb_init_desc(usb_handle_t *pdev);

#endif /* STM32MP1_USB_DESC_H */
