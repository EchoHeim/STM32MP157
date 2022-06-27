/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <limits.h>
#include <string.h>

#include <platform_def.h>

#include <common/debug.h>
#include <drivers/st/bsec.h>
#include <lib/usb/usb_core.h>
#include <lib/usb/usb_st_dfu.h>

#include <stm32mp_common.h>
#include <stm32mp1_usb_desc.h>

/* USB Standard Device Descriptor */
static const uint8_t usb_stm32mp1_desc[USB_LEN_DEV_DESC] = {
	USB_LEN_DEV_DESC,           /* bLength */
	USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
	0x00,                       /* bcdUSB */
	0x02,                       /* version */
	0x00,                       /* bDeviceClass */
	0x00,                       /* bDeviceSubClass */
	0x00,                       /* bDeviceProtocol */
	USB_MAX_EP0_SIZE,           /* bMaxPacketSize */
	LOBYTE(USBD_VID),           /* idVendor */
	HIBYTE(USBD_VID),           /* idVendor */
	LOBYTE(USBD_PID),           /* idVendor */
	HIBYTE(USBD_PID),           /* idVendor */
	0x00,                       /* bcdDevice rel. 2.00 */
	0x02,
	USBD_IDX_MFC_STR,           /* Index of manufacturer string */
	USBD_IDX_PRODUCT_STR,       /* Index of product string */
	USBD_IDX_SERIAL_STR,        /* Index of serial number string */
	USBD_MAX_NUM_CONFIGURATION  /* bNumConfigurations */
}; /* USB_DeviceDescriptor */

/* USB Standard String Descriptor */
static const uint8_t usb_stm32mp1_lang_id_desc[USB_LEN_LANGID_STR_DESC] = {
	USB_LEN_LANGID_STR_DESC,
	USB_DESC_TYPE_STRING,
	LOBYTE(USBD_LANGID_STRING),
	HIBYTE(USBD_LANGID_STRING),
};

/* USB Standard Device Descriptor */
static const uint8_t
usbd_stm32mp1_qualifier_desc[USB_LEN_DEV_QUALIFIER_DESC] = {
	USB_LEN_DEV_QUALIFIER_DESC,
	USB_DESC_TYPE_DEVICE_QUALIFIER,
	0x00,
	0x02,
	0x00,
	0x00,
	0x00,
	0x40,
	0x01,
	0x00,
};

static uint8_t usb_stm32mp1_serial[USB_SIZ_STRING_SERIAL + 1] = {
	USB_SIZ_STRING_SERIAL,
	USB_DESC_TYPE_STRING,
};

/* USB DFU device Configuration Descriptor */
static uint8_t usb_stm32mp1_config_desc[USB_DFU_CONFIG_DESC_SIZ] = {
	0x09, /* bLength: Configuration Descriptor size */
	USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
	USB_DFU_CONFIG_DESC_SIZ,
	/* wTotalLength: Bytes returned */
	0x00,
	0x01,/* bNumInterfaces: 1 interface*/
	0x01,/* bConfigurationValue: Configuration value*/
	0x02,/* iConfiguration: Index of string descriptor
	      *	describing the configuration
	      */
	0xC0,/* bmAttributes: bus powered and Supprts Remote Wakeup */
	0x32,/* MaxPower 100 mA: this current is used for detecting Vbus */
	/* 09 */

	/* Descriptor of DFU interface 0 Alternate setting 0 */
	USBD_DFU_IF_DESC(0), /* This interface is mandatory for all devices */

	/* Descriptor of DFU interface 0 Alternate setting 1 */
	USBD_DFU_IF_DESC(1),

	/* Descriptor of DFU interface 0 Alternate setting 2 */
	USBD_DFU_IF_DESC(2),

	/* Descriptor of DFU interface 0 Alternate setting 3 */
	USBD_DFU_IF_DESC(3),

	/* Descriptor of DFU interface 0 Alternate setting 4 */
	USBD_DFU_IF_DESC(4),

	/* Descriptor of DFU interface 0 Alternate setting 5 */
	USBD_DFU_IF_DESC(5),

	/* DFU Functional Descriptor */
	0x09,/* blength = 9 Bytes */
	DFU_DESCRIPTOR_TYPE,/* DFU Functional Descriptor*/
	DFU_BM_ATTRIBUTE,/* bmAttribute
			  * bitCanDnload		= 1 (bit 0)
			  * bitCanUpload		= 1 (bit 1)
			  * bitManifestationTolerant	= 1 (bit 2)
			  * bitWillDetach		= 1 (bit 3)
			  * Reserved			(bit4-6)
			  * bitAcceleratedST		= 0 (bit 7)
			  */
	0xFF,/* DetachTimeOut = 255 ms */
	0x00,
	/* WARNING: In DMA mode the multiple MPS packets feature
	 *  is still not supported ==> In this case,
	 *  when using DMA USBD_DFU_XFER_SIZE should be set
	 *  to 64 in usbd_conf.h
	 */
	TRANSFER_SIZE_BYTES(USBD_DFU_XFER_SIZE),/* TransferSize = 1024 Byte*/
	((USB_DFU_VERSION >> 0) & 0xFF), /* bcdDFUVersion*/
	((USB_DFU_VERSION >> 8) & 0xFF)
};

static uint8_t usb_local_string_dec[USBD_MAX_STR_DESC_SIZ];

/*
 * Convert Hex 32Bits value into char
 * value: value to convert
 * pbuf: pointer to the buffer
 * len: buffer length
 */
static void int_to_unicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
	uint8_t idx = 0;

	for (idx = 0; idx < len; idx++) {
		if (((value >> 28)) < 0xA)
			pbuf[2 * idx] = (value >> 28) + '0';
		else
			pbuf[2 * idx] = (value >> 28) + 'A' - 10;
		value = value << 4;
		pbuf[(2 * idx) + 1] = 0;
	}
}

/*
 * Create the serial number string descriptor
 */
static void update_serial_num_string(void)
{
	/* serial number is set to 0*/
	uint8_t i;
	uint32_t deviceserial[UID_WORD_NB] = {0U, 0U, 0U};
	uint32_t otp;
	uint32_t len;

	if (stm32_get_otp_index(UID_OTP, &otp, &len) != 0) {
		ERROR("BSEC: Get UID_OTP number Error\n");
		return;
	}

	if ((len / __WORD_BIT) != UID_WORD_NB) {
		ERROR("BSEC: Get UID_OTP length Error\n");
		return;
	}

	for (i = 0; i < UID_WORD_NB; i++) {
		if (bsec_shadow_read_otp(&deviceserial[i], i + otp) !=
		    BSEC_OK) {
			ERROR("BSEC: UID%d Error\n", i);
			return;
		}
	}

	int_to_unicode(deviceserial[0], (uint8_t *)&usb_stm32mp1_serial[2], 8);
	int_to_unicode(deviceserial[1], (uint8_t *)&usb_stm32mp1_serial[18], 8);
	int_to_unicode(deviceserial[2], (uint8_t *)&usb_stm32mp1_serial[34], 8);
}

/*
 * usb_get_qualifier_desc
 *         return Device Qualifier descriptor
 * param :  length : pointer data length
 * return : pointer to descriptor buffer
 */
static uint8_t *stm32mp1_get_qualifier_desc(uint16_t *length)
{
	*length = sizeof(usbd_stm32mp1_qualifier_desc);
	return (uint8_t *)usbd_stm32mp1_qualifier_desc;
}

/*
 * stm32mp1_get_dfu_desc
 *         return Device Qualifier descriptor
 * param :  length : pointer data length
 * return : pointer to descriptor buffer
 */
static uint8_t *stm32mp1_get_dfu_desc(uint16_t *len)
{
	*len = USB_DFU_DESC_SIZ;
	return ((uint8_t *)usb_stm32mp1_config_desc + (9 * 7));
}

/*
 * stm32mp1_get_config_desc
 *         return configuration descriptor
 * param : speed : current device speed
 * param : length : pointer data length
 * return : pointer to descriptor buffer
 */
static uint8_t *stm32mp1_get_config_desc(uint16_t *length)
{
	*length = sizeof(usb_stm32mp1_config_desc);
	return (uint8_t *)usb_stm32mp1_config_desc;
}

/*
 * stm32mp1_get_string
 *         Convert Ascii string into unicode one
 * param : desc : descriptor buffer
 * param : unicode : Formatted string buffer (unicode)
 * param : len : descriptor length
 * return : None
 */
static void stm32mp1_get_string(uint8_t *desc, uint8_t *unicode, uint16_t *len)
{
	uint8_t idx = 0;

	if (!desc)
		return;

	*len =  strlen((char *)desc) * 2 + 2;
	unicode[idx++] = *len;
	unicode[idx++] =  USB_DESC_TYPE_STRING;

	while (*desc != '\0') {
		unicode[idx++] = *desc++;
		unicode[idx++] =  0x00;
	}
}

/*
 * stm32mp1_device_desc
 * Returns the device descriptor.
 * length: Pointer to data length variable
 * return : Pointer to descriptor buffer
 */
static uint8_t *stm32mp1_device_desc(uint16_t *length)
{
	*length = sizeof(usb_stm32mp1_desc);
	return (uint8_t *)usb_stm32mp1_desc;
}

/*
 * stm32mp1_lang_id_desc
 * Returns the LangID string descriptor.
 * speed: Current device speed
 * length: Pointer to data length variable
 * return : Pointer to descriptor buffer
 */
static uint8_t *stm32mp1_lang_id_desc(uint16_t *length)
{
	*length = sizeof(usb_stm32mp1_lang_id_desc);

	return (uint8_t *)usb_stm32mp1_lang_id_desc;
}

/*
 * stm32mp1_product_desc
 * Returns the product string descriptor.
 * length: Pointer to data length variable
 * return : Pointer to descriptor buffer
 */
static uint8_t *stm32mp1_product_desc(uint16_t *length)
{
	stm32mp1_get_string((uint8_t *)USBD_PRODUCT_HS_STRING,
			    usb_local_string_dec, length);

	return usb_local_string_dec;
}

/*
 * stm32mp1_manufacturer_desc
 * Returns the manufacturer string descriptor.
 * length: Pointer to data length variable
 * return : Pointer to descriptor buffer
 */
static uint8_t *stm32mp1_manufacturer_desc(uint16_t *length)
{
	stm32mp1_get_string((uint8_t *)USBD_MANUFACTURER_STRING,
			    usb_local_string_dec, length);

	return usb_local_string_dec;
}

/*
 * stm32mp1_serial_desc
 * Returns the serial number string descriptor.
 * length: Pointer to data length variable
 * return : Pointer to descriptor buffer
 */
static uint8_t *stm32mp1_serial_desc(uint16_t *length)
{
	*length = USB_SIZ_STRING_SERIAL;

	/* Update the serial number string descriptor
	 * with the data from the unique ID
	 */
	update_serial_num_string();

	return (uint8_t *)usb_stm32mp1_serial;
}

/*
 * stm32mp1_Config_desc
 * Returns the configuration string descriptor.
 * length: Pointer to data length variable
 * return : Pointer to descriptor buffer
 */
static uint8_t *stm32mp1_config_desc(uint16_t *length)
{
	stm32mp1_get_string((uint8_t *)USBD_CONFIGURATION_HS_STRING,
			    usb_local_string_dec, length);

	return usb_local_string_dec;
}

/*
 * stm32mp1_interface_desc
 * Returns the interface string descriptor.
 * length : Pointer to data length variable
 * return : Pointer to descriptor buffer
 */
static uint8_t *stm32mp1_interface_desc(uint16_t *length)
{
	stm32mp1_get_string((uint8_t *)USBD_INTERFACE_HS_STRING,
			    usb_local_string_dec, length);

	return usb_local_string_dec;
}

/*
 * stm32mp1_get_usr_desc
 *         Manages the transfer of memory interfaces string descriptors.
 * param : index: descriptor index
 * param : length : pointer data length
 * return : pointer to the descriptor table or NULL if the descriptor
 *          is not supported.
 */
static uint8_t *stm32mp1_get_usr_desc(uint8_t index, uint16_t *length)
{
	uint8_t *ret;

	if (index > (USBD_IDX_INTERFACE_STR + USBD_DESC_MAX_ITF_NUM))
		return NULL;

	switch (index) {
	case 6:
		stm32mp1_get_string((uint8_t *)"@Partition0 /0x00/1*256Ke",
				    usb_local_string_dec, length);
		ret = usb_local_string_dec;
		break;
	case 7:
		stm32mp1_get_string((uint8_t *)"@FSBL /0x01/1*1Me",
				    usb_local_string_dec, length);
		ret = usb_local_string_dec;
		break;
	case 8:
		stm32mp1_get_string((uint8_t *)"@Partition2 /0x02/1*1Me",
				    usb_local_string_dec, length);
		ret = usb_local_string_dec;
		break;
	case 9:
		stm32mp1_get_string((uint8_t *)"@Partition3 /0x03/1*16Me",
				    usb_local_string_dec, length);
		ret = usb_local_string_dec;
		break;
	case 10:
		stm32mp1_get_string((uint8_t *)"@Partition4 /0x04/1*16Me",
				    usb_local_string_dec, length);
		ret = usb_local_string_dec;
		break;
	case 11:
		stm32mp1_get_string((uint8_t *)"@virtual /0xF1/1*512Ba",
				    usb_local_string_dec, length);
		ret = usb_local_string_dec;
		break;
	default:
		ret = NULL;
		break;
	}

	return ret;
}

static const usb_desc_t dfu_desc = {
	.get_device_desc = stm32mp1_device_desc,
	.get_lang_id_desc = stm32mp1_lang_id_desc,
	.get_manufacturer_desc = stm32mp1_manufacturer_desc,
	.get_product_desc = stm32mp1_product_desc,
	.get_configuration_desc = stm32mp1_config_desc,
	.get_serial_desc = stm32mp1_serial_desc,
	.get_interface_desc = stm32mp1_interface_desc,
	.get_usr_desc = stm32mp1_get_usr_desc,
	.get_hs_config_desc = stm32mp1_get_config_desc,
	.get_fs_config_desc = stm32mp1_get_config_desc,
	.get_other_speed_config_desc = stm32mp1_get_config_desc,
	.get_device_qualifier_desc = stm32mp1_get_qualifier_desc,
	.get_dfu_desc = stm32mp1_get_dfu_desc
};

void stm32mp_usb_init_desc(usb_handle_t *pdev)
{
	register_platform(pdev, &dfu_desc);
}
