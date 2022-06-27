/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef USB_ST_DFU_H
#define USB_ST_DFU_H

#include <stdint.h>

#include <lib/usb/usb_core.h>

#define DFU_DESCRIPTOR_TYPE		0x21

/* bmAttribute :
 * bitCanDnload = 1(bit 0)
 * bitCanUpload = 1(bit 1)
 * bitManifestationTolerant = 1 (bit 2)
 * bitWillDetach = 1(bit 3)
 * Reserved (bit4-6)
 * bitAcceleratedST = 0(bit 7)
 */
#define DFU_BM_ATTRIBUTE		0x0F

#define DFU_GET_PHASE			0x5

/* DFU Requests  DFU states */
#define APP_STATE_IDLE			0
#define APP_STATE_DETACH		1
#define DFU_STATE_IDLE			2
#define DFU_STATE_DNLOAD_SYNC		3
#define DFU_STATE_DNLOAD_BUSY		4
#define DFU_STATE_DNLOAD_IDLE		5
#define DFU_STATE_MANIFEST_SYNC		6
#define DFU_STATE_MANIFEST		7
#define DFU_STATE_MANIFEST_WAIT_RESET	8
#define DFU_STATE_UPLOAD_IDLE		9
#define DFU_STATE_ERROR			10

/* DFU errors */
#define DFU_ERROR_NONE			0x00
#define DFU_ERROR_TARGET		0x01
#define DFU_ERROR_FILE			0x02
#define DFU_ERROR_WRITE			0x03
#define DFU_ERROR_ERASE			0x04
#define DFU_ERROR_CHECK_ERASED		0x05
#define DFU_ERROR_PROG			0x06
#define DFU_ERROR_VERIFY		0x07
#define DFU_ERROR_ADDRESS		0x08
#define DFU_ERROR_NOTDONE		0x09
#define DFU_ERROR_FIRMWARE		0x0A
#define DFU_ERROR_VENDOR		0x0B
#define DFU_ERROR_USB			0x0C
#define DFU_ERROR_POR			0x0D
#define DFU_ERROR_UNKNOWN		0x0E
#define DFU_ERROR_STALLEDPKT		0x0F

/* DFU Manifestation State */
#define DFU_MANIFEST_COMPLETE		0x00
#define DFU_MANIFEST_IN_PROGRESS	0x01

/* Special Commands  with Download Request */
#define DFU_CMD_GETCOMMANDS		0x00
#define DFU_CMD_SETADDRESSPOINTER	0x21
#define DFU_CMD_ERASE			0x41

#define DFU_MEDIA_STATE_READY		0x00
#define DFU_MEDIA_STATE_WRITTEN		0x01
#define DFU_MEDIA_STATE_ERROR		0x02

/* Bit Detach capable = bit 3 in bmAttributes field */
#define DFU_DETACH_MASK			(uint8_t)(1 << 4)
#define DFU_STATUS_DEPTH		(6)

/* Undefined download address */
#define UNDEFINE_DOWN_ADDR		0xFFFFFFFF

typedef enum {
	DFU_DETACH = 0,
	DFU_DNLOAD,
	DFU_UPLOAD,
	DFU_GETSTATUS,
	DFU_CLRSTATUS,
	DFU_GETSTATE,
	DFU_ABORT
} dfu_request_t;

typedef void (*p_function)(void);

typedef struct {
	uint8_t buffer[10];
	uint8_t dev_state;
	uint8_t dev_status[DFU_STATUS_DEPTH];
	uint8_t manif_state;
	uint32_t wblock_num;
	uint32_t wlength;
	uintptr_t data_ptr;
	uint32_t alt_setting;
} usb_dfu_handle_t;

typedef struct {
	uint16_t (*write_done)(uint32_t *written_in, uint32_t len);
	uint8_t* (*read)(uint8_t *src, uint8_t *dest, uint32_t len);
	uint16_t (*get_status)(void);
} usb_dfu_media_t;

void usb_dfu_register_callback(usb_handle_t *pdev);
void usb_dfu_set_phase_id(uint32_t phase_id);
void usb_dfu_set_download_addr(uintptr_t addr);
uint32_t usb_dfu_download_is_completed(void);
uint32_t usb_dfu_get_current_req(void);
uint32_t usb_dfu_detach_req(void);
void usb_dfu_request_detach(void);

#endif /* USB_ST_DFU_H */
