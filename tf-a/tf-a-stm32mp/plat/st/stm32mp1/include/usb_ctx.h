/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef USB_CTX_H
#define USB_CTX_H

#include <lib/usb/usb_core.h>

struct usb_ctx {
	usb_handle_t *pusbd_device_ctx;
	pcd_handle_t *phpcd_ctx;
};

#endif /* USB_CTX_H */
