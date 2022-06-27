/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/st/usb_dwc2.h>

static usb_dwc2_t dwc2_handle;

static const usb_driver_t usb_dwc2driver = {
	.disable_int = usb_dwc2_disable_int,
	.ep0_out_start = usb_dwc2_ep0_out_start,
	.ep_start_xfer = usb_dwc2_ep_start_xfer,
	.ep0_start_xfer = usb_dwc2_ep0_start_xfer,
	.write_packet = usb_dwc2_write_packet,
	.read_packet = usb_dwc2_read_packet,
	.ep_set_stall = usb_dwc2_ep_set_stall,
	.stop_device = usb_dwc2_stop_device,
	.set_address = usb_dwc2_set_address,
	.dev_disconnect = usb_dwc2_dev_disconnect,
	.write_empty_tx_fifo = usb_dwc2_write_empty_tx_fifo,
	.it_handler = usb_dwc2_it_handler
};

/*
 * USB_OTG_FlushTxFifo : Flush a Tx FIFO
 * USBx : Selected device
 * num : FIFO number
 *       This parameter can be a value from 1 to 15
 *       15 means Flush all Tx FIFOs
 * return : status
 */
static usb_status_t usb_dwc2_flush_tx_fifo(usb_dwc2_global_t *usbx,
					   uint32_t num)
{
	uint32_t count = 0;

	usbx->grstctl = (USB_OTG_GRSTCTL_TXFFLSH | (uint32_t)(num << 6));

	do {
		if (++count > 200000)
			return USBD_TIMEOUT;
	} while ((usbx->grstctl & USB_OTG_GRSTCTL_TXFFLSH) ==
		 USB_OTG_GRSTCTL_TXFFLSH);

	return USBD_OK;
}

/*
 * USB_FlushRxFifo : Flush Rx FIFO
 * param : USBx : Selected device
 * return : status
 */
static usb_status_t usb_dwc2_flush_rx_fifo(usb_dwc2_global_t *usbx)
{
	uint32_t count = 0;

	usbx->grstctl = USB_OTG_GRSTCTL_RXFFLSH;

	do {
		if (++count > 200000)
			return USBD_TIMEOUT;
	} while ((usbx->grstctl & USB_OTG_GRSTCTL_RXFFLSH) ==
		  USB_OTG_GRSTCTL_RXFFLSH);

	return USBD_OK;
}

/*
 * USB_ReadInterrupts: return the global USB interrupt status
 * param  USBx : Selected device
 * return : interrupt register value
 */
static uint32_t usb_dwc2_read_int(usb_dwc2_global_t *usbx)
{
	uint32_t v = 0;

	v = usbx->gintsts;
	v &= usbx->gintmsk;

	return v;
}

/*
 * usb_dwc2_all_out_ep_int : return the USB device OUT endpoints interrupt
 * param : USBx : Selected device
 * return : device OUT endpoint interrupts
 */
static uint32_t usb_dwc2_all_out_ep_int(usb_dwc2_global_t *usbx)
{
	uint32_t v = 0;

	v  = dwc2_handle.usb_device->daint;
	v &= dwc2_handle.usb_device->daintmsk;

	return ((v & 0xffff0000) >> 16);
}

/*
 * usb_dwc2_all_in_ep_int: return the USB device IN endpoints interrupt
 * param : USBx : Selected device
 * return : device IN endpoint interrupts
 */
static uint32_t usb_dwc2_all_in_ep_int(usb_dwc2_global_t *usbx)
{
	uint32_t v = 0;

	v  = dwc2_handle.usb_device->daint;
	v &= dwc2_handle.usb_device->daintmsk;

	return ((v & 0xFFFF));
}

/*
 * usb_dwc2_out_ep_int : returns Device OUT EP Interrupt register
 * USBx : Selected device
 * epnum : endpoint number
 *         This parameter can be a value from 0 to 15
 * return : Device OUT EP Interrupt register
 */
static uint32_t usb_dwc2_out_ep_int(usb_dwc2_global_t *usbx, uint8_t epnum)
{
	uint32_t v = 0;

	v  = dwc2_handle.usb_out_endpoint[epnum]->epint;
	v &= dwc2_handle.usb_device->doepmsk;

	return v;
}

/*
 * usb_dwc2_in_ep_int : Returns Device IN EP Interrupt register
 * param : USBx : Selected device
 * param : epnum : endpoint number
 *         This parameter can be a value from 0 to 15
 * return : Device IN EP Interrupt register
 */
static uint32_t usb_dwc2_in_ep_int(usb_dwc2_global_t *usbx, uint8_t epnum)
{
	uint32_t msk, emp;

	msk = dwc2_handle.usb_device->diepmsk;
	emp = dwc2_handle.usb_device->diepempmsk;
	msk |= ((emp >> epnum) & 0x1) << 7;

	return (dwc2_handle.usb_in_endpoint[epnum]->epint & msk);
}

/*
 * usb_dwc2_get_mode : Returns USB core mode
 * param :  USBx : Selected device
 * return : core mode : Host or Device
 *          This parameter can be one of the these values:
 *           0 : Host
 *           1 : Device
 */
static uint32_t usb_dwc2_get_mode(usb_dwc2_global_t *usbx)
{
	return ((usbx->gintsts) & 0x1);
}

/*
 * usb_dwc2_activate_setup : Activate EP0 for Setup transactions
 * param : USBx : Selected device
 * return : status
 */
static usb_status_t usb_dwc2_activate_setup(usb_dwc2_global_t *usbx)
{
	/* Set the MPS of the IN EP based on the enumeration speed */
	dwc2_handle.usb_in_endpoint[0]->epctl &= ~USB_OTG_DIEPCTL_MPSIZ;

	if ((dwc2_handle.usb_device->dsts & USB_OTG_DSTS_ENUMSPD) ==
	   DSTS_ENUMSPD_LS_PHY_6MHZ)
		dwc2_handle.usb_in_endpoint[0]->epctl |= 3;

	dwc2_handle.usb_device->dctl |= USB_OTG_DCTL_CGINAK;

	return USBD_OK;
}

/*
 * usb_dwc2_disable_int :
 *         Disable the controller's Global Int in the AHB Config reg
 * param : handle : Selected device
 * return : status
 */
usb_status_t usb_dwc2_disable_int(void *handle)
{
	usb_dwc2_global_t *usbx = ((usb_dwc2_t *)handle)->usb_global;

	usbx->gahbcfg &= ~USB_OTG_GAHBCFG_GINT;
	return USBD_OK;
}

/*
 * usb_dwc2_ep0_out_start : Prepare the EP0 to start the first control setup
 * param : handle : Selected device
 * return : status
 */
usb_status_t usb_dwc2_ep0_out_start(void *handle)
{
	/*usb_dwc2_global_t *USBx = (usb_dwc2_global_t *)handle;*/

	dwc2_handle.usb_out_endpoint[0]->eptsiz = 0;
	dwc2_handle.usb_out_endpoint[0]->eptsiz |= (USB_OTG_DOEPTSIZ_PKTCNT &
						    (1 << 19));
	dwc2_handle.usb_out_endpoint[0]->eptsiz |= (3 * 8);
	dwc2_handle.usb_out_endpoint[0]->eptsiz |=  USB_OTG_DOEPTSIZ_STUPCNT;

	return USBD_OK;
}

/*
 * usb_dwc2_ep_start_xfer : setup and starts a transfer over an EP
 * param : handle : Selected device
 * param : ep: pointer to endpoint structure
 * return : status
 */
usb_status_t usb_dwc2_ep_start_xfer(void *handle, usb_otg_ep_t *ep)
{
	usb_dwc2_global_t *usbx = ((usb_dwc2_t *)handle)->usb_global;

	/* IN endpoint */
	if (ep->is_in == 1) {
		/* Zero Length Packet? */
		if (ep->xfer_len == 0) {
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz &=
					~(USB_OTG_DIEPTSIZ_PKTCNT);
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz |=
					(USB_OTG_DIEPTSIZ_PKTCNT & (1 << 19));
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz &=
					~(USB_OTG_DIEPTSIZ_XFRSIZ);
		} else {
			/* Program the transfer size and packet count
			 * as follows: xfersize = N * maxpacket +
			 * short_packet pktcnt = N + (short_packet
			 * exist ? 1 : 0)
			 */
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz &=
					~(USB_OTG_DIEPTSIZ_XFRSIZ);
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz &=
					~(USB_OTG_DIEPTSIZ_PKTCNT);
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz |=
					(USB_OTG_DIEPTSIZ_PKTCNT &
					 (((ep->xfer_len + ep->maxpacket - 1) /
					   ep->maxpacket) << 19));
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz |=
					(USB_OTG_DIEPTSIZ_XFRSIZ &
					 ep->xfer_len);

			if (ep->type == EP_TYPE_ISOC) {
				dwc2_handle.usb_in_endpoint[ep->num]->eptsiz &=
						~(USB_OTG_DIEPTSIZ_MULCNT);
				dwc2_handle.usb_in_endpoint[ep->num]->eptsiz |=
						(USB_OTG_DIEPTSIZ_MULCNT &
						 (1 << 29));
			}
		}

		if (ep->type != EP_TYPE_ISOC) {
			/* Enable the Tx FIFO Empty Interrupt for this EP */
			if (ep->xfer_len > 0)
				dwc2_handle.usb_device->diepempmsk |=
								1 << ep->num;
		}

		if (ep->type == EP_TYPE_ISOC) {
			if ((dwc2_handle.usb_device->dsts & (1 << 8)) == 0) {
				dwc2_handle.usb_in_endpoint[ep->num]->epctl |=
						USB_OTG_DIEPCTL_SODDFRM;
			} else {
				dwc2_handle.usb_in_endpoint[ep->num]->epctl |=
						USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
			}
		}

		/* EP enable, IN data in FIFO */
		dwc2_handle.usb_in_endpoint[ep->num]->epctl |=
						(USB_OTG_DIEPCTL_CNAK |
						USB_OTG_DIEPCTL_EPENA);

		if (ep->type == EP_TYPE_ISOC)
			usb_dwc2_write_packet(usbx, ep->xfer_buff,
					      ep->num, ep->xfer_len);
	} else {
		/* Program the transfer size and packet count as follows:
		 * pktcnt = N
		 * xfersize = N * maxpacket
		 */
		dwc2_handle.usb_out_endpoint[ep->num]->eptsiz &=
						~(USB_OTG_DOEPTSIZ_XFRSIZ);
		dwc2_handle.usb_out_endpoint[ep->num]->eptsiz &=
						~(USB_OTG_DOEPTSIZ_PKTCNT);

		if (ep->xfer_len == 0) {
			dwc2_handle.usb_out_endpoint[ep->num]->eptsiz |=
					(USB_OTG_DOEPTSIZ_XFRSIZ &
					 ep->maxpacket);
			dwc2_handle.usb_out_endpoint[ep->num]->eptsiz |=
					(USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19));
		} else {
			uint16_t pktcnt = (ep->xfer_len + ep->maxpacket - 1) /
				ep->maxpacket;

			dwc2_handle.usb_out_endpoint[ep->num]->eptsiz |=
					(USB_OTG_DOEPTSIZ_PKTCNT &
					 (pktcnt << 19));
			dwc2_handle.usb_out_endpoint[ep->num]->eptsiz |=
					(USB_OTG_DOEPTSIZ_XFRSIZ &
					 (ep->maxpacket * pktcnt));
		}

		if (ep->type == EP_TYPE_ISOC) {
			if ((dwc2_handle.usb_device->dsts & (1 << 8)) == 0)
				dwc2_handle.usb_out_endpoint[ep->num]->epctl |=
						USB_OTG_DOEPCTL_SODDFRM;
			else
				dwc2_handle.usb_out_endpoint[ep->num]->epctl |=
						USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
		}
		/* EP enable */
		dwc2_handle.usb_out_endpoint[ep->num]->epctl |=
				(USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}
	return USBD_OK;
}

/*
 * usb_dwc2_ep0_start_xfer : setup and starts a transfer over the EP  0
 * param : handle : Selected device
 * param : ep: pointer to endpoint structure
 * return : status
 */
usb_status_t usb_dwc2_ep0_start_xfer(void *handle, usb_otg_ep_t *ep)
{
	/*usb_dwc2_global_t *USBx = (usb_dwc2_global_t *)handle;*/

	/* IN endpoint */
	if (ep->is_in == 1) {
		/* Zero Length Packet? */
		if (ep->xfer_len == 0) {
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz &=
					~(USB_OTG_DIEPTSIZ_PKTCNT);
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz |=
					(USB_OTG_DIEPTSIZ_PKTCNT & (1 << 19));
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz &=
					~(USB_OTG_DIEPTSIZ_XFRSIZ);
		} else {
			/* Program the transfer size and packet count
			 * as follows: xfersize = N * maxpacket +
			 * short_packet pktcnt = N + (short_packet
			 * exist ? 1 : 0)
			 */
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz &=
					~(USB_OTG_DIEPTSIZ_XFRSIZ);
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz &=
					~(USB_OTG_DIEPTSIZ_PKTCNT);

			if (ep->xfer_len > ep->maxpacket)
				ep->xfer_len = ep->maxpacket;

			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz |=
					(USB_OTG_DIEPTSIZ_PKTCNT & (1 << 19));
			dwc2_handle.usb_in_endpoint[ep->num]->eptsiz |=
					(USB_OTG_DIEPTSIZ_XFRSIZ &
					 ep->xfer_len);
		}

		/* Enable the Tx FIFO Empty Interrupt for this EP */
		if (ep->xfer_len > 0)
			dwc2_handle.usb_device->diepempmsk |= 1 << (ep->num);

		/* EP enable, IN data in FIFO */
		dwc2_handle.usb_in_endpoint[ep->num]->epctl |=
				(USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	} else {
		/* Program the transfer size and packet count as follows:
		 * pktcnt = N
		 * xfersize = N * maxpacket
		 */
		dwc2_handle.usb_out_endpoint[ep->num]->eptsiz &=
						~(USB_OTG_DOEPTSIZ_XFRSIZ);
		dwc2_handle.usb_out_endpoint[ep->num]->eptsiz &=
						~(USB_OTG_DOEPTSIZ_PKTCNT);

		if (ep->xfer_len > 0)
			ep->xfer_len = ep->maxpacket;

		dwc2_handle.usb_out_endpoint[ep->num]->eptsiz |=
						(USB_OTG_DOEPTSIZ_PKTCNT &
						(1 << 19));
		dwc2_handle.usb_out_endpoint[ep->num]->eptsiz |=
						(USB_OTG_DOEPTSIZ_XFRSIZ &
						(ep->maxpacket));

		/* EP enable */
		dwc2_handle.usb_out_endpoint[ep->num]->epctl |=
						(USB_OTG_DOEPCTL_CNAK |
						USB_OTG_DOEPCTL_EPENA);
	}
	return USBD_OK;
}

/*
 * usb_dwc2_write_packet : Writes a packet into the Tx FIFO associated
 *         with the EP/channel
 * param : handle : Selected device
 * param : src :  pointer to source buffer
 * param : ch_ep_num : endpoint or host channel number
 * param : len : Number of bytes to write
 * return : status
 */
usb_status_t usb_dwc2_write_packet(void *handle, uint8_t *src,
				   uint8_t ch_ep_num, uint16_t len)
{
	uint32_t count32b, i, j;
	/*usb_dwc2_global_t *USBx = (usb_dwc2_global_t *)handle;*/

	count32b =  (len + 3) / 4;
	for (i = 0; i < count32b; i++, src += 4) {
		uint32_t src_copy = 0;

		/* Data written to fifo need to be 4 bytes aligned */
		for (j = 0; j < 4; j++)
			src_copy += (*(src + j)) << (8 * j);

		*dwc2_handle.usb_fifo[ch_ep_num] = src_copy;
	}

	return USBD_OK;
}

/*
 * usb_dwc2_read_packet : read a packet from the Tx FIFO associated
 *         with the EP/channel
 * param : handle : Selected device
 * param : src : source pointer
 * param : ch_ep_num : endpoint or host channel number
 * param : len : Number of bytes to read
 * return : pointer to destination buffer
 */
void *usb_dwc2_read_packet(void *handle, uint8_t *dest, uint16_t len)
{
	uint32_t i = 0;
	uint32_t count32b = (len + 3) / 4;
	/*usb_dwc2_global_t *USBx = (usb_dwc2_global_t *)handle;*/

	VERBOSE("read packet length %i to 0x%lx\n", len, (uintptr_t)dest);

	for (i = 0; i < count32b; i++, dest += 4) {
		*(uint32_t *)dest = *dwc2_handle.usb_fifo[0];
		dsb();
	}

	return ((void *)dest);
}

/*
 * usb_dwc2_EPSetStall : set a stall condition over an EP
 * param : handle : Selected device
 * param : ep: pointer to endpoint structure
 * return : status
 */
usb_status_t usb_dwc2_ep_set_stall(void *handle, usb_otg_ep_t *ep)
{
	/*usb_dwc2_global_t *USBx = (usb_dwc2_global_t *)handle;*/
	if (ep->is_in == 1) {
		if ((dwc2_handle.usb_in_endpoint[ep->num]->epctl &
						USB_OTG_DIEPCTL_EPENA) == 0)
			dwc2_handle.usb_in_endpoint[ep->num]->epctl &=
						~(USB_OTG_DIEPCTL_EPDIS);
		dwc2_handle.usb_in_endpoint[ep->num]->epctl |=
						USB_OTG_DIEPCTL_STALL;
	} else {
		if ((dwc2_handle.usb_out_endpoint[ep->num]->epctl &
						USB_OTG_DOEPCTL_EPENA) == 0)
			dwc2_handle.usb_out_endpoint[ep->num]->epctl &=
						~(USB_OTG_DOEPCTL_EPDIS);
		dwc2_handle.usb_out_endpoint[ep->num]->epctl |=
						USB_OTG_DOEPCTL_STALL;
	}
	return USBD_OK;
}

/*
 * usb_dwc2_stop_device : Stop the usb device mode
 * param : handle : Selected device
 * return : status
 */
usb_status_t usb_dwc2_stop_device(void *handle)
{
	uint32_t i = 0;
	usb_dwc2_global_t *usbx = ((usb_dwc2_t *)handle)->usb_global;

	/* Clear Pending interrupt */
	for (i = 0; i < 15 ; i++) {
		dwc2_handle.usb_in_endpoint[i]->epint = 0xFF;
		dwc2_handle.usb_out_endpoint[i]->epint = 0xFF;
	}
	dwc2_handle.usb_device->daint = 0xFFFFFFFF;

	/* Clear interrupt masks */
	dwc2_handle.usb_device->diepmsk  = 0;
	dwc2_handle.usb_device->doepmsk  = 0;
	dwc2_handle.usb_device->daintmsk = 0;

	/* Flush the FIFO */
	usb_dwc2_flush_rx_fifo(usbx);
	usb_dwc2_flush_tx_fifo(usbx, 0x10);

	return USBD_OK;
}

/*
 * usb_dwc2_set_address : Stop the usb device mode
 * param : handle : Selected device
 * param : address : new device address to be assigned
 *          This parameter can be a value from 0 to 255
 * return : status
 */
usb_status_t usb_dwc2_set_address(void *handle, uint8_t address)
{
	/*usb_dwc2_global_t *USBx = (usb_dwc2_global_t *)handle;*/

	dwc2_handle.usb_device->dcfg &= ~(USB_OTG_DCFG_DAD);
	dwc2_handle.usb_device->dcfg |= (address << 4) & USB_OTG_DCFG_DAD;

	return USBD_OK;
}

/*
 * usb_dwc2_dev_disconnect :
 *	Disconnect the USB device by disabling the pull-up/pull-down
 * param : handle : Selected device
 * return : status
 */
usb_status_t usb_dwc2_dev_disconnect(void *handle)
{
	/*usb_dwc2_global_t *USBx = (usb_dwc2_global_t *)handle;*/

	dwc2_handle.usb_device->dctl |= USB_OTG_DCTL_SDIS;

	return USBD_OK;
}

/*
 * usb_dwc2_write_empty_tx_fifo
 *         check FIFO for the next packet to be loaded
 * param : handle : Selected device
 * param : epnum : endpoint number
 * param : xfer_len : block length
 * param : xfer_count : number of block
 * param : maxpacket : max packet length
 * param : xfer_buff : buffer pointer
 * retval : status
 */
usb_status_t usb_dwc2_write_empty_tx_fifo(void *handle,
					  uint32_t epnum, uint32_t xfer_len,
					  uint32_t *xfer_count,
					  uint32_t maxpacket,
					  uint8_t **xfer_buff)
{
	int32_t len = 0;
	uint32_t len32b;
	usb_dwc2_global_t *usbx = ((usb_dwc2_t *)handle)->usb_global;

	len = xfer_len - *xfer_count;

	if ((len > 0) && ((uint32_t)len > maxpacket))
		len = maxpacket;

	len32b = (len + 3) / 4;

	while ((dwc2_handle.usb_in_endpoint[epnum]->txfsts &
		USB_OTG_DTXFSTS_INEPTFSAV) > len32b &&
		(*xfer_count < xfer_len) && (xfer_len != 0)) {
		/* Write the FIFO */
		len = xfer_len - *xfer_count;

		if ((len > 0) && ((uint32_t)len > maxpacket))
			len = maxpacket;

		len32b = (len + 3) / 4;

		usb_dwc2_write_packet(usbx, *xfer_buff, epnum, len);

		*xfer_buff  += len;
		*xfer_count += len;
	}

	if (len <= 0) {
		uint32_t fifoemptymsk = 0x1 << epnum;

		dwc2_handle.usb_device->diepempmsk &= ~fifoemptymsk;
	}

	return USBD_OK;
}

/*
 * @brief  This function handles PCD interrupt request.
 * @param  hpcd: PCD handle
 * @retval HAL status
 */
usb_action_t usb_dwc2_it_handler(void *handle, uint32_t *param)
{
	usb_dwc2_global_t *usbx = ((usb_dwc2_t *)handle)->usb_global;
	uint32_t ep_intr, epint, epnum = 0;
	uint32_t temp;

	/* ensure that we are in device mode */
	if (usb_dwc2_get_mode(usbx) != USB_OTG_MODE_DEVICE)
		return USB_NOTHING;

	/* avoid spurious interrupt */
	if (!usb_dwc2_read_int(usbx))
		return USB_NOTHING;

	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_MMIS)
		/* incorrect mode, acknowledge the interrupt */
		usbx->gintsts = USB_OTG_GINTSTS_MMIS;

	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_OEPINT) {
		/* Read in the device interrupt bits */
		ep_intr = usb_dwc2_all_out_ep_int(usbx);

		while (!(ep_intr & 1)) {
			epnum++;
			ep_intr >>= 1;
		}

		if (ep_intr & 1) {
			epint = usb_dwc2_out_ep_int(usbx, epnum);

			if ((epint & USB_OTG_DOEPINT_XFRC) ==
						USB_OTG_DOEPINT_XFRC) {
				dwc2_handle.usb_out_endpoint[epnum]->epint =
						USB_OTG_DOEPINT_XFRC;
				*param = epnum;
				return USB_DATA_OUT;
			}
			if ((epint & USB_OTG_DOEPINT_STUP) ==
						USB_OTG_DOEPINT_STUP) {
				/* Inform the upper layer that a setup packet
				 *  is available
				 */
				dwc2_handle.usb_out_endpoint[epnum]->epint =
						USB_OTG_DOEPINT_STUP;
				return USB_SETUP;
			}
			if ((epint & USB_OTG_DOEPINT_OTEPDIS) ==
						USB_OTG_DOEPINT_OTEPDIS)
				dwc2_handle.usb_out_endpoint[epnum]->epint =
						USB_OTG_DOEPINT_OTEPDIS;
		}
	}
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_IEPINT) {
		/* Read in the device interrupt bits */
		ep_intr = usb_dwc2_all_in_ep_int(usbx);

		while (!(ep_intr & 1)) {
			epnum++;
			ep_intr >>= 1;
		}
		/* In ITR */
		if (ep_intr & 0x1) {
			epint = usb_dwc2_in_ep_int(usbx, epnum);

			if ((epint & USB_OTG_DIEPINT_XFRC) ==
						USB_OTG_DIEPINT_XFRC) {
				uint32_t fifoemptymsk = 0x1 << epnum;

				dwc2_handle.usb_device->diepempmsk &=
						~fifoemptymsk;

				dwc2_handle.usb_in_endpoint[epnum]->epint =
						USB_OTG_DIEPINT_XFRC;

				*param = epnum;
				return USB_DATA_IN;
			}
			if ((epint & USB_OTG_DIEPINT_TOC) ==
						USB_OTG_DIEPINT_TOC)
				dwc2_handle.usb_in_endpoint[epnum]->epint =
						USB_OTG_DIEPINT_TOC;

			if ((epint & USB_OTG_DIEPINT_ITTXFE) ==
						USB_OTG_DIEPINT_ITTXFE)
				dwc2_handle.usb_in_endpoint[epnum]->epint =
						USB_OTG_DIEPINT_ITTXFE;

			if ((epint & USB_OTG_DIEPINT_INEPNE) ==
						USB_OTG_DIEPINT_INEPNE)
				dwc2_handle.usb_in_endpoint[epnum]->epint =
						USB_OTG_DIEPINT_INEPNE;

			if ((epint & USB_OTG_DIEPINT_EPDISD) ==
						USB_OTG_DIEPINT_EPDISD)
				dwc2_handle.usb_in_endpoint[epnum]->epint =
						USB_OTG_DIEPINT_EPDISD;

			if ((epint & USB_OTG_DIEPINT_TXFE) ==
						USB_OTG_DIEPINT_TXFE) {
				*param = epnum;
				return USB_WRITE_EMPTY;
			}
		}
	}

	/* Handle Resume Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_WKUINT) {
		INFO("handle USB : Resume\n");
		/* Clear the Remote Wake-up Signaling */
		dwc2_handle.usb_device->dctl &= ~USB_OTG_DCTL_RWUSIG;
		usbx->gintsts = USB_OTG_GINTSTS_WKUINT;
		return USB_RESUME;
	}

	/* Handle Suspend Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_USBSUSP) {
		INFO("handle USB : Suspend int\n");
		usbx->gintsts = USB_OTG_GINTSTS_USBSUSP;
		if ((dwc2_handle.usb_device->dsts & USB_OTG_DSTS_SUSPSTS) ==
				USB_OTG_DSTS_SUSPSTS){
			return USB_SUSPEND;
		}
	}

	/* Handle LPM Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_LPMINT) {
		INFO("handle USB : LPM int enter in suspend\n");
		usbx->gintsts = USB_OTG_GINTSTS_LPMINT;
		*param = (usbx->glpmcfg & USB_OTG_GLPMCFG_BESL) >> 2;
		return USB_LPM;
	}

	/* Handle Reset Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_USBRST) {
		INFO("handle USB : Reset\n");
		dwc2_handle.usb_device->dctl &= ~USB_OTG_DCTL_RWUSIG;
		usb_dwc2_flush_tx_fifo(usbx, 0);

		dwc2_handle.usb_device->daint = 0xFFFFFFFF;
		dwc2_handle.usb_device->daintmsk |= 0x10001;

		dwc2_handle.usb_device->doepmsk |= (USB_OTG_DOEPMSK_STUPM |
					 USB_OTG_DOEPMSK_XFRCM |
					 USB_OTG_DOEPMSK_EPDM);
		dwc2_handle.usb_device->diepmsk |= (USB_OTG_DIEPMSK_TOM |
					 USB_OTG_DIEPMSK_XFRCM |
					 USB_OTG_DIEPMSK_EPDM);

		/* Set Default Address to 0 */
		dwc2_handle.usb_device->dcfg &= ~USB_OTG_DCFG_DAD;

		/* setup EP0 to receive SETUP packets */
		usb_dwc2_ep0_out_start(usbx);

		usbx->gintsts = USB_OTG_GINTSTS_USBRST;
	}

	/* Handle Enumeration done Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_ENUMDNE) {
		usb_dwc2_activate_setup(usbx);
		usbx->gusbcfg &= ~USB_OTG_GUSBCFG_TRDT;

		usbx->gusbcfg |= (uint32_t)((USBD_HS_TRDT_VALUE << 10) &
					    USB_OTG_GUSBCFG_TRDT);

		usbx->gintsts = USB_OTG_GINTSTS_ENUMDNE;
		return USB_ENUM_DONE;
	}

	/* Handle RxQLevel Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_RXFLVL) {
		usbx->gintmsk &= ~USB_OTG_GINTSTS_RXFLVL;
		temp = usbx->grxstsp;
		*param = (temp & USB_OTG_GRXSTSP_EPNUM);
		*param |= ((temp & USB_OTG_GRXSTSP_BCNT) << 0xC);

		if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_DATA_UPDT) {
			if ((temp & USB_OTG_GRXSTSP_BCNT) != 0) {
				usbx->gintmsk |= USB_OTG_GINTSTS_RXFLVL;
				return USB_READ_DATA_PACKET;
			}
		} else if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==
							STS_SETUP_UPDT) {
			usbx->gintmsk |= USB_OTG_GINTSTS_RXFLVL;
			return USB_READ_SETUP_PACKET;
		}
		usbx->gintmsk |= USB_OTG_GINTSTS_RXFLVL;
	}

	/* Handle SOF Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_SOF) {
		INFO("handle USB : SOF\n");
		usbx->gintsts = USB_OTG_GINTSTS_SOF;
		return USB_SOF;
	}

	/* Handle Incomplete ISO IN Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_IISOIXFR) {
		INFO("handle USB : ISO IN\n");
		usbx->gintsts = USB_OTG_GINTSTS_IISOIXFR;
	}

	/* Handle Incomplete ISO OUT Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT) {
		INFO("handle USB : ISO OUT\n");
		usbx->gintsts = USB_OTG_GINTSTS_PXFR_INCOMPISOOUT;
	}

	/* Handle Connection event Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_SRQINT) {
		INFO("handle USB : Connect\n");
		usbx->gintsts = USB_OTG_GINTSTS_SRQINT;
	}

	/* Handle Disconnection event Interrupt */
	if (usb_dwc2_read_int(usbx) & USB_OTG_GINTSTS_OTGINT) {
		INFO("handle USB : Disconnect\n");
		temp = usbx->gotgint;
		if ((temp & USB_OTG_GOTGINT_SEDET) == USB_OTG_GOTGINT_SEDET)
			return USB_DISCONNECT;
	}
	return USB_NOTHING;
}

void usb_dwc2_init_driver(usb_handle_t *usb_core_handle,
			  uint32_t *base_register)
{
	uint32_t i = 0;
	uintptr_t base = (uintptr_t)base_register;

	dwc2_handle.usb_global = (usb_dwc2_global_t *)base;

	dwc2_handle.usb_device = (usb_dwc2_device_t *)
					(base + USB_OTG_DEVICE_BASE);

	for (i = 0; i < USB_MAX_ENDPOINT_NB; i++) {
		dwc2_handle.usb_in_endpoint[i] = (usb_dwc2_endpoint_t *)
					(base + USB_OTG_IN_ENDPOINT_BASE +
					 (i * sizeof(usb_dwc2_endpoint_t)));
		dwc2_handle.usb_out_endpoint[i] = (usb_dwc2_endpoint_t *)
					(base + USB_OTG_OUT_ENDPOINT_BASE +
					(i * sizeof(usb_dwc2_endpoint_t)));
		dwc2_handle.usb_fifo[i] = (uint32_t *)(base +
						       USB_OTG_FIFO_BASE +
						       (i * USB_OTG_FIFO_SIZE));
	}

	register_usb_driver(usb_core_handle, &usb_dwc2driver,
			    (void *)&dwc2_handle);
}
