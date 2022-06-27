/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>

#include <common/debug.h>
#include <lib/usb/usb_core.h>

/*
 * @brief  Set a STALL condition over an endpoint
 * @param  hpcd: PCD handle
 * @param  ep_addr: endpoint address
 * @retval HAL status
 */
static usb_status_t usb_core_set_stall(usb_handle_t *pdev, uint8_t ep_addr)
{
	usb_otg_ep_t *ep;
	pcd_handle_t *hpcd = (pcd_handle_t *)pdev->data;

	if ((0x80 & ep_addr) == 0x80)
		ep = &hpcd->in_ep[ep_addr & 0x7F];
	else
		ep = &hpcd->out_ep[ep_addr];

	ep->is_stall = 1;
	ep->num   = ep_addr & 0x7F;
	ep->is_in = ((ep_addr & 0x80) == 0x80);

	pdev->driver->ep_set_stall(hpcd->instance, ep);
	if ((ep_addr & 0x7F) == 0)
		pdev->driver->ep0_out_start(hpcd->instance);

	return USBD_OK;
}

/*
 * usb_core_get_desc
 *         Handle Get Descriptor requests
 * pdev : device instance
 * req : usb request
 * return : status
 */
static void usb_core_get_desc(usb_handle_t *pdev,
			      usb_setup_req_t *req)
{
	uint16_t len;
	uint8_t *pbuf;

	switch (req->value >> 8) {
	case USB_DESC_TYPE_DEVICE:
		pbuf = pdev->desc->get_device_desc(&len);
		break;

	case USB_DESC_TYPE_CONFIGURATION:
		pbuf   = (uint8_t *)pdev->desc->get_hs_config_desc(&len);
		pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
		break;

	case USB_DESC_TYPE_STRING:
		switch ((uint8_t)(req->value)) {
		case USBD_IDX_LANGID_STR:
			pbuf = pdev->desc->get_lang_id_desc(&len);
			break;

		case USBD_IDX_MFC_STR:
			pbuf = pdev->desc->get_manufacturer_desc(&len);
			break;

		case USBD_IDX_PRODUCT_STR:
			pbuf = pdev->desc->get_product_desc(&len);
			break;

		case USBD_IDX_SERIAL_STR:
			pbuf = pdev->desc->get_serial_desc(&len);
			break;

		case USBD_IDX_CONFIG_STR:
			pbuf = pdev->desc->get_configuration_desc(&len);
			break;

		case USBD_IDX_INTERFACE_STR:
			pbuf = pdev->desc->get_interface_desc(&len);
			break;

		default:
			pbuf = pdev->desc->get_usr_desc(req->value, &len);
			break;
		}
		break;

	case USB_DESC_TYPE_DEVICE_QUALIFIER:
		pbuf = (uint8_t *)pdev->desc->get_device_qualifier_desc(&len);
		break;

	case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
		pbuf = (uint8_t *)pdev->desc->get_other_speed_config_desc(&len);
		pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
		break;

	default:
		ERROR("Unknown request %i\n", req->value >> 8);
		usb_core_ctl_error(pdev);
		return;
	}

	if ((len != 0) && (req->length != 0)) {
		len = MIN(len, req->length);

		/* Set EP0 State */
		pdev->ep0_state = USBD_EP0_DATA_IN;
		pdev->ep_in[0].total_length = len;
		pdev->ep_in[0].rem_length = len;
		/* Start the transfer */
		usb_core_transmit(pdev, 0x00, pbuf, len);
	}
}

/*
 * usb_core_set_config
 *         Handle Set device configuration request
 * pdev : device instance
 * req : usb request
 * return : status
 */
static void usb_core_set_config(usb_handle_t *pdev, usb_setup_req_t *req)
{
	static uint8_t cfgidx;

	cfgidx = (uint8_t)(req->value);

	if (cfgidx > USBD_MAX_NUM_CONFIGURATION) {
		usb_core_ctl_error(pdev);
	} else {
		switch (pdev->dev_state) {
		case USBD_STATE_ADDRESSED:
			if (cfgidx) {
				pdev->dev_config = cfgidx;
				pdev->dev_state = USBD_STATE_CONFIGURED;
				if (!pdev->class) {
					usb_core_ctl_error(pdev);
					return;
				}
				/* Set configuration  and Start the Class*/
				if (pdev->class->init(pdev, cfgidx) != 0) {
					usb_core_ctl_error(pdev);
					return;
				}
			}
			break;

		case USBD_STATE_CONFIGURED:
			if (cfgidx == 0) {
				pdev->dev_state = USBD_STATE_ADDRESSED;
				pdev->dev_config = cfgidx;
				pdev->class->de_init(pdev, cfgidx);
			} else  if (cfgidx != pdev->dev_config) {
				if (!pdev->class) {
					usb_core_ctl_error(pdev);
					return;
				}
				/* Clear old configuration */
				pdev->class->de_init(pdev, pdev->dev_config);

				/* set new configuration */
				pdev->dev_config = cfgidx;
				/* Set configuration  and Start the Class*/
				if (pdev->class->init(pdev, cfgidx) != 0) {
					usb_core_ctl_error(pdev);
					return;
				}
			}
			break;

		default:
			usb_core_ctl_error(pdev);
			return;
		}
		/* Set EP0 State */
		pdev->ep0_state = USBD_EP0_STATUS_IN;

		/* Send status */
		usb_core_transmit(pdev, 0, NULL, 0);
	}
}

/*
 * usb_core_get_status
 *         Handle Get Status request
 * pdev : device instance
 * req : usb request
 * return : status
 */
static void usb_core_get_status(usb_handle_t *pdev, usb_setup_req_t *req)
{
	if ((pdev->dev_state == USBD_STATE_ADDRESSED) ||
	    (pdev->dev_state == USBD_STATE_CONFIGURED)) {
		pdev->dev_config_status = USB_CONFIG_SELF_POWERED;

		if (pdev->dev_remote_wakeup)
			pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;

		/* Set EP0 State */
		pdev->ep0_state = USBD_EP0_DATA_IN;
		pdev->ep_in[0].total_length = 2;
		pdev->ep_in[0].rem_length = 2;
		/* Start the transfer */
		usb_core_transmit(pdev, 0x00,
				  (uint8_t *)&pdev->dev_config_status, 2);
		return;
	}

	usb_core_ctl_error(pdev);
}

/*
 * usb_core_set_address
 *         Set device address
 * pdev : device instance
 * req : usb request
 * return : status
 */
static void usb_core_set_address(usb_handle_t *pdev, usb_setup_req_t *req)
{
	if ((req->index == 0) && (req->length == 0)) {
		uint8_t dev_addr = (uint8_t)(req->value) & 0x7F;

		if (pdev->dev_state == USBD_STATE_CONFIGURED) {
			usb_core_ctl_error(pdev);
		} else {
			pdev->dev_address = dev_addr;
			pdev->driver->set_address(((pcd_handle_t *)
						   (pdev->data))->instance,
						  dev_addr);
			/* Set EP0 State */
			pdev->ep0_state = USBD_EP0_STATUS_IN;

			/* Send status */
			usb_core_transmit(pdev, 0, NULL, 0);

			if (dev_addr != 0)
				pdev->dev_state  = USBD_STATE_ADDRESSED;
			else
				pdev->dev_state  = USBD_STATE_DEFAULT;
		}
	} else {
		usb_core_ctl_error(pdev);
	}
}

/*
 * usb_core_dev_req
 *         Handle standard usb device requests
 * pdev : device instance
 * req : usb request
 * return : status
 */
static usb_status_t usb_core_dev_req(usb_handle_t *pdev, usb_setup_req_t *req)
{
	INFO("receive request %i\n", req->b_request);
	switch (req->b_request) {
	case USB_REQ_GET_DESCRIPTOR:
		usb_core_get_desc(pdev, req);
		break;

	case USB_REQ_SET_CONFIGURATION:
		usb_core_set_config(pdev, req);
		break;

	case USB_REQ_GET_STATUS:
		usb_core_get_status(pdev, req);
		break;
	case USB_REQ_SET_ADDRESS:
		usb_core_set_address(pdev, req);
		break;
	case USB_REQ_GET_CONFIGURATION:
	case USB_REQ_SET_FEATURE:
	case USB_REQ_CLEAR_FEATURE:
	default:
		ERROR("NOT SUPPORTED %i\n", req->b_request);
		usb_core_ctl_error(pdev);
		break;
	}

	return USBD_OK;
}

/*
 * usb_core_itf_req
 *         Handle standard usb interface requests
 * pdev : device instance
 * req : usb request
 * return : status
 */
static usb_status_t usb_core_itf_req(usb_handle_t *pdev, usb_setup_req_t *req)
{
	switch (pdev->dev_state) {
	case USBD_STATE_CONFIGURED:
		if (LOBYTE(req->index) <= USBD_MAX_NUM_INTERFACES) {
			pdev->class->setup(pdev, req);

			if (req->length == 0) {
				/* Set EP0 State */
				pdev->ep0_state = USBD_EP0_STATUS_IN;

				usb_core_transmit(pdev, 0, NULL, 0);
			}
		} else {
			usb_core_ctl_error(pdev);
		}
		break;

	default:
		usb_core_ctl_error(pdev);
		break;
	}
	return USBD_OK;
}

/*
 * @brief  USBD_ParseSetupRequest
 *         Copy buffer into setup structure
 * @param  pdev: device instance
 * @param  req: usb request
 * @retval None
 */
static void usb_core_parse_req(usb_setup_req_t *req, uint8_t *pdata)
{
	req->bm_request = *(uint8_t *)(pdata);
	req->b_request = *(uint8_t *)(pdata + 1);
	req->value = SWAPBYTE(pdata +  2);
	req->index = SWAPBYTE(pdata +  4);
	req->length = SWAPBYTE(pdata +  6);
}

/*
 * usb_core_setup_stage
 *         Handle the setup stage
 * pdev: device instance
 * return : status
 */
static usb_status_t usb_core_setup_stage(usb_handle_t *pdev, uint8_t *psetup)
{
	usb_core_parse_req(&pdev->request, psetup);

	pdev->ep0_state = USBD_EP0_SETUP;
	pdev->ep0_data_len = pdev->request.length;

	switch (pdev->request.bm_request & 0x1F) {
	case USB_REQ_RECIPIENT_DEVICE:
		usb_core_dev_req(pdev, &pdev->request);
		break;

	case USB_REQ_RECIPIENT_INTERFACE:
		usb_core_itf_req(pdev, &pdev->request);
		break;

	case USB_REQ_RECIPIENT_ENDPOINT:
	default:
		ERROR("receive unsupported request %i",
		      pdev->request.bm_request & 0x1F);
		usb_core_set_stall(pdev, pdev->request.bm_request & 0x80);
		return USBD_FAIL;
	}
	return USBD_OK;
}

/*
 * usb_core_data_out
 *         Handle data OUT stage
 * pdev: device instance
 * epnum: endpoint index
 * return : status
 */
static usb_status_t usb_core_data_out(usb_handle_t *pdev, uint8_t epnum,
				      uint8_t *pdata)
{
	usb_endpoint_t *pep;

	if (epnum == 0) {
		pep = &pdev->ep_out[0];
		if (pdev->ep0_state == USBD_EP0_DATA_OUT) {
			if (pep->rem_length > pep->maxpacket) {
				pep->rem_length -=  pep->maxpacket;

				usb_core_receive(pdev, 0, pdata,
						 MIN(pep->rem_length,
						     pep->maxpacket));
			} else {
				if (pdev->class->ep0_rx_ready &&
				    (pdev->dev_state == USBD_STATE_CONFIGURED))
					pdev->class->ep0_rx_ready(pdev);

				pdev->ep0_state = USBD_EP0_STATUS_IN;
				usb_core_transmit(pdev, 0x00, NULL, 0);
			}
		}
	} else if (pdev->class->data_out &&
		   (pdev->dev_state == USBD_STATE_CONFIGURED))
		pdev->class->data_out(pdev, epnum);

	return USBD_OK;
}

/*
 * usb_core_data_in
 *         Handle data in stage
 * pdev: device instance
 * epnum: endpoint index
 * return : status
 */
static usb_status_t usb_core_data_in(usb_handle_t *pdev, uint8_t epnum,
				     uint8_t *pdata)
{
	if (epnum == 0) {
		usb_endpoint_t *pep = &pdev->ep_in[0];

		if (pdev->ep0_state == USBD_EP0_DATA_IN) {
			if (pep->rem_length > pep->maxpacket) {
				pep->rem_length -=  pep->maxpacket;

				usb_core_transmit(pdev, 0, pdata,
						  pep->rem_length);

				/* Prepare endpoint for premature
				 * end of transfer
				 */
				usb_core_receive(pdev, 0, NULL, 0);
			} else {
				/* last packet is MPS multiple,
				 * so send ZLP packet
				 */
				if ((pep->total_length % pep->maxpacket == 0) &&
				    (pep->total_length >= pep->maxpacket) &&
				    (pep->total_length < pdev->ep0_data_len)) {
					usb_core_transmit(pdev, 0, NULL, 0);

					pdev->ep0_data_len = 0;

					/* Prepare endpoint for premature
					 * end of transfer
					 */
					usb_core_receive(pdev, 0, NULL, 0);
				} else {
					if (pdev->class->ep0_tx_sent &&
					    (pdev->dev_state ==
					     USBD_STATE_CONFIGURED))
						pdev->class->ep0_tx_sent(pdev);

					/* Set EP0 State */
					pdev->ep0_state = USBD_EP0_STATUS_OUT;

					/* Start the transfer */
					usb_core_receive(pdev, 0, NULL, 0);
				}
			}
		}
		if (pdev->dev_test_mode == 1) {
			ERROR("Not supported");
			pdev->dev_test_mode = 0;
			return USBD_FAIL;
		}
	} else if (pdev->class->data_in &&
		  (pdev->dev_state == USBD_STATE_CONFIGURED)) {
		pdev->class->data_in(pdev, epnum);
	}
	return USBD_OK;
}

/*
 * usb_core_Suspend
 *         Handle Suspend event
 * pdev : device instance
 * return : status
 */

static usb_status_t usb_core_suspend(usb_handle_t  *pdev)
{
	INFO("USB Suspend mode\n");

	pdev->dev_old_state =  pdev->dev_state;
	pdev->dev_state  = USBD_STATE_SUSPENDED;

	return USBD_OK;
}

/*
 * usb_core_resume
 *         Handle Resume event
 * pdev : device instance
 * return : status
 */

static usb_status_t usb_core_resume(usb_handle_t *pdev)
{
	INFO("USB Resume\n");
	pdev->dev_state = pdev->dev_old_state;

	return USBD_OK;
}

/*
 * usb_core_sof
 *         Handle SOF event
 * pdev : device instance
 * return : status
 */

static usb_status_t usb_core_sof(usb_handle_t *pdev)
{
	if (pdev->dev_state == USBD_STATE_CONFIGURED) {
		if (pdev->class->sof)
			pdev->class->sof(pdev);
	}

	return USBD_OK;
}

/*
 * usb_core_DevDisconnected
 *         Handle device disconnection event
 * pdev : device instance
 * return : status
 */
static usb_status_t usb_core_disconnect(usb_handle_t *pdev)
{
	/* Free Class Resources */
	pdev->dev_state = USBD_STATE_DEFAULT;
	pdev->class->de_init(pdev, pdev->dev_config);

	return USBD_OK;
}

usb_status_t usb_core_handle_it(usb_handle_t *pdev)
{
	uint32_t param = 0;
	uint32_t len = 0;
	usb_otg_ep_t *ep;

	switch (pdev->driver->it_handler(pdev->data->instance, &param)) {
	case USB_DATA_OUT:
		usb_core_data_out(pdev, param,
				  pdev->data->out_ep[param].xfer_buff);
		break;
	case USB_DATA_IN:
		usb_core_data_in(pdev, param,
				 pdev->data->in_ep[param].xfer_buff);
		break;
	case USB_SETUP:
		usb_core_setup_stage(pdev, (uint8_t *)pdev->data->setup);
		break;
	case USB_ENUM_DONE:
		pdev->data->init.speed = USB_OTG_SPEED_HIGH;
		pdev->data->init.ep0_mps = USB_OTG_HS_MAX_PACKET_SIZE;
		break;
	case USB_READ_DATA_PACKET:
		ep = &pdev->data->out_ep[param & USB_OTG_OUT_EPNUM_MASK];
		len = (param & USB_OTG_OUT_COUNT_MASK) >> 0x10;
		pdev->driver->read_packet(pdev->data->instance,
					  ep->xfer_buff, len);
		ep->xfer_buff += len;
		ep->xfer_count += len;
		break;
	case USB_READ_SETUP_PACKET:
		ep = &pdev->data->out_ep[param & USB_OTG_OUT_EPNUM_MASK];
		len = (param & USB_OTG_OUT_COUNT_MASK) >> 0x10;
		pdev->driver->read_packet(pdev->data->instance,
					  (uint8_t *)pdev->data->setup, 8);
		ep->xfer_count += len;
		break;
	case USB_RESUME:
		if (pdev->data->lpm_state == LPM_L1)
			pdev->data->lpm_state = LPM_L0;
		else
			usb_core_resume(pdev);
		break;
	case USB_SUSPEND:
		usb_core_suspend(pdev);
		break;
	case USB_LPM:
		if (pdev->data->lpm_state == LPM_L0) {
			pdev->data->lpm_state = LPM_L1;
			pdev->data->besl = param;
		} else {
			usb_core_suspend(pdev);
		}
		break;
	case USB_SOF:
		usb_core_sof(pdev);
		break;
	case USB_DISCONNECT:
		usb_core_disconnect(pdev);
		break;
	case USB_WRITE_EMPTY:
		pdev->driver->write_empty_tx_fifo(pdev->data->instance, param,
				     pdev->data->in_ep[param].xfer_len,
				     (uint32_t *)
					&pdev->data->in_ep[param].xfer_count,
				     pdev->data->in_ep[param].maxpacket,
				     &pdev->data->in_ep[param].xfer_buff);
		break;
	case USB_NOTHING:
	default:
		break;
	}
	return USBD_OK;
}

/**
 * @brief  Receive an amount of data
 * @param  hpcd: PCD handle
 * @param  ep_addr: endpoint address
 * @param  pBuf: pointer to the reception buffer
 * @param  len: amount of data to be received
 * @retval HAL status
 */
usb_status_t usb_core_receive(usb_handle_t *pdev, uint8_t ep_addr,
			      uint8_t *buf, uint32_t len)
{
	usb_otg_ep_t *ep;
	pcd_handle_t *hpcd = (pcd_handle_t *)pdev->data;

	ep = &hpcd->out_ep[ep_addr & 0x7F];

	/*setup and start the Xfer */
	ep->xfer_buff = buf;
	ep->xfer_len = len;
	ep->xfer_count = 0;
	ep->is_in = 0;
	ep->num = ep_addr & 0x7F;

	if ((ep_addr & 0x7F) == 0)
		pdev->driver->ep0_start_xfer(hpcd->instance, ep);
	else
		pdev->driver->ep_start_xfer(hpcd->instance, ep);

	return USBD_OK;
}

/*
 * @brief  Send an amount of data
 * @param  hpcd: PCD handle
 * @param  ep_addr: endpoint address
 * @param  pBuf: pointer to the transmission buffer
 * @param  len: amount of data to be sent
 * @retval HAL status
 */
usb_status_t usb_core_transmit(usb_handle_t *pdev, uint8_t ep_addr,
			       uint8_t *buf, uint32_t len)
{
	usb_otg_ep_t *ep;
	pcd_handle_t *hpcd = (pcd_handle_t *)pdev->data;

	ep = &hpcd->in_ep[ep_addr & 0x7F];

	/*setup and start the Xfer */
	ep->xfer_buff = buf;
	ep->xfer_len = len;
	ep->xfer_count = 0;
	ep->is_in = 1;
	ep->num = ep_addr & 0x7F;

	if ((ep_addr & 0x7F) == 0)
		pdev->driver->ep0_start_xfer(hpcd->instance, ep);
	else
		pdev->driver->ep_start_xfer(hpcd->instance, ep);

	return USBD_OK;
}

/*
 * @brief  usb_core_ctl_error
 *         Handle USB low level Error
 * @param  pdev: device instance
 * @param  req: usb request
 * @retval None
 */

void usb_core_ctl_error(usb_handle_t *pdev)
{
	ERROR("%s : Send an ERROR\n", __func__);
	usb_core_set_stall(pdev, 0x80);
	usb_core_set_stall(pdev, 0);
}

/*
 * usb_core_stop
 *         Stop the USB Device Core.
 * pdev: Device Handle
 * return : USBD Status
 */
usb_status_t usb_core_stop(usb_handle_t *pdev)
{
	/* Free Class Resources */
	pdev->class->de_init(pdev, pdev->dev_config);

	/* Stop the low level driver */
	pdev->driver->disable_int(pdev->data->instance);
	pdev->driver->stop_device(pdev->data->instance);
	pdev->driver->dev_disconnect(pdev->data->instance);
	return USBD_OK;
}

/*
 * usb_core_stop
 *         Stop the USB Device Core.
 * pdev: Device Handle
 * return : USBD Status
 */
usb_status_t register_usb_driver(usb_handle_t *pdev, const usb_driver_t *driver,
				 void *driver_handle)
{
	/* Free Class Resources */
	pdev->driver = driver;
	pdev->data->instance = driver_handle;
	return USBD_OK;
}

/*
 * usb_core_stop
 *         Stop the USB Device Core.
 * pdev: Device Handle
 * return : USBD Status
 */
usb_status_t register_platform(usb_handle_t *pdev,
			       const usb_desc_t *plat_call_back)
{
	/* Free Class Resources */
	pdev->desc = plat_call_back;
	return USBD_OK;
}
