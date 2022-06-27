/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include <platform_def.h>

#include <common/debug.h>
#include <lib/usb/usb_st_dfu.h>

static uintptr_t usbd_dfu_download_address;
static uint32_t usbd_dfu_phase_id;
static uint32_t usbd_dfu_operation_complete;
static uint32_t usbd_dfu_current_req;
static uint32_t usbd_detach_req;

/*
 * @brief  USBD_DFU_Init
 *         Initialize the DFU interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t usb_dfu_init(usb_handle_t *pdev, uint8_t cfgidx)
{
	/* Nothing to do in this stage */
	return USBD_OK;
}

/**
 * @brief  USBD_DFU_Init
 *         De-Initialize the DFU layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t usb_dfu_de_init(usb_handle_t *pdev, uint8_t cfgidx)
{
	/* Nothing to do in this stage */
	return USBD_OK;
}

/*
 * @brief  USBD_DFU_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t  usb_dfu_data_in(usb_handle_t *pdev, uint8_t epnum)
{
	(void)pdev;
	(void)epnum;

	return USBD_OK;
}

/*
 * @brief  DFU_Leave
 *         Handles the sub-protocol DFU leave DFU mode request (leaves DFU mode
 *         and resets device to jump to user loaded code).
 * @param  pdev: device instance
 * @retval None
 */
static void usb_dfu_leave(usb_handle_t *pdev)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	hdfu->manif_state = DFU_MANIFEST_COMPLETE;

	if (DFU_BM_ATTRIBUTE & 0x04) {
		hdfu->dev_state = DFU_STATE_MANIFEST_SYNC;

		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0;
		hdfu->dev_status[4] = hdfu->dev_state;
	} else {
		hdfu->dev_state = DFU_STATE_MANIFEST_WAIT_RESET;

		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0;
		hdfu->dev_status[4] = hdfu->dev_state;

		/* Disconnect the USB device */
		usb_core_stop(pdev);
	}
}

/*
 * @brief  USBD_DFU_EP0_RxReady
 *         handle EP0 Rx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t usb_dfu_ep0_rx_ready(usb_handle_t *pdev)
{
	(void)pdev;

	return USBD_OK;
}

/*
 * @brief  USBD_DFU_EP0_TxReady
 *         handle EP0 TRx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t usb_dfu_ep0_tx_ready(usb_handle_t *pdev)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;
	uint16_t len, dfu_version = 0;
	uint8_t *serial = pdev->desc->get_dfu_desc(&len);

	dfu_version = serial[len - 1] << 8 | serial[len - 2];

	if (hdfu->dev_state == DFU_STATE_DNLOAD_BUSY) {
		if (dfu_version == 0x011a) {
			/* Decode the Special Command*/
			if (hdfu->wblock_num == 0) {
				if (hdfu->buffer[0] ==
				     DFU_CMD_SETADDRESSPOINTER &&
				    hdfu->wlength == 5) {
					hdfu->data_ptr  = hdfu->buffer[1];
					hdfu->data_ptr +=
						hdfu->buffer[2] << 8;
					hdfu->data_ptr +=
						hdfu->buffer[3] << 16;
					hdfu->data_ptr +=
						hdfu->buffer[4] << 24;
				} else if (hdfu->buffer[0] ==
					    DFU_CMD_ERASE &&
					   hdfu->wlength == 5) {
					hdfu->data_ptr  = hdfu->buffer[1];
					hdfu->data_ptr +=
						hdfu->buffer[2] << 8;
					hdfu->data_ptr +=
						hdfu->buffer[3] << 16;
					hdfu->data_ptr +=
						hdfu->buffer[4] << 24;
				} else {
				/* Reset the global length and block number */
					hdfu->wlength = 0;
					hdfu->wblock_num = 0;
					/* Call the error management function
					 * (command will be nacked)
					 */
					usb_core_ctl_error(pdev);
				}
			}
		}
		if ((hdfu->wblock_num > 1 && dfu_version == 0x011a) ||
		    dfu_version != 0x011a) {
			/* Perform the write operation */
			if (((usb_dfu_media_t *)
			     pdev->user_data)->write_done((uint32_t *)
							  hdfu->data_ptr,
							  hdfu->wlength)
			    != USBD_OK)
				return USBD_FAIL;
		}

		/* Reset the global length and block number */
		hdfu->wlength = 0;
		hdfu->wblock_num = 0;

		/* Update the state machine */
		hdfu->dev_state =  DFU_STATE_DNLOAD_SYNC;
		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0;
		hdfu->dev_status[4] = hdfu->dev_state;
		return USBD_OK;
	} else if (hdfu->dev_state == DFU_STATE_MANIFEST) {
		/* Manifestation in progress*/
		/* Start leaving DFU mode */
		usb_dfu_leave(pdev);
	}

	return USBD_OK;
}

/*
 * @brief  USBD_DFU_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t usb_dfu_sof(usb_handle_t *pdev)
{
	(void)pdev;

	return USBD_OK;
}

/*
 * @brief  USBD_DFU_IsoINIncomplete
 *         handle data ISO IN Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t usb_dfu_iso_in_incomplete(usb_handle_t *pdev, uint8_t epnum)
{
	(void)pdev;
	(void)epnum;
	return USBD_OK;
}

/*
 * @brief  USBD_DFU_IsoOutIncomplete
 *         handle data ISO OUT Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t usb_dfu_iso_out_incomplete(usb_handle_t *pdev, uint8_t epnum)
{
	(void)pdev;
	(void)epnum;
	return USBD_OK;
}

/*
 * @brief  USBD_DFU_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t usb_dfu_data_out(usb_handle_t *pdev, uint8_t epnum)
{
	(void)pdev;
	(void)epnum;
	return USBD_OK;
}

/*
 * @brief  DFU_Detach
 *         Handles the DFU DETACH request.
 * @param  pdev: device instance
 * @param  req: pointer to the request structure.
 * @retval None.
 */
static void usb_dfu_detach(usb_handle_t *pdev, usb_setup_req_t *req)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	INFO("Receive Detach\n");

	if (hdfu->dev_state == DFU_STATE_IDLE ||
	    hdfu->dev_state == DFU_STATE_DNLOAD_SYNC ||
	    hdfu->dev_state == DFU_STATE_DNLOAD_IDLE ||
	    hdfu->dev_state == DFU_STATE_MANIFEST_SYNC ||
	    hdfu->dev_state == DFU_STATE_UPLOAD_IDLE) {
		/* Update the state machine */
		hdfu->dev_state = DFU_STATE_IDLE;
		hdfu->dev_status[0] = DFU_ERROR_NONE;
		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0; /*bwPollTimeout=0ms*/
		hdfu->dev_status[4] = hdfu->dev_state;
		hdfu->dev_status[5] = 0; /*iString*/
		hdfu->wblock_num = 0;
	}
	hdfu->wlength = 0;

	usbd_detach_req = 0;
}

/*
 * @brief  DFU_Download
 *         Handles the DFU DNLOAD request.
 * @param  pdev: device instance
 * @param  req: pointer to the request structure
 * @retval None
 */
static void usb_dfu_download(usb_handle_t *pdev, usb_setup_req_t *req)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	/* Data setup request */
	if (req->length > 0) {
		if ((hdfu->dev_state == DFU_STATE_IDLE) ||
		    (hdfu->dev_state == DFU_STATE_DNLOAD_IDLE)) {
			/* Update the global length and block number */
			hdfu->wblock_num = req->value;
			hdfu->wlength = req->length;

			/* Update the data address */
			hdfu->data_ptr = usbd_dfu_download_address;

			/* Update the state machine */
			hdfu->dev_state = DFU_STATE_DNLOAD_SYNC;
			hdfu->dev_status[4] = hdfu->dev_state;

			/* Prepare the reception of the buffer over EP0 */
			/* Set EP0 State */
			pdev->ep0_state = USBD_EP0_DATA_OUT;
			pdev->ep_out[0].total_length = hdfu->wlength;
			pdev->ep_out[0].rem_length   = hdfu->wlength;

			/* Start the transfer */
			usb_core_receive(pdev,
					 0,
					 (uint8_t *)usbd_dfu_download_address,
					 hdfu->wlength);

			usbd_dfu_download_address += hdfu->wlength;
		} else {
			/* Unsupported state */
			/* Call the error management function
			 * (command will be nacked)
			 */
			usb_core_ctl_error(pdev);
		}
	} else {
		/* End of DNLOAD operation*/
		if (hdfu->dev_state == DFU_STATE_DNLOAD_IDLE ||
		    hdfu->dev_state == DFU_STATE_IDLE) {
			hdfu->manif_state = DFU_MANIFEST_IN_PROGRESS;
			hdfu->dev_state = DFU_STATE_MANIFEST_SYNC;
			hdfu->dev_status[1] = 0;
			hdfu->dev_status[2] = 0;
			hdfu->dev_status[3] = 0;
			hdfu->dev_status[4] = hdfu->dev_state;
		} else {
			/* Call the error management function
			 * (command will be nacked)
			 */
			usb_core_ctl_error(pdev);
		}
	}
}

/*
 * @brief  DFU_Upload
 *         Handles the DFU UPLOAD request.
 * @param  pdev: instance
 * @param  req: pointer to the request structure
 * @retval status
 */
static void usb_dfu_upload(usb_handle_t *pdev, usb_setup_req_t *req)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	/* Data setup request */
	if (req->length > 0) {
		if ((hdfu->dev_state == DFU_STATE_IDLE) ||
		    (hdfu->dev_state == DFU_STATE_UPLOAD_IDLE)) {
			/* Update the global length and block number */
			hdfu->wblock_num = req->value;
			hdfu->wlength = req->length;

			/* DFU GetPhase Command */
			if (hdfu->wblock_num == 0) {
				/* Update the state machine */
				hdfu->dev_state = (hdfu->wlength > 3) ?
						  DFU_STATE_IDLE :
						  DFU_STATE_UPLOAD_IDLE;

				hdfu->dev_status[1] = 0;
				hdfu->dev_status[2] = 0;
				hdfu->dev_status[3] = 0;
				hdfu->dev_status[4] = hdfu->dev_state;

				INFO("UPLOAD :\n");
				INFO("\t\tPhase ID : %i\n", usbd_dfu_phase_id);
				INFO("\t\taddress 0x%lx\n",
				     usbd_dfu_download_address);

				hdfu->buffer[0] = usbd_dfu_phase_id;
				hdfu->buffer[1] = (uint8_t)
						  (usbd_dfu_download_address);
				hdfu->buffer[2] = (uint8_t)
						  (usbd_dfu_download_address >>
						   8);
				hdfu->buffer[3] = (uint8_t)
						  (usbd_dfu_download_address >>
						   16);
				hdfu->buffer[4] = (uint8_t)
						  (usbd_dfu_download_address >>
						   24);

				hdfu->buffer[5] = 0x00;
				hdfu->buffer[6] = 0x00;
				hdfu->buffer[7] = 0x00;
				hdfu->buffer[8] = 0x00;

				if ((usbd_dfu_download_address ==
				    UNDEFINE_DOWN_ADDR) &&
				   (usbd_detach_req)) {
					INFO("Send detach request\n");
					hdfu->buffer[9] = 0x01;
					pdev->ep_in[0].total_length = 10;
					pdev->ep_in[0].rem_length   = 10;
				} else {
					pdev->ep_in[0].total_length = 9;
					pdev->ep_in[0].rem_length   = 9;
				}

				/* Send the status data over EP0 */
				pdev->ep0_state = USBD_EP0_DATA_IN;
				/* Start the transfer */
				usb_core_transmit(pdev, 0x00,
						  (uint8_t *)&hdfu->buffer[0],
						  pdev->ep_in[0].total_length);
			} else {
				/* unsupported hdfu->wblock_num */
				ERROR("UPLOAD : Unsupported block : %i\n",
				      hdfu->wblock_num);

				hdfu->dev_state = DFU_ERROR_STALLEDPKT;

				hdfu->dev_status[1] = 0;
				hdfu->dev_status[2] = 0;
				hdfu->dev_status[3] = 0;
				hdfu->dev_status[4] = hdfu->dev_state;

				/* Call the error management function
				 * (command will be nacked
				 */
				usb_core_ctl_error(pdev);
			}
		} else {
			/* Unsupported state */
			ERROR("UPLOAD : Unsupported State\n");

			hdfu->wlength = 0;
			hdfu->wblock_num = 0;
			/* Call the error management function
			 * (command will be nacked
			 */
			usb_core_ctl_error(pdev);
		}
	} else {
		/* No Data setup request */
		INFO("USB : DFU : Nothing to do\n");
		hdfu->dev_state = DFU_STATE_IDLE;

		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0;
		hdfu->dev_status[4] = hdfu->dev_state;
	}
}

/*
 * @brief  DFU_GetStatus
 *         Handles the DFU GETSTATUS request.
 * @param  pdev: instance
 * @retval status
 */
static void usb_dfu_get_status(usb_handle_t *pdev)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;
	uint16_t status;
	uint8_t dfu_bm_attribute = DFU_BM_ATTRIBUTE;

	switch (hdfu->dev_state) {
	case DFU_STATE_DNLOAD_SYNC:
		status = ((usb_dfu_media_t *)pdev->user_data)->get_status();

		switch (status) {
		case DFU_MEDIA_STATE_WRITTEN:
			/* SRAM block writing is finished, checks if checksum
			 * error has been detected
			 */
			hdfu->dev_state = DFU_STATE_DNLOAD_IDLE;
			break;

		case DFU_MEDIA_STATE_ERROR:
			hdfu->dev_state = DFU_STATE_ERROR;
			break;

		case DFU_MEDIA_STATE_READY:
		default:
			/* SRAM is ready to be written */
			hdfu->dev_state = DFU_STATE_DNLOAD_BUSY;
			break;
		}
		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0;
		hdfu->dev_status[4] = hdfu->dev_state;
		break;

	case DFU_STATE_MANIFEST_SYNC:
		if (hdfu->manif_state == DFU_MANIFEST_IN_PROGRESS) {
			hdfu->dev_state = DFU_STATE_MANIFEST;

			hdfu->dev_status[1] = 1;/*bwPollTimeout = 1ms*/
			hdfu->dev_status[2] = 0;
			hdfu->dev_status[3] = 0;
			hdfu->dev_status[4] = hdfu->dev_state;
		} else if ((hdfu->manif_state == DFU_MANIFEST_COMPLETE) &&
			   (dfu_bm_attribute & 0x04)) {
			INFO("USB : DFU : end of download partition : %i\n",
			     hdfu->alt_setting);
			hdfu->dev_state = DFU_STATE_IDLE;
			usbd_dfu_operation_complete = 1;

			hdfu->dev_status[1] = 0;
			hdfu->dev_status[2] = 0;
			hdfu->dev_status[3] = 0;
			hdfu->dev_status[4] = hdfu->dev_state;
		}
		break;

	default:
		break;
	}

	/* Send the status data over EP0 */
	pdev->ep0_state = USBD_EP0_DATA_IN;
	pdev->ep_in[0].total_length = 6;
	pdev->ep_in[0].rem_length = 6;
	/* Start the transfer */
	usb_core_transmit(pdev, 0x00, (uint8_t *)&hdfu->dev_status[0], 6);
}

/*
 * @brief  DFU_ClearStatus
 *         Handles the DFU CLRSTATUS request.
 * @param  pdev: device instance
 * @retval status
 */
static void usb_dfu_clear_status(usb_handle_t *pdev)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	if (hdfu->dev_state == DFU_STATE_ERROR) {
		hdfu->dev_state = DFU_STATE_IDLE;
		hdfu->dev_status[0] = DFU_ERROR_NONE;/*bStatus*/
		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0; /*bwPollTimeout=0ms*/
		hdfu->dev_status[4] = hdfu->dev_state;/*bState*/
		hdfu->dev_status[5] = 0;/*iString*/
	} else {
		/*State Error*/
		hdfu->dev_state = DFU_STATE_ERROR;
		hdfu->dev_status[0] = DFU_ERROR_UNKNOWN;/*bStatus*/
		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0; /*bwPollTimeout=0ms*/
		hdfu->dev_status[4] = hdfu->dev_state;/*bState*/
		hdfu->dev_status[5] = 0;/*iString*/
	}
}

/*
 * @brief  DFU_GetState
 *         Handles the DFU GETSTATE request.
 * @param  pdev: device instance
 * @retval None
 */
static void usb_dfu_get_state(usb_handle_t *pdev)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	/* Return the current state of the DFU interface */
	/* Send the status data over EP0 */
	pdev->ep0_state = USBD_EP0_DATA_IN;
	pdev->ep_in[0].total_length = 1;
	pdev->ep_in[0].rem_length = 1;

	/* Start the transfer */
	usb_core_transmit(pdev, 0x00, &hdfu->dev_state, 1);
}

/*
 * @brief  DFU_Abort
 *         Handles the DFU ABORT request.
 * @param  pdev: device instance
 * @retval None
 */
static void usb_dfu_abort(usb_handle_t *pdev)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	if (hdfu->dev_state == DFU_STATE_IDLE ||
	    hdfu->dev_state == DFU_STATE_DNLOAD_SYNC ||
	    hdfu->dev_state == DFU_STATE_DNLOAD_IDLE ||
	    hdfu->dev_state == DFU_STATE_MANIFEST_SYNC ||
	    hdfu->dev_state == DFU_STATE_UPLOAD_IDLE) {
		hdfu->dev_state = DFU_STATE_IDLE;
		hdfu->dev_status[0] = DFU_ERROR_NONE;
		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0; /*bwPollTimeout=0ms*/
		hdfu->dev_status[4] = hdfu->dev_state;
		hdfu->dev_status[5] = 0; /*iString*/
		hdfu->wblock_num = 0;
		hdfu->wlength = 0;
	}
}

/*
 * @brief  USBD_DFU_Setup
 *         Handle the DFU specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t usb_dfu_setup(usb_handle_t *pdev, usb_setup_req_t *req)
{
	uint8_t *pbuf = NULL;
	uint16_t len = 0;
	uint8_t ret = USBD_OK;
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	VERBOSE("alt_setting %i, bmRequest : 0x%x, brequest : 0x%x\n",
		hdfu->alt_setting, req->bm_request & USB_REQ_TYPE_MASK,
		req->b_request);
	switch (req->bm_request & USB_REQ_TYPE_MASK) {
	case USB_REQ_TYPE_CLASS:
		usbd_dfu_current_req = req->b_request;
		if (hdfu->alt_setting == usbd_dfu_phase_id) {
			switch (req->b_request) {
			case DFU_DNLOAD:
				usb_dfu_download(pdev, req);
				break;

			case DFU_UPLOAD:
				usb_dfu_upload(pdev, req);
				break;

			case DFU_GETSTATUS:
				usb_dfu_get_status(pdev);
				break;

			case DFU_CLRSTATUS:
				usb_dfu_clear_status(pdev);
				break;

			case DFU_GETSTATE:
				usb_dfu_get_state(pdev);
				break;

			case DFU_ABORT:
				usb_dfu_abort(pdev);
				break;

			case DFU_DETACH:
				usb_dfu_detach(pdev, req);
				break;

			default:
				ERROR("phase ID :%i\n", usbd_dfu_phase_id);
				usb_core_ctl_error(pdev);
				ret = USBD_FAIL;
				break;
			}
		} else if (hdfu->alt_setting == DFU_GET_PHASE) {
			switch (req->b_request) {
			case DFU_UPLOAD:
				usb_dfu_upload(pdev, req);
				break;

			case DFU_GETSTATUS:
				INFO("GETSTATUS :\n");
				usb_dfu_get_status(pdev);

				switch (hdfu->dev_state) {
				case APP_STATE_IDLE:
					INFO("\t\tAPP_STATE_IDLE\n");
					break;
				case APP_STATE_DETACH:
					INFO("\t\tAPP_STATE_DETACH\n");
					break;
				case DFU_STATE_IDLE:
					INFO("\t\tDFU_STATE_IDLE\n");
					break;
				case DFU_STATE_DNLOAD_SYNC:
					INFO("\t\tDFU_STATE_DNLOAD_SYNC\n");
					break;
				case DFU_STATE_DNLOAD_BUSY:
					INFO("\t\tDFU_STATE_DNLOAD_BUSY\n");
					break;
				case DFU_STATE_DNLOAD_IDLE:
					INFO("\t\tDFU_STATE_DNLOAD_IDLE\n");
					break;
				case DFU_STATE_MANIFEST_SYNC:
					INFO("\t\tDFU_STATE_MANIFEST_SYNC\n");
					break;
				case DFU_STATE_MANIFEST:
					INFO("\t\tDFU_STATE_MANIFEST\n");
					break;
				case DFU_STATE_MANIFEST_WAIT_RESET:
					INFO("\t\tDFU_STATE_MANIFEST_WAIT_RESET\n");
					break;
				case DFU_STATE_UPLOAD_IDLE:
					INFO("\t\tDFU_STATE_UPLOAD_IDLE\n");
					break;
				case DFU_STATE_ERROR:
					ERROR("\t\tDFU_STATE_ERROR\n");
					break;
				default:
					break;
				}
				break;

			case DFU_CLRSTATUS:
				INFO("Receive DFU clear status\n");
				usb_dfu_clear_status(pdev);
				break;

			case DFU_GETSTATE:
				INFO("GETSTATE :\n");
				usb_dfu_get_state(pdev);

				switch (hdfu->dev_state) {
				case APP_STATE_IDLE:
					INFO("\t\tAPP_STATE_IDLE\n");
					break;
				case APP_STATE_DETACH:
					INFO("\t\tAPP_STATE_DETACH\n");
					break;
				case DFU_STATE_IDLE:
					INFO("\t\tDFU_STATE_IDLE\n");
					break;
				case DFU_STATE_DNLOAD_SYNC:
					INFO("\t\tDFU_STATE_DNLOAD_SYNC\n");
					break;
				case DFU_STATE_DNLOAD_BUSY:
					INFO("\t\tDFU_STATE_DNLOAD_BUSY\n");
					break;
				case DFU_STATE_DNLOAD_IDLE:
					INFO("\t\tDFU_STATE_DNLOAD_IDLE\n");
					break;
				case DFU_STATE_MANIFEST_SYNC:
					INFO("\t\tDFU_STATE_MANIFEST_SYNC\n");
					break;
				case DFU_STATE_MANIFEST:
					INFO("\t\tDFU_STATE_MANIFEST\n");
					break;
				case DFU_STATE_MANIFEST_WAIT_RESET:
					INFO("\t\tDFU_STATE_MANIFEST_WAIT_RESET\n");
					break;
				case DFU_STATE_UPLOAD_IDLE:
					INFO("\t\tDFU_STATE_UPLOAD_IDLE\n");
					break;
				case DFU_STATE_ERROR:
					ERROR("\t\tDFU_STATE_ERROR\n");
					break;
				default:
					break;
				}
				break;

			case DFU_ABORT:
				INFO("Receive DFU abort\n");
				usb_dfu_abort(pdev);
				break;

			case DFU_DETACH:
				usb_dfu_detach(pdev, req);
				break;

			default:
				ERROR("phase ID :%i\n", DFU_GET_PHASE);
				usb_core_ctl_error(pdev);
				ret = USBD_FAIL;
				break;
			}
		} else {
			ERROR("Unknown alternate : %i\n", hdfu->alt_setting);
			ret = USBD_FAIL;
		}
		break;
	case USB_REQ_TYPE_STANDARD:
		switch (req->b_request) {
		case USB_REQ_GET_DESCRIPTOR:
			if ((req->value >> 8) == DFU_DESCRIPTOR_TYPE) {
				pbuf = pdev->desc->get_dfu_desc(&len);
				len = MIN(len, req->length);
			}

			pdev->ep0_state = USBD_EP0_DATA_IN;
			pdev->ep_in[0].total_length = len;
			pdev->ep_in[0].rem_length = len;
			/* Start the transfer */
			usb_core_transmit(pdev, 0x00, pbuf, len);

			break;

		case USB_REQ_GET_INTERFACE:
			pdev->ep0_state = USBD_EP0_DATA_IN;
			pdev->ep_in[0].total_length = 1;
			pdev->ep_in[0].rem_length   = 1;
			/* Start the transfer */
			usb_core_transmit(pdev, 0x00,
					  (uint8_t *)&hdfu->alt_setting, 1);
			break;

		case USB_REQ_SET_INTERFACE:
			hdfu->alt_setting = (uint8_t)(req->value);
			break;

		default:
			usb_core_ctl_error(pdev);
			ret = USBD_FAIL;
			break;
		}
	default:
		break;
	}

	return ret;
}

static const usb_class_t  USBD_DFU_initvalue = {
	usb_dfu_init,
	usb_dfu_de_init,
	usb_dfu_setup,
	usb_dfu_ep0_tx_ready,
	usb_dfu_ep0_rx_ready,
	usb_dfu_data_in,
	usb_dfu_data_out,
	usb_dfu_sof,
	usb_dfu_iso_in_incomplete,
	usb_dfu_iso_out_incomplete,
	0
};

void usb_dfu_register_callback(usb_handle_t *pdev)
{
	pdev->class = (usb_class_t *)&USBD_DFU_initvalue;
}

void usb_dfu_set_phase_id(uint32_t phase_id)
{
	usbd_dfu_phase_id = phase_id;
	usbd_dfu_operation_complete = 0;
}

void usb_dfu_set_download_addr(uintptr_t addr)
{
	usbd_dfu_download_address = addr;
}

uint32_t usb_dfu_download_is_completed(void)
{
	return usbd_dfu_operation_complete;
}

uint32_t usb_dfu_get_current_req(void)
{
	return usbd_dfu_current_req;
}

uint32_t usb_dfu_detach_req(void)
{
	return usbd_detach_req;
}

void usb_dfu_request_detach(void)
{
	usbd_detach_req = 1;
}
