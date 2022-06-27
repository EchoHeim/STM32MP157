/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_DWC2_H
#define __USB_DWC2_H

#include <lib/usb/usb_core.h>

/* define value use in register */

#define USB_OTG_MODE_DEVICE			0
#define USB_OTG_MODE_HOST			1
#define USB_OTG_MODE_DRD			2

#define DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ	(0 << 1)
#define DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ	(1 << 1)
#define DSTS_ENUMSPD_LS_PHY_6MHZ		(2 << 1)
#define DSTS_ENUMSPD_FS_PHY_48MHZ		(3 << 1)

#define EP_TYPE_CTRL				0
#define EP_TYPE_ISOC				1
#define EP_TYPE_BULK				2
#define EP_TYPE_INTR				3

#define STS_GOUT_NAK				1
#define STS_DATA_UPDT				2
#define STS_XFER_COMP				3
#define STS_SETUP_COMP				4
#define STS_SETUP_UPDT				6

#define USBD_HS_TRDT_VALUE			9

#define USB_OTG_DEVICE_BASE			((uint32_t)0x800)
#define USB_OTG_IN_ENDPOINT_BASE		((uint32_t)0x900)
#define USB_OTG_OUT_ENDPOINT_BASE		((uint32_t)0xB00)
#define USB_OTG_FIFO_BASE			((uint32_t)0x1000)
#define USB_OTG_FIFO_SIZE			((uint32_t)0x1000)
#define USB_MAX_ENDPOINT_NB			0x10

/* Bit definition for register */
/* USB_OTG_GRSTCTL register */
/* Core soft reset */
#define USB_OTG_GRSTCTL_CSRST			((uint32_t)0x00000001)
/* HCLK soft reset */
#define USB_OTG_GRSTCTL_HSRST			((uint32_t)0x00000002)
/* Host frame counter reset */
#define USB_OTG_GRSTCTL_FCRST			((uint32_t)0x00000004)
/* RxFIFOflush */
#define USB_OTG_GRSTCTL_RXFFLSH			((uint32_t)0x00000010)
/* TxFIFOflush */
#define USB_OTG_GRSTCTL_TXFFLSH			((uint32_t)0x00000020)

/* USB_OTG_DIEPCTLregister */
/* Maximum packet size */
#define USB_OTG_DIEPCTL_MPSIZ			((uint32_t)0x000007FF)
/* USB active endpoint */
#define USB_OTG_DIEPCTL_USBAEP			((uint32_t)0x00008000)
/* Even/odd frame */
#define USB_OTG_DIEPCTL_EONUM_DPID		((uint32_t)0x00010000)
/* NAK status */
#define USB_OTG_DIEPCTL_NAKSTS			((uint32_t)0x00020000)
/* Endpoint type */
#define USB_OTG_DIEPCTL_EPTYP			((uint32_t)0x000C0000)
/* Bit0 */
#define USB_OTG_DIEPCTL_EPTYP_0			((uint32_t)0x00040000)
/* Bit1 */
#define USB_OTG_DIEPCTL_EPTYP_1			((uint32_t)0x00080000)
/* STALL handshake */
#define USB_OTG_DIEPCTL_STALL			((uint32_t)0x00200000)
/* TxFIFO number */
#define USB_OTG_DIEPCTL_TXFNUM			((uint32_t)0x03C00000)
/* Bit0 */
#define USB_OTG_DIEPCTL_TXFNUM_0		((uint32_t)0x00400000)
/* Bit1 */
#define USB_OTG_DIEPCTL_TXFNUM_1		((uint32_t)0x00800000)
/* Bit2 */
#define USB_OTG_DIEPCTL_TXFNUM_2		((uint32_t)0x01000000)
/* Bit3 */
#define USB_OTG_DIEPCTL_TXFNUM_3		((uint32_t)0x02000000)
/* Clear NAK */
#define USB_OTG_DIEPCTL_CNAK			((uint32_t)0x04000000)
/* Set NAK */
#define USB_OTG_DIEPCTL_SNAK			((uint32_t)0x08000000)
/* Set DATA0 PID */
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM		((uint32_t)0x10000000)
/* Set odd frame */
#define USB_OTG_DIEPCTL_SODDFRM			((uint32_t)0x20000000)
/* Endpoint disable */
#define USB_OTG_DIEPCTL_EPDIS			((uint32_t)0x40000000)
/* Endpoint enable */
#define USB_OTG_DIEPCTL_EPENA			((uint32_t)0x80000000)

/* USB_OTG_DOEPCTL register */
/* Maximum packet size */
#define USB_OTG_DOEPCTL_MPSIZ			((uint32_t)0x000007FF)
/* USB active endpoint */
#define USB_OTG_DOEPCTL_USBAEP			((uint32_t)0x00008000)
/* NAK status */
#define USB_OTG_DOEPCTL_NAKSTS			((uint32_t)0x00020000)
/* Set DATA0 PID */
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM		((uint32_t)0x10000000)
/* Set odd frame */
#define USB_OTG_DOEPCTL_SODDFRM			((uint32_t)0x20000000)
/* Endpoint type */
#define USB_OTG_DOEPCTL_EPTYP			((uint32_t)0x000C0000)
/* Bit0 */
#define USB_OTG_DOEPCTL_EPTYP_0			((uint32_t)0x00040000)
/* Bit1 */
#define USB_OTG_DOEPCTL_EPTYP_1			((uint32_t)0x00080000)
/* Snoop mode */
#define USB_OTG_DOEPCTL_SNPM			((uint32_t)0x00100000)
/* STALL handshake */
#define USB_OTG_DOEPCTL_STALL			((uint32_t)0x00200000)
/* Clear NAK */
#define USB_OTG_DOEPCTL_CNAK			((uint32_t)0x04000000)
/* Set NAK */
#define USB_OTG_DOEPCTL_SNAK			((uint32_t)0x08000000)
/* Endpoint disable */
#define USB_OTG_DOEPCTL_EPDIS			((uint32_t)0x40000000)
/* Endpoint enable */
#define USB_OTG_DOEPCTL_EPENA			((uint32_t)0x80000000)

/* USB_OTG_DSTSregister */
/* Suspend status */
#define USB_OTG_DSTS_SUSPSTS			((uint32_t)0x00000001)
/* Enumerated speed */
#define USB_OTG_DSTS_ENUMSPD			((uint32_t)0x00000006)
/* Bit0 */
#define USB_OTG_DSTS_ENUMSPD_0			((uint32_t)0x00000002)
/* Bit1 */
#define USB_OTG_DSTS_ENUMSPD_1			((uint32_t)0x00000004)
/* Erratic error */
#define USB_OTG_DSTS_EERR			((uint32_t)0x00000008)
/* Frame number of the received SOF */
#define USB_OTG_DSTS_FNSOF			((uint32_t)0x003FFF00)

/* USB_OTG_DCTLregister */
/* Remote wakeup signaling */
#define USB_OTG_DCTL_RWUSIG			((uint32_t)0x00000001)
/* Soft disconnect */
#define USB_OTG_DCTL_SDIS			((uint32_t)0x00000002)
/* Global IN NAK status */
#define USB_OTG_DCTL_GINSTS			((uint32_t)0x00000004)
/* Global OUT NAK status */
#define USB_OTG_DCTL_GONSTS			((uint32_t)0x00000008)
/* Test control */
#define USB_OTG_DCTL_TCTL			((uint32_t)0x00000070)
/* Bit0 */
#define USB_OTG_DCTL_TCTL_0			((uint32_t)0x00000010)
/* Bit1 */
#define USB_OTG_DCTL_TCTL_1			((uint32_t)0x00000020)
/* Bit2 */
#define USB_OTG_DCTL_TCTL_2			((uint32_t)0x00000040)
/* Set global IN NAK */
#define USB_OTG_DCTL_SGINAK			((uint32_t)0x00000080)
/* Clear global IN NAK */
#define USB_OTG_DCTL_CGINAK			((uint32_t)0x00000100)
/* Set global OUT NAK */
#define USB_OTG_DCTL_SGONAK			((uint32_t)0x00000200)
/* Clear global OUT NAK */
#define USB_OTG_DCTL_CGONAK			((uint32_t)0x00000400)
/* Power-on programming done */
#define USB_OTG_DCTL_POPRGDNE			((uint32_t)0x00000800)

/* USB_OTG_GAHBCFG register */
/* Global interrupt mask */
#define USB_OTG_GAHBCFG_GINT			((uint32_t)0x00000001)

/* USB_OTG_DOEPTSIZr egister */
/* Transfer size */
#define USB_OTG_DOEPTSIZ_XFRSIZ			((uint32_t)0x0007FFFF)
/* Packet count */
#define USB_OTG_DOEPTSIZ_PKTCNT			((uint32_t)0x1FF80000)
/* SETUP packet count */
#define USB_OTG_DOEPTSIZ_STUPCNT		((uint32_t)0x60000000)
/* Bit0 */
#define USB_OTG_DOEPTSIZ_STUPCNT_0		((uint32_t)0x20000000)
/* Bit1 */
#define USB_OTG_DOEPTSIZ_STUPCNT_1		((uint32_t)0x40000000)

/* USB_OTG_DIEPTSIZ register */
/* Transfer size */
#define USB_OTG_DIEPTSIZ_XFRSIZ			((uint32_t)0x0007FFFF)
/* Packet count */
#define USB_OTG_DIEPTSIZ_PKTCNT			((uint32_t)0x1FF80000)
/* Packet count */
#define USB_OTG_DIEPTSIZ_MULCNT			((uint32_t)0x60000000)

/* USB_OTG_DCFG register */
/* Device address */
#define USB_OTG_DCFG_DAD			((uint32_t)0x000007F0)

/* USB_OTG_DTXFSTS register */
/* IN endpoint Tx FIFO space available */
#define USB_OTG_DTXFSTS_INEPTFSAV		((uint32_t)0x0000FFFF)

/* USB_OTG_GINTSTS register */
/* Current mode of operation */
#define USB_OTG_GINTSTS_CMOD			((uint32_t)0x00000001)
/* Modem is match interrupt */
#define USB_OTG_GINTSTS_MMIS			((uint32_t)0x00000002)
/* OTG interrupt */
#define USB_OTG_GINTSTS_OTGINT			((uint32_t)0x00000004)
/* Start offrame */
#define USB_OTG_GINTSTS_SOF			((uint32_t)0x00000008)
/* Rx FIFO nonempty */
#define USB_OTG_GINTSTS_RXFLVL			((uint32_t)0x00000010)
/* Non periodic Tx FIFO empty */
#define USB_OTG_GINTSTS_NPTXFE			((uint32_t)0x00000020)
/* Global IN non periodic NAK effective */
#define USB_OTG_GINTSTS_GINAKEFF		((uint32_t)0x00000040)
/* Global OUT NAK effective */
#define USB_OTG_GINTSTS_BOUTNAKEFF		((uint32_t)0x00000080)
/* Early suspend */
#define USB_OTG_GINTSTS_ESUSP			((uint32_t)0x00000400)
/* USB suspend */
#define USB_OTG_GINTSTS_USBSUSP			((uint32_t)0x00000800)
/* USB reset */
#define USB_OTG_GINTSTS_USBRST			((uint32_t)0x00001000)
/* Enumeration done */
#define USB_OTG_GINTSTS_ENUMDNE			((uint32_t)0x00002000)
/* Isochronous OUT packet dropped interrupt */
#define USB_OTG_GINTSTS_ISOODRP			((uint32_t)0x00004000)
/* End of periodic frame interrupt */
#define USB_OTG_GINTSTS_EOPF			((uint32_t)0x00008000)
/* IN endpoint interrupt */
#define USB_OTG_GINTSTS_IEPINT			((uint32_t)0x00040000)
/* OUT endpoint interrupt */
#define USB_OTG_GINTSTS_OEPINT			((uint32_t)0x00080000)
/* Incomplete isochronous IN transfer */
#define USB_OTG_GINTSTS_IISOIXFR		((uint32_t)0x00100000)
/* Incomplete periodic transfer */
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT	((uint32_t)0x00200000)
/* Data fetch suspended */
#define USB_OTG_GINTSTS_DATAFSUSP		((uint32_t)0x00400000)
/* Reset detected interrupt */
#define USB_OTG_GINTSTS_RSTDET			((uint32_t)0x00800000)
/* Host port interrupt */
#define USB_OTG_GINTSTS_HPRTINT			((uint32_t)0x01000000)
/* Host channels interrupt */
#define USB_OTG_GINTSTS_HCINT			((uint32_t)0x02000000)
/* Periodic Tx FIFO empty */
#define USB_OTG_GINTSTS_PTXFE			((uint32_t)0x04000000)
/* LPM interrupt */
#define USB_OTG_GINTSTS_LPMINT			((uint32_t)0x08000000)
/* Connector ID status change */
#define USB_OTG_GINTSTS_CIDSCHG			((uint32_t)0x10000000)
/* Disconnect detected interrupt */
#define USB_OTG_GINTSTS_DISCINT			((uint32_t)0x20000000)
/* Session request/new session detected interrupt */
#define USB_OTG_GINTSTS_SRQINT			((uint32_t)0x40000000)
/* Resume/remote wakeup detected interrupt */
#define USB_OTG_GINTSTS_WKUINT			((uint32_t)0x80000000)

/* USB_OTG_DOEPINT register */
/* Transfer completed interrupt */
#define USB_OTG_DOEPINT_XFRC			((uint32_t)0x00000001)
/* Endpoint disabled interrupt */
#define USB_OTG_DOEPINT_EPDISD			((uint32_t)0x00000002)
/* SETUP phase done */
#define USB_OTG_DOEPINT_STUP			((uint32_t)0x00000008)
/* OUT token received when endpoint disabled */
#define USB_OTG_DOEPINT_OTEPDIS			((uint32_t)0x00000010)
/* Back-to-back SETUP packets received */
#define USB_OTG_DOEPINT_B2BSTUP			((uint32_t)0x00000040)
/* NYET interrupt */
#define USB_OTG_DOEPINT_NYET			((uint32_t)0x00004000)

/* USB_OTG_DIEPINTregister */
/* Transfer completed interrupt */
#define USB_OTG_DIEPINT_XFRC			((uint32_t)0x00000001)
/* Endpoint disabled interrupt */
#define USB_OTG_DIEPINT_EPDISD			((uint32_t)0x00000002)
/* Timeout condition */
#define USB_OTG_DIEPINT_TOC			((uint32_t)0x00000008)
/* IN token received when Tx FIFO is empty */
#define USB_OTG_DIEPINT_ITTXFE			((uint32_t)0x00000010)
/* IN endpoint NAK effective */
#define USB_OTG_DIEPINT_INEPNE			((uint32_t)0x00000040)
/* Transmit Fifo empty */
#define USB_OTG_DIEPINT_TXFE			((uint32_t)0x00000080)
/* Transmit Fifo Underrun */
#define USB_OTG_DIEPINT_TXFIFOUDRN		((uint32_t)0x00000100)
/* Buffer not available interrupt */
#define USB_OTG_DIEPINT_BNA			((uint32_t)0x00000200)
/* Packet dropped status */
#define USB_OTG_DIEPINT_PKTDRPSTS		((uint32_t)0x00000800)
/* Babble error interrupt */
#define USB_OTG_DIEPINT_BERR			((uint32_t)0x00001000)
/* NAK interrupt */
#define USB_OTG_DIEPINT_NAK			((uint32_t)0x00002000)

/* USB_OTG_GLPMCFG register */
/* BESL value received with last ACKed LPM Token */
#define USB_OTG_GLPMCFG_BESL			((uint32_t)0x0000003C)

/* USB_OTG_GOTGINT register */
/* Session end detected */
#define USB_OTG_GOTGINT_SEDET			((uint32_t)0x00000004)

/* USB_OTG_GRXSTSP register */
/* IN EP interrupt mask bits */
#define USB_OTG_GRXSTSP_EPNUM			((uint32_t)0x0000000F)
/* OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_BCNT			((uint32_t)0x00007FF0)
/* OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_DPID			((uint32_t)0x00018000)
/* OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_PKTSTS			((uint32_t)0x001E0000)

/* USB_OTG_GUSBCFG register */
/* USB turn around time */
#define USB_OTG_GUSBCFG_TRDT			((uint32_t)0x00003C00)

/* USB_OTG_DOEPMSK register */
/* Transfer completed interrupt mask */
#define USB_OTG_DOEPMSK_XFRCM			((uint32_t)0x00000001)
/* Endpoint disabled interrupt mask */
#define USB_OTG_DOEPMSK_EPDM			((uint32_t)0x00000002)
/* SETUP phase done mask */
#define USB_OTG_DOEPMSK_STUPM			((uint32_t)0x00000008)
/* OUT token received when endpoint disabled mask */
#define USB_OTG_DOEPMSK_OTEPDM			((uint32_t)0x00000010)
/* Back-to-back SETUP packets received mask */
#define USB_OTG_DOEPMSK_B2BSTUP			((uint32_t)0x00000040)
/* OUT packet error mask */
#define USB_OTG_DOEPMSK_OPEM			((uint32_t)0x00000100)
/* BNA interrupt mask */
#define USB_OTG_DOEPMSK_BOIM			((uint32_t)0x00000200)

/* USB_OTG_DIEPMSK register */
/* Transfer completed interrupt mask */
#define USB_OTG_DIEPMSK_XFRCM			((uint32_t)0x00000001)
/* Endpoint disabled interrupt mask */
#define USB_OTG_DIEPMSK_EPDM			((uint32_t)0x00000002)
/* Timeout condition mask(non isochronous endpoints) */
#define USB_OTG_DIEPMSK_TOM			((uint32_t)0x00000008)
/* IN token received when Tx FIFO empty mask */
#define USB_OTG_DIEPMSK_ITTXFEMSK		((uint32_t)0x00000010)
/* IN token received with EP mismatch mask */
#define USB_OTG_DIEPMSK_INEPNMM			((uint32_t)0x00000020)
/* IN endpoint NAK effective mask */
#define USB_OTG_DIEPMSK_INEPNEM			((uint32_t)0x00000040)
/* FIFO under run mask */
#define USB_OTG_DIEPMSK_TXFURM			((uint32_t)0x00000100)
/* BNA interrupt mask */
#define USB_OTG_DIEPMSK_BIM			((uint32_t)0x00000200)

typedef struct {
	uint32_t dcfg;/* dev Configuration Register */
	uint32_t dctl;/* dev Control Register */
	uint32_t dsts;/* dev Status Register(RO) */
	uint32_t reserved1;/* reserved */
	uint32_t diepmsk;/* dev IN Endpoint Mask */
	uint32_t doepmsk;/* dev OUT Endpoint Mask */
	uint32_t daint;/* dev All Endpoints Itr Reg */
	uint32_t daintmsk;/* dev All Endpoints Itr Mask */
	uint32_t reserved2;/* reserved */
	uint32_t reserved3;/* reserved */
	uint32_t dvbusdis;/* dev VBUS discharge Register */
	uint32_t dvbuspulse;/* dev VBUS Pulse Register */
	uint32_t dthrctl;/* dev threshold */
	uint32_t diepempmsk;/* dev empty msk */
	uint32_t deachint;/* dedicated EP interrupt */
	uint32_t deachmsk;/* dedicated EP msk */
	uint32_t reserved4;/* dedicated EP mask */
	uint32_t dinep1msk;/* dedicated EP mask */
	uint32_t reserved5[15];/* reserved */
	uint32_t doutep1msk;/* dedicated EP msk */
} usb_dwc2_device_t;

typedef struct {
	uint32_t epctl;/* dev IN Endpoint Control Reg */
	uint32_t reserved1;/* reserved */
	uint32_t epint;/* dev IN Endpoint Itr Reg */
	uint32_t reserved2;/* reserved*/
	uint32_t eptsiz;/* IN Endpoint Txfer Size */
	uint32_t epdma;/* IN Endpoint DMA Address Reg */
	uint32_t txfsts;/* IN Endpoint Tx FIFO Status Reg */
	uint32_t reserved3;/* reserved */
} usb_dwc2_endpoint_t;

typedef struct {
	uint32_t gotgctl;/* USB_OTG Control and Status Register */
	uint32_t gotgint;/* USB_OTG Interrupt Register */
	uint32_t gahbcfg;/* Core AHB Configuration Register */
	uint32_t gusbcfg;/* Core USB Configuration Register */
	uint32_t grstctl;/* Core Reset Register */
	uint32_t gintsts;/* Core Interrupt Register */
	uint32_t gintmsk;/* Core Interrupt Mask Register */
	uint32_t grxstsr;/* Receive StsQ Read Register */
	uint32_t grxstsp;/* Receive StsQ Read & POP Register */
	uint32_t grxfsiz;/* Receive FIFO SizeRegister */
	uint32_t dieptxfo;/* EP0/Non Periodic Tx FIFO Size Reg */
	uint32_t hnptxsts;/* Non Periodic Tx FIFO / Queue Sts reg */
	uint32_t reserved1[2];/* reserved */
	uint32_t gccfg;/* General Purpose IO Register */
	uint32_t cid;/* User ID Register */
	uint32_t reserved2[3];/* reserved */
	uint32_t ghwcfg3;/* User HW config */
	uint32_t reserved3;/* reserved */
	uint32_t glpmcfg;/* LPM Register */
	uint32_t gpwrdn;/* Power Down Register */
	uint32_t gdfifocfg;/* DFIFO Software Config Register */
	uint32_t gadpctl;/* ADPTimer, Control and Status Register */
	uint32_t reserved4[39];/* reserved */
	uint32_t hptxfsiz;/* Host Periodic Tx FIFO Size Reg */
	uint32_t dieptxf[0x0F];/* dev Periodic Transmit FIFO */
} usb_dwc2_global_t;

typedef struct {
	usb_dwc2_global_t *usb_global;
	usb_dwc2_device_t *usb_device;
	usb_dwc2_endpoint_t *usb_in_endpoint[USB_MAX_ENDPOINT_NB];
	usb_dwc2_endpoint_t *usb_out_endpoint[USB_MAX_ENDPOINT_NB];
	uint32_t *usb_fifo[USB_MAX_ENDPOINT_NB];
} usb_dwc2_t;

usb_status_t usb_dwc2_disable_int(void *handle);
usb_status_t usb_dwc2_ep0_out_start(void *handle);
usb_status_t usb_dwc2_ep_start_xfer(void *handle, usb_otg_ep_t *ep);
usb_status_t usb_dwc2_ep0_start_xfer(void *handle, usb_otg_ep_t *ep);
usb_status_t usb_dwc2_write_packet(void *handle, uint8_t *src,
				   uint8_t ch_ep_num, uint16_t len);
void *usb_dwc2_read_packet(void *handle, uint8_t *dest, uint16_t len);
usb_status_t usb_dwc2_ep_set_stall(void *handle, usb_otg_ep_t *ep);
usb_status_t usb_dwc2_stop_device(void *handle);
usb_status_t usb_dwc2_set_address(void *handle, uint8_t address);
usb_status_t usb_dwc2_dev_disconnect(void *handle);
usb_status_t usb_dwc2_write_empty_tx_fifo(void *handle,	uint32_t epnum,
					  uint32_t xfer_len,
					  uint32_t *xfer_count,
					  uint32_t maxpacket,
					  uint8_t **xfer_buff);
usb_action_t usb_dwc2_it_handler(void *handle, uint32_t *param);
void usb_dwc2_init_driver(usb_handle_t *usb_core_handle,
			  uint32_t *base_register);

#endif /* __USB_DWC2_H */

