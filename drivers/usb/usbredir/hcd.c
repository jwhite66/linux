/*
 * Copyright (C) 2015 Jeremy White based on vhci_hcd which is
 * Copyright (C) 2003-2008 Takahiro Hirofuchi
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307,
 * USA.
 */

#include <linux/init.h>
#include <linux/file.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/net.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "usbredir.h"

#define DRIVER_AUTHOR "Jeremy White"
#define DRIVER_DESC "USBREDIR Host Controller Driver"
#define DRIVER_VERSION USBREDIR_MODULE_VERSION

static const char driver_name[] = "usbredir";
static const char driver_desc[] = DRIVER_DESC;

struct usbredir_hcd *the_controller;

static const char * const bit_desc[] = {
	"CONNECTION",		/*0*/
	"ENABLE",		/*1*/
	"SUSPEND",		/*2*/
	"OVER_CURRENT",		/*3*/
	"RESET",		/*4*/
	"R5",			/*5*/
	"R6",			/*6*/
	"R7",			/*7*/
	"POWER",		/*8*/
	"LOWSPEED",		/*9*/
	"HIGHSPEED",		/*10*/
	"PORT_TEST",		/*11*/
	"INDICATOR",		/*12*/
	"R13",			/*13*/
	"R14",			/*14*/
	"R15",			/*15*/
	"C_CONNECTION",		/*16*/
	"C_ENABLE",		/*17*/
	"C_SUSPEND",		/*18*/
	"C_OVER_CURRENT",	/*19*/
	"C_RESET",		/*20*/
	"R21",			/*21*/
	"R22",			/*22*/
	"R23",			/*23*/
	"R24",			/*24*/
	"R25",			/*25*/
	"R26",			/*26*/
	"R27",			/*27*/
	"R28",			/*28*/
	"R29",			/*29*/
	"R30",			/*30*/
	"R31",			/*31*/
};

static void dump_port_status_diff(u32 prev_status, u32 new_status)
{
	int i = 0;
	u32 bit = 1;

	pr_debug("status prev -> new: %08x -> %08x\n", prev_status, new_status);
	while (bit) {
		u32 prev = prev_status & bit;
		u32 new = new_status & bit;
		char change;

		if (!prev && new)
			change = '+';
		else if (prev && !new)
			change = '-';
		else
			change = ' ';

		if (prev || new)
			pr_debug(" %c%s\n", change, bit_desc[i]);
		bit <<= 1;
		i++;
	}
	pr_debug("\n");
}


static u32 speed_to_portflag(enum usb_device_speed speed)
{
	switch(speed) {
	case usb_redir_speed_low:   return USB_PORT_STAT_LOW_SPEED;
	case usb_redir_speed_high:  return USB_PORT_STAT_HIGH_SPEED;

	case usb_redir_speed_full:
	case usb_redir_speed_super:
	default:		    return 0;
	}
}

// TODO - no thought at all to Super speed stuff...
void hcd_connect_port(struct usbredir_device *vdev)
{
	u32 *port = vdev->uhcd->port_status + vdev->rhport;

	pr_debug("hcd_connect_port %s\n", vdev->devid);
	spin_lock(&vdev->uhcd->lock);

	*port |= USB_PORT_STAT_CONNECTION | (1 << USB_PORT_FEAT_C_CONNECTION);
	*port |= speed_to_portflag(vdev->connect_header.speed);

	spin_unlock(&vdev->uhcd->lock);

	usb_hcd_poll_rh_status(usbredir_to_hcd(vdev->uhcd));
}

struct usbredir_device *find_devid(struct usbredir_hcd *uhcd, const char *devid)
{
	int i;
	struct usbredir_device *vdev = NULL;

	spin_lock(&uhcd->lock);
	for (i = 0; i < USBREDIR_NPORTS; i++) {
		vdev = port_to_vdev(uhcd, i);
		spin_lock(&vdev->lock);
		if (vdev->devid && strcmp(vdev->devid, devid) == 0) {
			spin_unlock(&vdev->lock);
			break;
		}
		spin_unlock(&vdev->lock);
	}
	spin_unlock(&uhcd->lock);

	if (i >= USBREDIR_NPORTS)
		return NULL;

	return vdev;
}

static void rh_port_disconnect(struct usbredir_device *vdev)
{
	pr_debug("rh_port_disconnect %s\n", vdev->devid);

	spin_lock(&vdev->uhcd->lock);

	vdev->uhcd->port_status[vdev->rhport] &= ~USB_PORT_STAT_CONNECTION;
	vdev->uhcd->port_status[vdev->rhport] |=
					(1 << USB_PORT_FEAT_C_CONNECTION);

	spin_unlock(&vdev->uhcd->lock);
	usb_hcd_poll_rh_status(usbredir_to_hcd(vdev->uhcd));
}

#define PORT_C_MASK				\
	((USB_PORT_STAT_C_CONNECTION		\
	  | USB_PORT_STAT_C_ENABLE		\
	  | USB_PORT_STAT_C_SUSPEND		\
	  | USB_PORT_STAT_C_OVERCURRENT		\
	  | USB_PORT_STAT_C_RESET) << 16)

/*
 * Returns 0 if the status hasn't changed, or the number of bytes in buf.
 * Ports are 0-indexed from the HCD point of view,
 * and 1-indexed from the USB core pointer of view.
 *
 * @buf: a bitmap to show which port status has been changed.
 *  bit  0: reserved
 *  bit  1: the status of port 0 has been changed.
 *  bit  2: the status of port 1 has been changed.
 *  ...
 */
static int hub_status(struct usb_hcd *hcd, char *buf)
{
	struct usbredir_hcd	*uhcd = hcd_to_usbredir(hcd);
	int		retval;
	int		rhport;
	int		changed = 0;

	retval = DIV_ROUND_UP(USBREDIR_NPORTS + 1, 8);
	memset(buf, 0, retval);

	spin_lock(&uhcd->lock);
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		pr_debug("hw accessible flag not on?\n");
		goto done;
	}

	/* check pseudo status register for each port */
	for (rhport = 0; rhport < USBREDIR_NPORTS; rhport++) {
		if ((uhcd->port_status[rhport] & PORT_C_MASK)) {
			/* The status of a port has been changed, */
			pr_debug("port %d status changed\n", rhport);

			buf[(rhport + 1) / 8] |= 1 << (rhport + 1) % 8;
			changed = 1;
		}
	}

	if ((hcd->state == HC_STATE_SUSPENDED) && (changed == 1))
		usb_hcd_resume_root_hub(hcd);

done:
	spin_unlock(&uhcd->lock);
	return changed ? retval : 0;
}


static inline void hub_descriptor(struct usb_hub_descriptor *desc)
{
	memset(desc, 0, sizeof(*desc));
// TODO - where do these magic numbers come from?
	desc->bDescriptorType = 0x29;
	desc->bDescLength = 9;
	desc->wHubCharacteristics = __constant_cpu_to_le16(
		HUB_CHAR_INDV_PORT_LPSM | HUB_CHAR_COMMON_OCPM);
	desc->bNbrPorts = USBREDIR_NPORTS;
	desc->u.hs.DeviceRemovable[0] = 0xff;
	desc->u.hs.DeviceRemovable[1] = 0xff;
}

static int hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue,
			    u16 wIndex, char *buf, u16 wLength)
{
	struct usbredir_hcd	*dum;
	int             retval = 0;
	int		rhport;

	u32 prev_port_status[USBREDIR_NPORTS];

	if (!HCD_HW_ACCESSIBLE(hcd))
		return -ETIMEDOUT;

	/*
	 * NOTE:
	 * wIndex shows the port number and begins from 1.
	 */
	pr_debug("hub_control: typeReq %x wValue %x wIndex %x\n",
		 typeReq, wValue, wIndex);
	rhport = ((__u8)(wIndex & 0x00ff)) - 1;

	dum = hcd_to_usbredir(hcd);

	spin_lock(&dum->lock);

	/* store old status and compare now and old later */
	memcpy(prev_port_status, dum->port_status,
			sizeof(prev_port_status));

	switch (typeReq) {
	case ClearHubFeature:
		pr_debug(" ClearHubFeature\n");
		break;
	case ClearPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			pr_debug(" ClearPortFeature: USB_PORT_FEAT_SUSPEND\n");
			if (dum->port_status[rhport] & USB_PORT_STAT_SUSPEND) {
				/* 20msec signaling */
				dum->resuming = 1;
				dum->re_timeout =
					jiffies + msecs_to_jiffies(20);
			}
			break;
		case USB_PORT_FEAT_POWER:
			pr_debug(" ClearPortFeature: USB_PORT_FEAT_POWER\n");
			dum->port_status[rhport] = 0;
			dum->resuming = 0;
			break;
		case USB_PORT_FEAT_C_RESET:
			pr_debug(" ClearPortFeature: USB_PORT_FEAT_C_RESET\n");
			// TODO - USB 3.0 stuff as well?
			switch (dum->vdev[rhport].connect_header.speed) {
			case usb_redir_speed_high:
				dum->port_status[rhport] |=
					USB_PORT_STAT_HIGH_SPEED;
				break;
			case usb_redir_speed_low:
				dum->port_status[rhport] |=
					USB_PORT_STAT_LOW_SPEED;
				break;
			default:
				break;
			}
		default:
			pr_debug(" ClearPortFeature: default %x\n", wValue);
			dum->port_status[rhport] &= ~(1 << wValue);
			break;
		}
		break;
	case GetHubDescriptor:
		pr_debug(" GetHubDescriptor\n");
		hub_descriptor((struct usb_hub_descriptor *) buf);
		break;
	case GetHubStatus:
		pr_debug(" GetHubStatus\n");
		*(__le32 *) buf = cpu_to_le32(0);
		break;
	case GetPortStatus:
		pr_debug(" GetPortStatus port %x\n", wIndex);
		if (wIndex > USBREDIR_NPORTS || wIndex < 1) {
			pr_err("invalid port number %d\n", wIndex);
			retval = -EPIPE;
		}

		/* TODO - read these comments and delete them or 
		 *   make sure JPW understands them */
		/* we do not care about resume. */

		/* whoever resets or resumes must GetPortStatus to
		 * complete it!!
		 */
		if (dum->resuming && time_after(jiffies, dum->re_timeout)) {
			dum->port_status[rhport] |=
				(1 << USB_PORT_FEAT_C_SUSPEND);
			dum->port_status[rhport] &=
				~(1 << USB_PORT_FEAT_SUSPEND);
			dum->resuming = 0;
			dum->re_timeout = 0;
		}

		if ((dum->port_status[rhport] & (1 << USB_PORT_FEAT_RESET)) !=
		    0 && time_after(jiffies, dum->re_timeout)) {
			dum->port_status[rhport] |=
				(1 << USB_PORT_FEAT_C_RESET);
			dum->port_status[rhport] &=
				~(1 << USB_PORT_FEAT_RESET);
			dum->re_timeout = 0;

			if (dum->vdev[rhport].status ==
			    VDEV_ST_NOTASSIGNED) {
				pr_debug(
					" enable rhport %d (status %u)\n",
					rhport,
					dum->vdev[rhport].status);
				dum->port_status[rhport] |=
					USB_PORT_STAT_ENABLE;
			}
		}
		((__le16 *) buf)[0] = cpu_to_le16(dum->port_status[rhport]);
		((__le16 *) buf)[1] =
			cpu_to_le16(dum->port_status[rhport] >> 16);

		pr_debug(" GetPortStatus bye %x %x\n", ((u16 *)buf)[0],
				  ((u16 *)buf)[1]);
		break;
	case SetHubFeature:
		pr_debug(" SetHubFeature\n");
		retval = -EPIPE;
		break;
	case SetPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			pr_debug(
				" SetPortFeature: USB_PORT_FEAT_SUSPEND\n");
			break;
		case USB_PORT_FEAT_RESET:
			pr_debug(
				" SetPortFeature: USB_PORT_FEAT_RESET\n");
			dum->port_status[rhport] &= ~USB_PORT_STAT_ENABLE;

			/* 50msec reset signaling */
			dum->re_timeout = jiffies + msecs_to_jiffies(50);

			/* FALLTHROUGH */
		default:
			pr_debug(" SetPortFeature: default %d\n", wValue);
			dum->port_status[rhport] |= (1 << wValue);
			break;
		}
		break;

	default:
		pr_err("default: no such request\n");

		/* "protocol stall" on error */
		retval = -EPIPE;
	}

	/* Only dump valid port status */
	if (rhport >= 0) {
		dump_port_status_diff(prev_port_status[rhport],
				      dum->port_status[rhport]);
	}

	spin_unlock(&dum->lock);

	return retval;
}

static int urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
			    gfp_t mem_flags)
{
	struct device *dev = &urb->dev->dev;
	int ret = 0;
	struct usbredir_hcd *uhcd = hcd_to_usbredir(hcd);
	struct usbredir_device *vdev;

	pr_debug("urb_enqueue: enter, usb_hcd %p urb %p mem_flags %d\n",
			  hcd, urb, mem_flags);

	spin_lock(&uhcd->lock);

	if (urb->status != -EINPROGRESS) {
		dev_err(dev, "URB already unlinked!, status %d\n", urb->status);
		spin_unlock(&uhcd->lock);
		return urb->status;
	}

	vdev = port_to_vdev(uhcd, urb->dev->portnum-1);

	/* refuse enqueue for dead connection */
	spin_lock(&vdev->lock);
	if (vdev->status == VDEV_ST_NULL ||
	    vdev->status == VDEV_ST_ERROR) {
		dev_err(dev, "enqueue for inactive port %d\n", vdev->rhport);
		spin_unlock(&vdev->lock);
		spin_unlock(&uhcd->lock);
		return -ENODEV;
	}
	spin_unlock(&vdev->lock);

	ret = usb_hcd_link_urb_to_ep(hcd, urb);
	if (ret)
		goto no_need_unlink;

	/*
	 * The enumeration process is as follows;
	 *
	 *  1. Get_Descriptor request to DevAddrs(0) EndPoint(0)
	 *     to get max packet length of default pipe
	 *
	 *  2. Set_Address request to DevAddr(0) EndPoint(0)
	 *
	 */
	if (usb_pipedevice(urb->pipe) == 0) {
		__u8 type = usb_pipetype(urb->pipe);
		struct usb_ctrlrequest *ctrlreq =
			(struct usb_ctrlrequest *) urb->setup_packet;

		if (type != PIPE_CONTROL || !ctrlreq) {
			dev_err(dev, "invalid request to devnum 0\n");
			ret = -EINVAL;
			goto no_need_xmit;
		}

		switch (ctrlreq->bRequest) {
		case USB_REQ_SET_ADDRESS:
			/* set_address may come when a device is reset */
			dev_info(dev, "SetAddress Request (%d) to port %d\n",
				 ctrlreq->wValue, vdev->rhport);

			usb_put_dev(vdev->udev);
			vdev->udev = usb_get_dev(urb->dev);

			spin_lock(&vdev->lock);
			vdev->status = VDEV_ST_USED;
			spin_unlock(&vdev->lock);

			if (urb->status == -EINPROGRESS) {
				/* This request is successfully completed. */
				/* If not -EINPROGRESS, possibly unlinked. */
				urb->status = 0;
			}

			goto no_need_xmit;

		case USB_REQ_GET_DESCRIPTOR:
			pr_debug("Requesting descriptor; wValue %x\n",
				 ctrlreq->wValue);
			if (ctrlreq->wValue == cpu_to_le16(USB_DT_DEVICE << 8))
				pr_debug(
					"Not yet?:Get_Descriptor to device 0 (get max pipe size)\n");

			usb_put_dev(vdev->udev);
			vdev->udev = usb_get_dev(urb->dev);
			goto out;

		default:
			/* NOT REACHED */
			dev_err(dev,
				"invalid request to devnum 0 bRequest %u, wValue %u\n",
				ctrlreq->bRequest,
				ctrlreq->wValue);
			ret =  -EINVAL;
			goto no_need_xmit;
		}

	}

out:
	tx_urb(urb);
	spin_unlock(&uhcd->lock);

	return 0;

no_need_xmit:
	usb_hcd_unlink_urb_from_ep(hcd, urb);
no_need_unlink:
	spin_unlock(&uhcd->lock);
	usb_hcd_giveback_urb(usbredir_to_hcd(uhcd), urb, urb->status);
	return ret;
}

static int urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct usbredir_priv *priv;
	struct usbredir_device *vdev;

	pr_debug("enter dequeue urb %p\n", urb);

	spin_lock(&the_controller->lock);

	priv = urb->hcpriv;
	if (!priv) {
		/* URB was never linked! or will be soon given back by
		 * rx_loop. */
		spin_unlock(&the_controller->lock);
		return 0;
	}

	// TODO - this is not tidy...
	{
		int ret = 0;

		ret = usb_hcd_check_unlink_urb(hcd, urb, status);
		if (ret) {
			spin_unlock(&the_controller->lock);
			return ret;
		}
	}

	 /* TODO - understand this comment: 'send unlink request here?' */
	vdev = priv->vdev;

	if (!vdev->socket) {
		/* tcp connection is closed */
		spin_lock(&vdev->priv_lock);

		pr_info("device %p seems to be disconnected\n", vdev);
		list_del(&priv->list);
		kfree(priv);
		urb->hcpriv = NULL;

		spin_unlock(&vdev->priv_lock);

		pr_info("gives back urb %p\n", urb);

		usb_hcd_unlink_urb_from_ep(hcd, urb);

		spin_unlock(&the_controller->lock);
		usb_hcd_giveback_urb(usbredir_to_hcd(the_controller), urb,
				     urb->status);
		spin_lock(&the_controller->lock);

	} else {
		/* tcp connection is alive */
		struct usbredir_unlink *unlink;

		spin_lock(&vdev->priv_lock);

		/* setup CMD_UNLINK pdu */
		unlink = kzalloc(sizeof(struct usbredir_unlink), GFP_ATOMIC);
		if (!unlink) {
			spin_unlock(&vdev->priv_lock);
			spin_unlock(&the_controller->lock);
			usbredir_event_add(vdev, VDEV_EVENT_ERROR_MALLOC);
			return -ENOMEM;
		}

		unlink->seqnum = atomic_inc_return(&the_controller->aseqnum);

		unlink->unlink_seqnum = priv->seqnum;

		/* send cmd_unlink and try to cancel the pending URB in the
		 * peer */
		list_add_tail(&unlink->list, &vdev->unlink_tx);
		wake_up(&vdev->waitq_tx);

		spin_unlock(&vdev->priv_lock);
	}

	spin_unlock(&the_controller->lock);

	pr_debug("leave dequeue urb %p\n", urb);
	return 0;
}

static void usbredir_device_unlink_cleanup(struct usbredir_device *vdev)
{
	struct usbredir_unlink *unlink, *tmp;

	spin_lock(&vdev->uhcd->lock);
	spin_lock(&vdev->priv_lock);

	list_for_each_entry_safe(unlink, tmp, &vdev->unlink_tx, list) {
		pr_debug("unlink cleanup tx %d\n", unlink->unlink_seqnum);
		list_del(&unlink->list);
		kfree(unlink);
	}

	while (!list_empty(&vdev->unlink_rx)) {
		struct urb *urb;

		unlink = list_first_entry(&vdev->unlink_rx, struct usbredir_unlink,
			list);

		/* give back URB of unanswered unlink request */
		pr_debug("unlink cleanup rx %d\n", unlink->unlink_seqnum);

		urb = pickup_urb_and_free_priv(vdev, unlink->unlink_seqnum);
		if (!urb) {
			pr_info("the urb (seqnum %d) was already given back\n",
				unlink->unlink_seqnum);
			list_del(&unlink->list);
			kfree(unlink);
			continue;
		}

		urb->status = -ENODEV;

		usb_hcd_unlink_urb_from_ep(usbredir_to_hcd(vdev->uhcd), urb);

		list_del(&unlink->list);

		spin_unlock(&vdev->priv_lock);
		spin_unlock(&vdev->uhcd->lock);

		usb_hcd_giveback_urb(usbredir_to_hcd(vdev->uhcd), urb,
				     urb->status);

		spin_lock(&vdev->uhcd->lock);
		spin_lock(&vdev->priv_lock);

		kfree(unlink);
	}

	spin_unlock(&vdev->priv_lock);
	spin_unlock(&vdev->uhcd->lock);
}

/*
 * The important thing is that only one context begins cleanup.
 * This is why error handling and cleanup become simple.
 * We do not want to consider race condition as possible.
 */
static void usbredir_shutdown_connection(struct usbredir_device *vdev)
{
	if (vdev->socket) {
		pr_debug("shutdown socket %p\n", vdev->socket);
		kernel_sock_shutdown(vdev->socket, SHUT_RDWR);
	}

	/* kill threads related to this dev */
	if (vdev->rx) {
		kthread_stop(vdev->rx);
		vdev->rx = NULL;
	}
	if (vdev->tx) {
		kthread_stop(vdev->tx);
		vdev->tx = NULL;
	}

	/* active connection is closed */
	if (vdev->socket) {
		sockfd_put(vdev->socket);
		vdev->socket = NULL;
	}

	usbredir_device_unlink_cleanup(vdev);

	/*
	 * rh_port_disconnect() is a trigger of ...
	 *   usb_disable_device():
	 *	disable all the endpoints for a USB device.
	 *   usb_disable_endpoint():
	 *	disable endpoints. pending urbs are unlinked(dequeued).
	 *
	 * NOTE: After calling rh_port_disconnect(), the USB device drivers of a
	 * detached device should release used urbs in a cleanup function (i.e.
	 * xxx_disconnect()). Therefore, usbredir_hcd does not need to release
	 * pushed urbs and their private data in this function.
	 *
	 * NOTE: usbredir_dequeue() must be considered carefully. When shutting down
	 * a connection, shutdown_connection() expects usbredir_dequeue()
	 * gives back pushed urbs and frees their private data by request of
	 * the cleanup function of a USB driver. When unlinking a urb with an
	 * active connection, usbredir_dequeue() does not give back the urb which
	 * is actually given back by rx_loop after receiving its return pdu.
	 *
	 */
	rh_port_disconnect(vdev);

	pr_info("disconnect device\n");
}


static void usbredir_device_reset(struct usbredir_device *vdev)
{
	spin_lock(&vdev->lock);

	if (vdev->devid)
		kfree(vdev->devid);
	vdev->devid  = NULL;

	usb_put_dev(vdev->udev);
	vdev->udev = NULL;

	if (vdev->socket) {
		// TODO - kernel shut down?
		sockfd_put(vdev->socket);
		vdev->socket = NULL;
	}
	// TODO - clean up parser and threads?
	vdev->status = VDEV_ST_NULL;

	spin_unlock(&vdev->lock);
}

static void usbredir_device_unusable(struct usbredir_device *vdev)
{
	spin_lock(&vdev->lock);
	vdev->status = VDEV_ST_ERROR;
	spin_unlock(&vdev->lock);
}

static void usbredir_device_init(struct usbredir_device *vdev, int port)
{
	memset(vdev, 0, sizeof(*vdev));

	vdev->rhport = port;
	vdev->status = VDEV_ST_NULL;
	spin_lock_init(&vdev->lock);

	INIT_LIST_HEAD(&vdev->priv_rx);
	INIT_LIST_HEAD(&vdev->priv_tx);
	INIT_LIST_HEAD(&vdev->unlink_tx);
	INIT_LIST_HEAD(&vdev->unlink_rx);
	spin_lock_init(&vdev->priv_lock);

	init_waitqueue_head(&vdev->waitq_tx);

	// TODO - why callbacks?  Why not just invoke directly?
	vdev->eh_ops.shutdown = usbredir_shutdown_connection;
	vdev->eh_ops.reset = usbredir_device_reset;
	vdev->eh_ops.unusable = usbredir_device_unusable;

	usbredir_start_eh(vdev);
}

static int usbredir_start(struct usb_hcd *hcd)
{
	struct usbredir_hcd *uhcd = hcd_to_usbredir(hcd);
	int rhport;
	int err = 0;

	/* initialize private data of usb_hcd */

	for (rhport = 0; rhport < USBREDIR_NPORTS; rhport++) {
		struct usbredir_device *vdev = &uhcd->vdev[rhport];

		usbredir_device_init(vdev, rhport);
	}

	atomic_set(&uhcd->aseqnum, 0);
	spin_lock_init(&uhcd->lock);

	hcd->power_budget = 0; /* no limit */
	hcd->uses_new_polling = 1;

	/* usbredir_hcd is now ready to be controlled through sysfs */
	err = sysfs_create_group(&usbredir_dev(uhcd)->kobj, &hub_attr_group);
	if (err) {
		pr_err("create sysfs files\n");
		return err;
	}

	return 0;
}

static void usbredir_stop(struct usb_hcd *hcd)
{
	struct usbredir_hcd *uhcd = hcd_to_usbredir(hcd);
	int rhport = 0;

	/* 1. remove the userland interface of usbredir_hcd */
	sysfs_remove_group(&usbredir_dev(uhcd)->kobj, &hub_attr_group);

	/* 2. shutdown all the ports of usbredir_hcd */
	for (rhport = 0; rhport < USBREDIR_NPORTS; rhport++) {
		struct usbredir_device *vdev = &uhcd->vdev[rhport];

		usbredir_event_add(vdev, VDEV_EVENT_REMOVED);
		usbredir_stop_eh(vdev); // TODO - shouldn't we let that event do
					// the stop?
	}
}

static int get_frame_number(struct usb_hcd *hcd)
{
	pr_err("get_frame_number: Not yet implemented\n"); // TODO
	return 0;
}

#ifdef CONFIG_PM

/* FIXME: suspend/resume */
static int bus_suspend(struct usb_hcd *hcd)
{
	struct usbredir_hcd *uhcd = hcd_to_usbredir(hcd);

	dev_dbg(&hcd->self.root_hub->dev, "%s\n", __func__);

	spin_lock(&uhcd->lock);
	hcd->state = HC_STATE_SUSPENDED;
	spin_unlock(&uhcd->lock);

	return 0;
}

static int bus_resume(struct usb_hcd *hcd)
{
	struct usbredir_hcd *uhcd = hcd_to_usbredir(hcd);
	int rc = 0;

	dev_dbg(&hcd->self.root_hub->dev, "%s\n", __func__);

	spin_lock(&uhcd->lock);
	if (!HCD_HW_ACCESSIBLE(hcd))
		rc = -ESHUTDOWN;
	else
		hcd->state = HC_STATE_RUNNING;
	spin_unlock(&uhcd->lock);

	return rc;
}

#else

#define bus_suspend      NULL
#define bus_resume       NULL
#endif

static struct hc_driver usbredir_hc_driver = {
	.description	= driver_name,
	.product_desc	= driver_desc,
	.hcd_priv_size	= sizeof(struct usbredir_hcd),

	// TODO = what other flags are available and what of USB3?
	.flags		= HCD_USB2,

	.start		= usbredir_start,
	.stop		= usbredir_stop,

	.urb_enqueue	= urb_enqueue,
	.urb_dequeue	= urb_dequeue,

	.get_frame_number = get_frame_number,

	.hub_status_data = hub_status,
	.hub_control    = hub_control,
	.bus_suspend	= bus_suspend,
	.bus_resume	= bus_resume,
};

static int usbredir_hcd_probe(struct platform_device *pdev)
{
	struct usb_hcd		*hcd;
	int			ret;

	pr_debug("probe name %s id %d\n", pdev->name, pdev->id);


	/* TODO - compare/contrast with usb_create_shared_hcd */

	/*
	 * Allocate and initialize hcd.
	 * Our private data is also allocated automatically.
	 */
	hcd = usb_create_hcd(&usbredir_hc_driver, &pdev->dev,
			     dev_name(&pdev->dev));
	if (!hcd) {
		pr_err("create hcd failed\n");
		return -ENOMEM;
	}
	// TODO - review if we want to has_tt, and anything like it...
	hcd->has_tt = 1;

	/* this is private data for usbredir_hcd */
	the_controller = hcd_to_usbredir(hcd);

	/*
	 * Finish generic HCD structure initialization and register.
	 * Call the driver's reset() and start() routines.
	 */
	ret = usb_add_hcd(hcd, 0, 0);
	if (ret != 0) {
		pr_err("usb_add_hcd failed %d\n", ret);
		usb_put_hcd(hcd);
		the_controller = NULL;
		return ret;
	}

	return 0;
}

static int usbredir_hcd_remove(struct platform_device *pdev)
{
	struct usb_hcd	*hcd;

	hcd = platform_get_drvdata(pdev);
	if (!hcd)
		return 0;

	/*
	 * Disconnects the root hub,
	 * then reverses the effects of usb_add_hcd(),
	 * invoking the HCD's stop() methods.
	 */
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	the_controller = NULL;

	return 0;
}

#ifdef CONFIG_PM

/* TODO:  what should happen under suspend/resume? */
static int usbredir_hcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct usb_hcd *hcd;
	int rhport = 0;
	int connected = 0;
	int ret = 0;

	hcd = platform_get_drvdata(pdev);

	spin_lock(&the_controller->lock);

	for (rhport = 0; rhport < USBREDIR_NPORTS; rhport++)
		if (the_controller->port_status[rhport] &
		    USB_PORT_STAT_CONNECTION)
			connected += 1;

	spin_unlock(&the_controller->lock);

	if (connected > 0) {
		dev_info(&pdev->dev,
			 "We have %d active connection%s. Do not suspend.\n",
			 connected, (connected == 1 ? "" : "s"));
		ret =  -EBUSY;
	} else {
		dev_info(&pdev->dev, "suspend usbredir_hcd");
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	}

	return ret;
}

static int usbredir_hcd_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	hcd = platform_get_drvdata(pdev);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	usb_hcd_poll_rh_status(hcd);

	return 0;
}

#else

#define usbredir_hcd_suspend	NULL
#define usbredir_hcd_resume	NULL

#endif

static struct platform_driver usbredir_driver = {
	.probe	= usbredir_hcd_probe,
	.remove	= usbredir_hcd_remove,
	.suspend = usbredir_hcd_suspend,
	.resume	= usbredir_hcd_resume,
	.driver	= {
		.name = driver_name,
	},
};

/*
 * The USBREDIR 'device' is 'virtual'; not real plug&play hardware.
 * We need to add this virtual device as a platform device arbitrarily:
 *	1. platform_device_register()
 */
static void the_pdev_release(struct device *dev)
{
}

static struct platform_device the_pdev = {
	/* should be the same name as driver_name */
	.name = driver_name,
	.id = -1, // TODO - this will have to not be -1 when we go general
	.dev = {
		.release = the_pdev_release,
	},
};

static int __init usbredir_hcd_init(void)
{
	int ret;

	if (usb_disabled())
		return -ENODEV;

	ret = platform_driver_register(&usbredir_driver);
	if (ret)
		goto err_driver_register;

	ret = platform_device_register(&the_pdev);
	if (ret)
		goto err_platform_device_register;

	pr_info(DRIVER_DESC " v" DRIVER_VERSION "\n");
	return ret;

err_platform_device_register:
	platform_driver_unregister(&usbredir_driver);
err_driver_register:
	return ret;
}

static void __exit usbredir_hcd_exit(void)
{
	platform_device_unregister(&the_pdev);
	platform_driver_unregister(&usbredir_driver);
}

module_init(usbredir_hcd_init);
module_exit(usbredir_hcd_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
