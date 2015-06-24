/*
 * Copyright (C) 2015 Jeremy White loosely based on vhci_hcd which is
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

#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "usbredir.h"

static spinlock_t hubs_lock;
static struct list_head hubs;
static atomic_t hub_count;

static int usbredir_hub_start(struct usb_hcd *hcd)
{
	struct usbredir_hub *hub = usbredir_hub_from_hcd(hcd);
	int i;

	pr_debug("usbredir_hub_start %p\n", hub);

	spin_lock(&hub->lock);

	hub->device_count = devices_per_hub;
	hub->devices = kzalloc(hub->device_count * sizeof(*hub->devices),
			       GFP_ATOMIC);
	if (! hub->devices) {
		spin_unlock(&hub->lock);
		return -ENOMEM;
	}

	for (i = 0; i < hub->device_count; i++)
		usbredir_device_init(hub->devices + i, i);

	hcd->power_budget = 0; /* no limit */
	hcd->uses_new_polling = 1;
	atomic_set(&hub->aseqnum, 0);
	spin_unlock(&hub->lock);

	return 0;
}

static void usbredir_hub_stop(struct usb_hcd *hcd)
{
	struct usbredir_hub *hub = usbredir_hub_from_hcd(hcd);
	int i;

	pr_debug("usbredir_hub_stop %p\n", hub);

	spin_lock(&hub->lock);

	for (i = 0; i < hub->device_count && hub->devices; i++)
		usbredir_device_destroy(hub->devices + i);

	if (hub->devices)
		kfree(hub->devices);
	hub->devices = NULL;
	hub->device_count = 0;

	spin_unlock(&hub->lock);
}

static int urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
			    gfp_t mem_flags)
{
#if defined(HACK_FOR_NOW)
	struct device *dev = &urb->dev->dev;
	int ret = 0;
	struct usbredir_hcd *uhcd = hcd_to_usbredir(hcd);
	struct usbredir_device *vdev;
#endif
	pr_debug("urb_enqueue: enter, usb_hcd %p urb %p mem_flags %d\n",
			  hcd, urb, mem_flags);

	return 0;
#if defined(HACK_FOR_NOW)
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
#endif
}

static int urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
#if defined(HACK_FOR_NOW)
	struct usbredir_priv *priv;
	struct usbredir_device *vdev;
#endif

	pr_debug("enter dequeue urb %p\n", urb);
#if defined(HACK_FOR_NOW)

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

#endif
	pr_debug("leave dequeue urb %p\n", urb);
	return 0;
}



static int get_frame_number(struct usb_hcd *hcd)
{
	pr_err("get_frame_number: Not yet implemented\n"); // TODO
	return 0;
}

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
static int usbredir_hub_status(struct usb_hcd *hcd, char *buf)
{
	struct usbredir_hub *hub = usbredir_hub_from_hcd(hcd);
	int		ret;
	int		rhport;
	int		changed = 0;

	pr_debug("usbredir_hub_status for %p\n", hub);

	spin_lock(&hub->lock);

	ret = DIV_ROUND_UP(hub->device_count + 1, 8);
	memset(buf, 0, ret);

	if (!HCD_HW_ACCESSIBLE(hcd)) {
		pr_debug("hw accessible flag not on?\n");
		goto done;
	}

	/* check pseudo status register for each port */
	for (rhport = 0; rhport < hub->device_count; rhport++) {
		struct usbredir_device *dev = hub->devices + rhport;
		spin_lock(&dev->lock);
		/* TODO - this left shift 16 puzzles me... */
		if (dev->port_status &
			((  USB_PORT_STAT_C_CONNECTION
			  | USB_PORT_STAT_C_ENABLE
			  | USB_PORT_STAT_C_SUSPEND
			  | USB_PORT_STAT_C_OVERCURRENT
			  | USB_PORT_STAT_C_RESET) << 16)) {

			/* The status of a port has been changed, */
			pr_debug("port %d status changed\n", rhport);

			buf[(rhport + 1) / 8] |= 1 << (rhport + 1) % 8;
			changed = 1;
		}
		spin_unlock(&dev->lock);
	}

	if ((hcd->state == HC_STATE_SUSPENDED) && (changed == 1))
		usb_hcd_resume_root_hub(hcd);

done:
	spin_unlock(&hub->lock);
	return changed ? ret : 0;
}

static inline void usbredir_hub_descriptor(struct usbredir_hub *hub,
					   struct usb_hub_descriptor *desc)
{
	memset(desc, 0, sizeof(*desc));
// TODO - where do these magic numbers come from?
	desc->bDescriptorType = 0x29;
	desc->bDescLength = 9;
	desc->wHubCharacteristics = __constant_cpu_to_le16(
		HUB_CHAR_INDV_PORT_LPSM | HUB_CHAR_COMMON_OCPM);
	spin_lock(&hub->lock);
	desc->bNbrPorts = hub->device_count;
	spin_unlock(&hub->lock);
	desc->u.hs.DeviceRemovable[0] = 0xff;
	desc->u.hs.DeviceRemovable[1] = 0xff;
}

static int usbredir_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue,
			    u16 wIndex, char *buf, u16 wLength)
{
	struct usbredir_hub *hub;
	int             ret = 0;
	int		rhport;

	pr_debug("usbredir_hub_control "
		 "[hcd %p|typeReq %x|wValue %x|wIndex%u|wLength %u]\n",
		 hcd, typeReq, wValue, wIndex, wLength);

	if (!HCD_HW_ACCESSIBLE(hcd))
		return -ETIMEDOUT;

	/* wIndex is 1 based */
	rhport = ((__u8)(wIndex & 0x00ff)) - 1;

	hub = usbredir_hub_from_hcd(hcd);

	switch (typeReq) {
	case ClearHubFeature:
		pr_debug(" ClearHubFeature\n");
		break;
	case ClearPortFeature:
		pr_debug(" ClearPortFeature\n");
		return usbredir_device_clear_port_feature(hub, rhport, wValue);
	case GetHubDescriptor:
		pr_debug(" GetHubDescriptor\n");
		usbredir_hub_descriptor(hub, (struct usb_hub_descriptor *) buf);
		break;
	case GetHubStatus:
		pr_debug(" GetHubStatus\n");
		*(__le32 *) buf = cpu_to_le32(0);
		break;
	case GetPortStatus:
		pr_debug(" GetPortStatus\n");
		return usbredir_device_port_status(hub, rhport, buf);
	case SetHubFeature:
		pr_debug(" SetHubFeature\n");
		ret = -EPIPE;
		break;
	case SetPortFeature:
		pr_debug(" SetPortFeature\n");
		return usbredir_device_set_port_feature(hub, rhport, wValue);

	default:
		pr_err("usbredir_hub_control: no such request %x\n", typeReq);

		/* "protocol stall" on error */
		ret = -EPIPE;
	}
	return ret;
}

#ifdef CONFIG_PM
/* FIXME: suspend/resume */
static int bus_suspend(struct usb_hcd *hcd)
{
	struct usbredir_hub *hub = usbredir_hub_from_hcd(hcd);

	dev_dbg(&hcd->self.root_hub->dev, "%s\n", __func__);

	spin_lock(&hub->lock);
	hcd->state = HC_STATE_SUSPENDED;
	spin_unlock(&hub->lock);

	return 0;
}

static int bus_resume(struct usb_hcd *hcd)
{
	int rc = 0;
	struct usbredir_hub *hub = usbredir_hub_from_hcd(hcd);

	dev_dbg(&hcd->self.root_hub->dev, "%s\n", __func__);

	spin_lock(&hub->lock);
	if (!HCD_HW_ACCESSIBLE(hcd))
		rc = -ESHUTDOWN;
	else
		hcd->state = HC_STATE_RUNNING;
	spin_unlock(&hub->lock);
	return rc;
}
#else

#define bus_suspend      NULL
#define bus_resume       NULL
#endif


static void usbredir_release_hub_dev(struct device *dev)
{
	/* TODO - do we need to implement anything here? */
	pr_info("usbredir_release_hub_dev %p\n", dev);
}

static int usbredir_register_hub(struct usbredir_hub *hub)
{
	int ret;

	hub->pdev.name = driver_name;
	hub->pdev.id = hub->id;
	hub->pdev.dev.release = usbredir_release_hub_dev;

	ret = platform_device_register(&hub->pdev);
	if (ret) {
		pr_err("Unable to register platform device %d", hub->id);
		return ret;
	}

	return 0;
}

static void usbredir_unregister_hub(struct usbredir_hub *hub)
{
	platform_device_unregister(&hub->pdev);
}


static struct hc_driver usbredir_hc_driver = {
	.description	= driver_name,
	.product_desc	= driver_desc,
	.hcd_priv_size	= sizeof(struct usbredir_hub *),

	// TODO = what other flags are available and what of USB3?
	.flags		= HCD_USB2,

	.start		= usbredir_hub_start,
	.stop		= usbredir_hub_stop,

	.urb_enqueue	= urb_enqueue,
	.urb_dequeue	= urb_dequeue,

	.get_frame_number = get_frame_number,

	.hub_status_data = usbredir_hub_status,
	.hub_control    = usbredir_hub_control,
	.bus_suspend	= bus_suspend,
	.bus_resume	= bus_resume,
};


static int usbredir_create_hcd(struct usbredir_hub *hub)
{
	int ret;
	hub->hcd = usb_create_hcd(&usbredir_hc_driver, &hub->pdev.dev,
			     dev_name(&hub->pdev.dev));
	if (!hub->hcd) {
		pr_err("usb_create_hcd failed\n");
		return -ENOMEM;
	}

	// TODO - review if we want to has_tt, and anything like it...
	hub->hcd->has_tt = 1;
	* ((struct usbredir_hub **) hub->hcd->hcd_priv) = hub;

	ret = usb_add_hcd(hub->hcd, 0, 0);
	if (ret != 0) {
		pr_err("usb_add_hcd failed %d\n", ret);
		usb_put_hcd(hub->hcd);
		return ret;
	}

	return 0;
}

static void usbredir_destroy_hcd(struct usbredir_hub *hub)
{
	if (hub->hcd) {
		usb_remove_hcd(hub->hcd);
		usb_put_hcd(hub->hcd);
	}
	hub->hcd = NULL;
}

struct usbredir_hub *usbredir_hub_create(void)
{
	struct usbredir_hub *hub;
	int id = atomic_inc_return(&hub_count);
	if (id > max_hubs)
		goto dec_exit;

	hub = kzalloc(sizeof(*hub), GFP_ATOMIC);
	if (!hub)
		goto dec_exit;
	hub->id = id - 1;

	if (usbredir_register_hub(hub)) {
		kfree(hub);
		goto dec_exit;
	}

	if (usbredir_create_hcd(hub)) {
		usbredir_unregister_hub(hub);
		kfree(hub);
		goto dec_exit;
	}

	spin_lock_init(&hub->lock);

	spin_lock(&hubs_lock);
	list_add_tail(&hub->list, &hubs);
	spin_unlock(&hubs_lock);
	return hub;
dec_exit:
	atomic_dec(&hub_count);
	return NULL;
}

void usbredir_hub_destroy(struct usbredir_hub *hub)
{
	pr_err("TODO better implement destroy of %p\n", hub);
	usbredir_destroy_hcd(hub);
	usbredir_unregister_hub(hub);
}

int usbredir_hub_init(void)
{
	INIT_LIST_HEAD(&hubs);
	atomic_set(&hub_count, 0);
	spin_lock_init(&hubs_lock);

	return 0;
}

void usbredir_hub_exit(void)
{
}
