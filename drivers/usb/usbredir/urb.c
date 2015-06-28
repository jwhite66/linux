/*
 * Copyright (C) 2015 Jeremy White based on work by
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
 */

#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "usbredir.h"

/* TODO - this logic was mostly copied from USBIP.
 *        While some of it has been dissected, it
 *        really needs a much more thoughtful review
 *        and analysis */

int urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
			    gfp_t mem_flags)
{
	struct device *dev = &urb->dev->dev;
	int ret = 0;
	struct usbredir_hub *hub = usbredir_hub_from_hcd(hcd);
	struct usbredir_device *udev;

	pr_debug("urb_enqueue: enter, usb_hcd %p urb %p mem_flags %d\n",
			  hcd, urb, mem_flags);

	if (urb->status != -EINPROGRESS) {
		dev_err(dev, "URB already unlinked!, status %d\n", urb->status);
		return urb->status;
	}

	udev = hub->devices + urb->dev->portnum - 1;

	/* refuse enqueue for dead connection */
	if (!atomic_read(&udev->active)) {
		dev_err(dev, "enqueue for inactive port %d\n", udev->rhport);
		return -ENODEV;
	}

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
				 ctrlreq->wValue, udev->rhport);

			usb_put_dev(udev->usb_dev);
			udev->usb_dev = usb_get_dev(urb->dev);

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

			usb_put_dev(udev->usb_dev);
			udev->usb_dev = usb_get_dev(urb->dev);
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
	tx_urb(udev, urb);

	return 0;

no_need_xmit:
	usb_hcd_unlink_urb_from_ep(hcd, urb);
no_need_unlink:
	usb_hcd_giveback_urb(hub->hcd, urb, urb->status);
	return ret;
}

int urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct usbredir_urb *uurb;
	struct usbredir_device *udev;
	struct usbredir_hub *hub = usbredir_hub_from_hcd(hcd);

	pr_debug("enter dequeue urb %p\n", urb);

	uurb = urb->hcpriv;
	if (!uurb) {
		/* URB was never linked! or will be soon given back by
		 * rx_loop. */
		return 0;
	}

	/* TODO - this is not tidy... */
	{
		int ret = 0;

		ret = usb_hcd_check_unlink_urb(hcd, urb, status);
		if (ret) {
			return ret;
		}
	}

	 /* TODO - understand this comment: 'send unlink request here?' */
	udev = uurb->udev;

	if (!udev->socket) {
		/* tcp connection is closed */
		spin_lock(&udev->lists_lock);

		pr_info("device %p seems to be disconnected\n", udev);
		list_del(&uurb->list);
		kfree(uurb);
		urb->hcpriv = NULL;

		spin_unlock(&udev->lists_lock);

		pr_info("gives back urb %p\n", urb);

		usb_hcd_unlink_urb_from_ep(hcd, urb);

		usb_hcd_giveback_urb(hub->hcd, urb, urb->status);

	} else {
		/* tcp connection is alive */
		struct usbredir_unlink *unlink;

		spin_lock(&udev->lists_lock);

		/* setup CMD_UNLINK pdu */
		unlink = kzalloc(sizeof(struct usbredir_unlink), GFP_ATOMIC);
		if (!unlink) {
			spin_unlock(&udev->lists_lock);
			/* TODO complain somehow... */
			return -ENOMEM;
		}

		unlink->seqnum = usbredir_hub_seqnum(hub);

		unlink->unlink_seqnum = uurb->seqnum;

		/* send cmd_unlink and try to cancel the pending URB in the
		 * peer */
		list_add_tail(&unlink->list, &udev->unlink_tx);
		wake_up(&udev->waitq_tx);

		spin_unlock(&udev->lists_lock);
	}

	pr_debug("leave dequeue urb %p\n", urb);
	return 0;
}
