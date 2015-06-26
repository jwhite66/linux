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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307,
 * USA.
 */

#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/kthread.h>

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
		usbredir_device_init(hub->devices + i, i, hub);

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
		usbredir_device_deallocate(hub->devices + i);

	if (hub->devices)
		kfree(hub->devices);
	hub->devices = NULL;
	hub->device_count = 0;

	spin_unlock(&hub->lock);
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
		struct usbredir_device *udev = hub->devices + rhport;
		spin_lock(&udev->lock);
		if (udev->port_status &
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
		spin_unlock(&udev->lock);
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
		pr_err("Unable to register platform device %d\n", hub->id);
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

	// TODO - no one else stores a pointer
	//        may want to rethink the structure.
	//        Question:  do we really need to create the pdev first?
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
	usbredir_hub_stop(hub->hcd);
	usbredir_destroy_hcd(hub);
	usbredir_unregister_hub(hub);
}


struct usbredir_device *usbredir_hub_find_device(const char *devid)
{
	struct usbredir_device *ret = NULL;
	struct usbredir_hub *hub;
	int i;
	spin_lock(&hubs_lock);
	list_for_each_entry(hub, &hubs, list) {
		spin_lock(&hub->lock);
		for (i = 0; i < hub->device_count; i++) {
			struct usbredir_device *udev = hub->devices + i;
			spin_lock(&udev->lock);
			if (atomic_read(&udev->active) &&
			    udev->devid &&
			    strcmp(udev->devid, devid) == 0)
				ret = udev;
			spin_unlock(&udev->lock);
			if (ret)
				break;
		}
		spin_unlock(&hub->lock);
		if (ret)
			break;
	}
	spin_unlock(&hubs_lock);
	return ret;
}

struct usbredir_device *usbredir_hub_allocate_device(const char *devid,
						     struct socket *socket)
{
	int found = 0;
	struct usbredir_hub *hub;
	struct usbredir_device *udev;
	int i;

	spin_lock(&hubs_lock);
	list_for_each_entry(hub, &hubs, list) {
		spin_lock(&hub->lock);
		for (i = 0; i < hub->device_count; i++) {
			udev = hub->devices + i;
			spin_lock(&udev->lock);
			if (! atomic_read(&udev->active)) {
				found++;
				break;
			}
			spin_unlock(&udev->lock);
		}
		if (found)
			break;
		spin_unlock(&hub->lock);
	}
	spin_unlock(&hubs_lock);

	if (! found) {
		hub = usbredir_hub_create();
		if (! hub)
			return NULL;

		return usbredir_hub_allocate_device(devid, socket);
	}

	usbredir_device_allocate(udev, devid, socket);

	spin_unlock(&udev->lock);
	spin_unlock(&hub->lock);

	return udev;
}

int usbredir_hub_show_global_status(char *out)
{
	int count = 0;
	int active = 0;
	int used = 0;

	struct usbredir_hub *hub;
	struct usbredir_device *udev;
	int i;

	spin_lock(&hubs_lock);
	list_for_each_entry(hub, &hubs, list) {
		spin_lock(&hub->lock);
		for (i = 0; i < hub->device_count; count++, i++) {
			udev = hub->devices + i;
			spin_lock(&udev->lock);
			active += atomic_read(&udev->active);
			if (udev->usb_dev)
				used++;
			spin_unlock(&udev->lock);
		}
		spin_unlock(&hub->lock);
	}
	spin_unlock(&hubs_lock);

	sprintf(out, "%d/%d hubs. %d/%d devices (%d active, %d used).\n",
			atomic_read(&hub_count), max_hubs,
			count, max_hubs * devices_per_hub, active, used);

	return strlen(out);
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
	struct usbredir_hub *hub;

	spin_lock(&hubs_lock);
	list_for_each_entry(hub, &hubs, list) {
		usbredir_hub_destroy(hub);
		// TODO list remove, kfree
	}
	spin_unlock(&hubs_lock);
}
