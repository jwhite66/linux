/*
 * Copyright (C) 2015 Jeremy White
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

#include "usbredir.h"

void usbredir_device_init(struct usbredir_device *dev, int port)
{
	memset(dev, 0, sizeof(*dev));

	dev->rhport = port;
	dev->status = VDEV_ST_NULL;
	spin_lock_init(&dev->lock);

	INIT_LIST_HEAD(&dev->priv_rx);
	INIT_LIST_HEAD(&dev->priv_tx);
	INIT_LIST_HEAD(&dev->unlink_tx);
	INIT_LIST_HEAD(&dev->unlink_rx);

	spin_lock_init(&dev->priv_lock);

	init_waitqueue_head(&dev->waitq_tx);
}

void usbredir_device_destroy(struct usbredir_device *dev)
{
}


static int valid_port(struct usbredir_hub *hub, int rhport)
{
	int ret;
	spin_lock(&hub->lock);
	ret = rhport >= 0 && rhport < hub->device_count;
	spin_unlock(&hub->lock);
	if (! ret)
		pr_err("invalid port number %d\n", rhport);
	return ret;
}

int usbredir_device_clear_port_feature(struct usbredir_hub *hub,
			       int rhport, u16 wValue)
{
	struct usbredir_device *dev;

	if (! valid_port(hub, rhport))
		return -EPIPE;

	spin_lock(&hub->lock);

	dev = hub->devices + rhport;
	spin_lock(&dev->lock);

	switch (wValue) {
	case USB_PORT_FEAT_SUSPEND:
		pr_debug(" ClearPortFeature: USB_PORT_FEAT_SUSPEND\n");
		if (dev->port_status & USB_PORT_STAT_SUSPEND) {
			/* 20msec signaling */
			/* TODO - figure out what this is about */
			hub->resuming = 1;
			hub->re_timeout =
				jiffies + msecs_to_jiffies(20);
		}
		break;
	case USB_PORT_FEAT_POWER:
		pr_debug(" ClearPortFeature: USB_PORT_FEAT_POWER\n");
		dev->port_status = 0;
		hub->resuming = 0;
		break;
	case USB_PORT_FEAT_C_RESET:
		pr_debug(" ClearPortFeature: USB_PORT_FEAT_C_RESET\n");
		// TODO - USB 3.0 stuff as well?
		switch (dev->connect_header.speed) {
		case usb_redir_speed_high:
			dev->port_status |= USB_PORT_STAT_HIGH_SPEED;
			break;
		case usb_redir_speed_low:
			dev->port_status |= USB_PORT_STAT_LOW_SPEED;
			break;
		default:
			break;
		}
	default:
		pr_debug(" ClearPortFeature: default %x\n", wValue);
		dev->port_status &= ~(1 << wValue);
		break;
	}

	spin_unlock(&dev->lock);
	spin_unlock(&hub->lock);

	return 0;
}

int usbredir_device_port_status(struct usbredir_hub *hub, int rhport, char *buf)
{
	struct usbredir_device *dev;
	if (! valid_port(hub, rhport))
		return -EPIPE;

	spin_lock(&hub->lock);

	dev = hub->devices + rhport;
	spin_lock(&dev->lock);


	/* TODO - read these comments and delete them or 
	 *   make sure JPW understands them */
	/* we do not care about resume. */

	/* whoever resets or resumes must GetPortStatus to
	 * complete it!!
	 */
	if (hub->resuming && time_after(jiffies, hub->re_timeout)) {
		dev->port_status |= (1 << USB_PORT_FEAT_C_SUSPEND);
		dev->port_status &= ~(1 << USB_PORT_FEAT_SUSPEND);
		hub->resuming = 0;
		hub->re_timeout = 0;
	}

	if ((dev->port_status & (1 << USB_PORT_FEAT_RESET)) != 0 &&
	     time_after(jiffies, hub->re_timeout)) {
		dev->port_status |= (1 << USB_PORT_FEAT_C_RESET);
		dev->port_status &= ~(1 << USB_PORT_FEAT_RESET);
		hub->re_timeout = 0;

		if (dev->status == VDEV_ST_NOTASSIGNED) {
			pr_debug(
				" enable rhport %d (status %u)\n",
				rhport,
				dev->status);
			dev->port_status |= USB_PORT_STAT_ENABLE;
		}
	}

	((__le16 *) buf)[0] = cpu_to_le16(dev->port_status);
	((__le16 *) buf)[1] =
		cpu_to_le16(dev->port_status >> 16);

	pr_debug(" GetPortStatus bye %x %x\n", ((u16 *)buf)[0],
			  ((u16 *)buf)[1]);

	spin_unlock(&dev->lock);
	spin_unlock(&hub->lock);
	return 0;
}

int usbredir_device_set_port_feature(struct usbredir_hub *hub,
			       int rhport, u16 wValue)
{
	struct usbredir_device *dev;
	if (! valid_port(hub, rhport))
		return -EPIPE;

	spin_lock(&hub->lock);

	dev = hub->devices + rhport;
	spin_lock(&dev->lock);
	switch (wValue) {
	case USB_PORT_FEAT_SUSPEND:
		pr_debug(
			" SetPortFeature: USB_PORT_FEAT_SUSPEND\n");
		break;
	case USB_PORT_FEAT_RESET:
		pr_debug(
			" SetPortFeature: USB_PORT_FEAT_RESET\n");
		dev->port_status &= ~USB_PORT_STAT_ENABLE;

		/* 50msec reset signaling */
		hub->re_timeout = jiffies + msecs_to_jiffies(50);

		/* FALLTHROUGH */
	default:
		pr_debug(" SetPortFeature: default %d\n", wValue);
		dev->port_status |= (1 << wValue);
		break;
	}

	spin_unlock(&dev->lock);
	spin_unlock(&hub->lock);
	return 0;
}
