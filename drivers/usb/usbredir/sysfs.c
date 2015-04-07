/*
 * Copyright (C) 2015 Jeremy White based somewhat on USB/IP code from:
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

#include <linux/kthread.h>
#include <linux/file.h>
#include <linux/net.h>

#include "usbredirparser.h"
#include "usbredir.h"


/* Sysfs entry to show port status */
static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *out)
{
	char *s = out;

	BUG_ON(!the_controller || !out);

	spin_lock(&the_controller->lock);

	// TODO - implement a proper status
	out += sprintf(out, "JPW status\n");

	spin_unlock(&the_controller->lock);

	return out - s;
}
static DEVICE_ATTR_RO(status);

static ssize_t store_detach(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int err;
	char devid[256];
	__u32 rhport;

	memset(devid, 0, sizeof(devid));
	if (sscanf(buf, "%255s", devid) != 1)
		return -EINVAL;

	spin_lock(&the_controller->lock);

	vdev = find_port(devid);
	if (! vdev) {
		pr_err("not connected %d\n", vdev->status);
		goto error_unlock_controller;
	}

	spin_lock(&vdev->lock);
	if (vdev->status == VDEV_ST_NULL) {
		pr_err("not connected %d\n", vdev->status);

		spin_unlock(&vdev->lock);
		goto error_unlock_controller;
	}

	rhport = vdev->rhport;
	spin_unlock(&vdev->lock);

	usbredir_event_add(vdev, VDEV_EVENT_DOWN);
	err = port_disconnect(rhport);
	if (err < 0)
		return -EINVAL;

	return count;

error_unlock_controller:
	spin_unlock(&the_controller->lock);
	return -EINVAL;
}
static DEVICE_ATTR(detach, S_IWUSR, NULL, store_detach);


/*
 * To start a new USBREDIR attachment, a userland program needs to setup a
 * connection and then write its socket descriptor with remote device
 * information into this sysfs file.
 *
 */
static ssize_t store_attach(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct usbredir_device *vdev;
	struct socket *socket;
	int sockfd = 0;
	char devid[256];
	int err;
	__u32 rhport;
	uint32_t caps[USB_REDIR_CAPS_SIZE] = {
		usb_redir_cap_bulk_streams |
		usb_redir_cap_connect_device_version |
		usb_redir_cap_filter |
		usb_redir_cap_device_disconnect_ack |
		usb_redir_cap_ep_info_max_packet_size |
		usb_redir_cap_64bits_ids |
		usb_redir_cap_32bits_bulk_length |
		usb_redir_cap_bulk_receiving
	};

	/*
	 * @sockfd: socket descriptor of an established TCP connection
	 * @devid: unique device identifier in a remote host
	 */
	memset(devid, 0, sizeof(devid));
	if (sscanf(buf, "%u %255s", &sockfd, devid) != 2)
		return -EINVAL;

	pr_debug("sockfd(%u) devid(%s)\n", sockfd, devid);

	/* Extract socket from fd. */
	socket = sockfd_lookup(sockfd, &err);
	if (!socket)
		return -EINVAL;

	spin_lock(&the_controller->lock);
	if (find_port(devid)) {
		dev_err(dev, "%s: already in use\n", devid);
		goto error_free_socket_controller;
	}

	vdev = find_open_port();

	if (! vdev) {
		dev_err(dev, "%s: no port available\n", devid);
		goto error_free_socket_controller;
	}

	spin_lock(&vdev->lock);
	dev_info(dev, "sockfd(%d) devid(%s)\n", sockfd, devid);

	vdev->devid         = kstrdup(devid, GFP_KERNEL);
	vdev->status        = VDEV_ST_NOTASSIGNED;

	vdev->parser     = usbredirparser_create();
	usbredirparser_init(vdev->parser, "XXX TODO VERSION",
			    caps, USB_REDIR_CAPS_SIZE, /* flags */0);

	rhport = vdev->rhport;

	spin_unlock(&vdev->lock);
	spin_unlock(&the_controller->lock);

	vdev->rx = kthread_get_run(vhci_rx_loop, vdev, "vhci_rx");
	vdev->tx = kthread_get_run(vhci_tx_loop, vdev, "vhci_tx");

	rh_connect_port(rhport);

	return count;

error_free_socket_controller:
	spin_unlock(&the_controller->lock);
	sockfd_put(socket);
	return -EINVAL;
}
static DEVICE_ATTR(attach, S_IWUSR, NULL, store_attach);

static struct attribute *dev_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_detach.attr,
	&dev_attr_attach.attr,
	NULL,
};

const struct attribute_group dev_attr_group = {
	.attrs = dev_attrs,
};
