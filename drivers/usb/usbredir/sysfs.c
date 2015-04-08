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
	int i;
	struct usbredir_device *vdev;

	BUG_ON(!the_controller || !out);

	spin_lock(&the_controller->lock);

	out += sprintf(out,
		       "prt sta spd %30.30s %16s local_busid\n",
		       "devid", "socket");
	for (i = 0; i < USBREDIR_NPORTS; i++) {
		vdev = port_to_vdev(i);
		spin_lock(&vdev->lock);
		out += sprintf(out, "%03u %03u ", i, vdev->status);

		if (vdev->status == VDEV_ST_USED) {
			out += sprintf(out, "%03u %40.40s ",
				       vdev->connect_header.speed, vdev->devid);
			out += sprintf(out, "%16p ", vdev->socket);
			out += sprintf(out, "%s", dev_name(&vdev->udev->dev));

		} else {
			out += sprintf(out, "000 ------------------------------- 0000000000000000 0-0");
		}

		out += sprintf(out, "\n");
		spin_unlock(&vdev->lock);
	}

	spin_unlock(&the_controller->lock);

	return out - s;
}
static DEVICE_ATTR_RO(status);

static ssize_t store_detach(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	char devid[256];
	__u32 rhport;
	struct usbredir_device *vdev;

	memset(devid, 0, sizeof(devid));
	if (sscanf(buf, "%255s", devid) != 1)
		return -EINVAL;

	rhport = id_to_port(devid);
	if (rhport < 0) {
		pr_err("%s: not found\n", devid);
		return -EINVAL;
	}

	spin_lock(&the_controller->lock);
	vdev = port_to_vdev(rhport);
	spin_lock(&vdev->lock);
	if (vdev->status == VDEV_ST_NULL) {
		pr_err("%s: not connected %d\n", devid, vdev->status);

		spin_unlock(&vdev->lock);
		spin_unlock(&the_controller->lock);
		return -EINVAL;
	}

	spin_unlock(&vdev->lock);
	spin_unlock(&the_controller->lock);

	usbredir_event_add(vdev, VDEV_EVENT_DOWN);

	return count;
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
	__u32 rhport = -1;
	int i;

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

	if (id_to_port(devid) >= 0) {
		dev_err(dev, "%s: already in use\n", devid);
		sockfd_put(socket);
		return -EINVAL;
	}

	spin_lock(&the_controller->lock);
	for (i = 0; i < USBREDIR_NPORTS; i++) {
		vdev = port_to_vdev(i);
		spin_lock(&vdev->lock);
		if (vdev->status == VDEV_ST_NULL) {
			rhport = i;
			break;
		}
		spin_unlock(&vdev->lock);
	}

	if (rhport < 0) {
		dev_err(dev, "%s: no port available\n", devid);
		spin_unlock(&the_controller->lock);
		sockfd_put(socket);
		return -EINVAL;
	}

	dev_info(dev, "sockfd(%d) devid(%s)\n", sockfd, devid);

	vdev->devid  = kstrdup(devid, GFP_KERNEL);
	vdev->status = VDEV_ST_NOTASSIGNED;
	vdev->socket = socket;

	vdev->parser = redir_parser_init(vdev);

	vdev->rx = kthread_get_run(vhci_rx_loop, vdev, "vhci_rx");
	vdev->tx = kthread_get_run(vhci_tx_loop, vdev, "vhci_tx");

	spin_unlock(&vdev->lock);
	spin_unlock(&the_controller->lock);

	return count;
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
