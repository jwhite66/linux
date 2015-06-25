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

#include <linux/file.h>
#include <linux/net.h>

#include "usbredir.h"


/* Sysfs entry to show port status */
static ssize_t status_show(struct device_driver *driver, char *out)
{
	char *s = out;
#if defined(HACK_TEMP_HACK)
	int i;
	struct usbredir_device *vdev;

	BUG_ON(!the_controller || !out);

	spin_lock(&the_controller->lock);

	out += sprintf(out,
		       "prt sta spd %40.40s %16s busid\n",
		       "devid", "socket");
	for (i = 0; i < USBREDIR_NPORTS; i++) {
		vdev = port_to_vdev(the_controller, i);
		spin_lock(&vdev->lock);
		out += sprintf(out, "%03u %03u ", i, vdev->status);

		if (vdev->status == VDEV_ST_USED) {
			out += sprintf(out, "%03u %40.40s ",
				       vdev->connect_header.speed, vdev->devid);
			out += sprintf(out, "%16p ", vdev->socket);
			out += sprintf(out, "%s", dev_name(&vdev->udev->dev));

		} else {
			out += sprintf(out, "000 "
				"---------------------------------------- "
				"0000000000000000 0-0");
		}

		out += sprintf(out, "\n");
		spin_unlock(&vdev->lock);
	}

	spin_unlock(&the_controller->lock);

#endif
	out += sprintf(out, "JPW hacking show\n");
	return out - s;
}
static DRIVER_ATTR_RO(status);

static ssize_t store_detach(struct device_driver *driver,
			    const char *buf, size_t count)
{
#if defined(HACK_TEMP_HACK)
	char devid[256];
	struct usbredir_device *vdev;

	memset(devid, 0, sizeof(devid));
	if (sscanf(buf, "%255s", devid) != 1)
		return -EINVAL;

	vdev = find_devid(the_controller, devid);
	if (! vdev) {
		pr_err("%s: not found\n", devid);
		return -EINVAL;
	}

	spin_lock(&the_controller->lock);
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
#endif
	return count;
}
static DRIVER_ATTR(detach, S_IWUSR, NULL, store_detach);


/*
 * To start a new USBREDIR attachment, a userland program needs to setup a
 * connection and then write its socket descriptor with remote device
 * information into this sysfs file.
 *
 */
static ssize_t store_attach(struct device_driver *driver,
			    const char *buf, size_t count)
{
	struct socket *socket;
	int sockfd = 0;
	char devid[256];
	int err;

	/*
	 * @sockfd: socket descriptor of an established TCP connection
	 * @devid: unique device identifier in a remote host
	 */
	memset(devid, 0, sizeof(devid));
	if (sscanf(buf, "%u %255s", &sockfd, devid) != 2)
		return -EINVAL;

	pr_debug("attach sockfd(%u) devid(%s)\n", sockfd, devid);

	/* Extract socket from fd. */
	socket = sockfd_lookup(sockfd, &err);
	if (!socket)
		return -EINVAL;

	if (usbredir_hub_find_device(devid)) {
		pr_err("%s: already in use\n", devid);
		sockfd_put(socket);
		return -EINVAL;
	}

	if (!usbredir_hub_allocate_device(devid, socket)) {
		pr_err("%s: unable to create\n", devid);
		sockfd_put(socket);
		return -EINVAL;
	}

	return count;
}
static DRIVER_ATTR(attach, S_IWUSR, NULL, store_attach);

int usbredir_sysfs_register(struct device_driver *driver)
{
	int ret;
	ret = driver_create_file(driver, &driver_attr_status);
	if (ret)
		return ret;

	ret = driver_create_file(driver, &driver_attr_detach);
	if (ret)
		return ret;

	return driver_create_file(driver, &driver_attr_attach);
}

void usbredir_sysfs_unregister(struct device_driver *dev)
{
	driver_remove_file(dev, &driver_attr_status);
	driver_remove_file(dev, &driver_attr_detach);
	driver_remove_file(dev, &driver_attr_attach);
}

