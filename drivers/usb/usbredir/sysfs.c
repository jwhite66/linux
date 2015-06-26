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

#include <linux/file.h>
#include <linux/net.h>

#include "usbredir.h"


/* Sysfs entry to show port status */
static ssize_t status_show(struct device_driver *driver, char *out)
{
	return usbredir_hub_show_global_status(out);
}
static DRIVER_ATTR_RO(status);

/*
 * To start a new USBREDIR attachment, a user space program needs to setup a
 * connection and then write its socket descriptor along with a unique
 * identifer into the 'attach' sysfs file.
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
	 * @devid: user supplied unique device identifier
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


static ssize_t store_detach(struct device_driver *driver,
			    const char *buf, size_t count)
{
	char devid[256];
	struct usbredir_device *udev;

	/*
	 * @devid: user supplied unique device identifier
	 */
	memset(devid, 0, sizeof(devid));
	if (sscanf(buf, "%255s", devid) != 1)
		return -EINVAL;

	pr_debug("detach devid(%s)\n", devid);

	udev = usbredir_hub_find_device(devid);
	if (! udev) {
		pr_warning("USBREDIR device %s detach requested, but not found\n", devid);
		return count;
	}

	usbredir_device_disconnect(udev);
	usbredir_device_deallocate(udev, true, true);

	return count;
}
static DRIVER_ATTR(detach, S_IWUSR, NULL, store_detach);


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

