/*
 * Copyright (C) 2015 Jeremy White based on work by
 * Copyright (C) 2003-2008 Takahiro Hirofuchi
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __USBREDIR_H
#define __USBREDIR_H

#include <linux/device.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "usbredirparser.h"

#define USBREDIR_MODULE_VERSION	"1.0"


struct usbredir_device {
	// TODO - a thoughtful look into the locks is overdue
	spinlock_t lock;

	atomic_t active;

	struct usb_device *usb_dev;
	struct usbredir_hub *hub;

	u32 port_status;

	struct socket *socket;
        struct usbredirparser *parser;

	struct task_struct *rx;
	struct task_struct *tx;

	/*
	 * devid specifies a remote usb device; user space
	 *  is responsible for seeing that they are unique
	 */
	char *devid;

	/* Store information transmitted by the remote side */
	struct usb_redir_device_connect_header connect_header;
	struct usb_redir_interface_info_header info_header;
	struct usb_redir_ep_info_header ep_info_header;

	/* root-hub port to which this device is attached */
	__u32 rhport;

	/* lock for the below link lists */
	// TODO - Do away with in favor of a single lock?
	spinlock_t lists_lock;

	/* list of types usbredir_urb */
	struct list_head urblist_tx;
	struct list_head urblist_rx;

	/* list of types usbredir_unlink */
	struct list_head unlink_tx;
	struct list_head unlink_rx;

	/* tx thread sleeps for this queue */
	wait_queue_head_t waitq_tx;
};

/* Structure to hold a USB hub */
struct usbredir_hub {
	struct list_head	list;
	int			id;
	struct platform_device	pdev;
	struct usb_hcd		*hcd;

	spinlock_t lock;

	atomic_t aseqnum;

	unsigned resuming:1;
	unsigned long re_timeout;

	int			device_count;
	struct usbredir_device *devices;
};

/* Structure to hold a urb as we process it */
struct usbredir_urb {
	int seqnum;
	struct list_head list;

	struct usbredir_device *udev;
	struct urb *urb;
};

/* Structure to hold an unlink request as we process it */
struct usbredir_unlink {
	/* seqnum of this request */
	int seqnum;

	struct list_head list;

	/* seqnum of the unlink target */
	int unlink_seqnum;
};


/* main.c */
extern unsigned int max_hubs;
extern unsigned int devices_per_hub;

extern const char driver_name[];
extern const char driver_desc[];

/* sysfs.c */
int usbredir_sysfs_register(struct device_driver *dev);
void usbredir_sysfs_unregister(struct device_driver *dev);

/* hub.c */
int usbredir_hub_init(void);
void usbredir_hub_exit(void);
struct usbredir_hub *usbredir_hub_create(void);
void usbredir_hub_destroy(struct usbredir_hub *hub);
struct usbredir_device *usbredir_hub_allocate_device(const char *devid,
						     struct socket *socket);
struct usbredir_device *usbredir_hub_find_device(const char *devid);
int usbredir_hub_show_global_status(char *out);


/* device.c */
void usbredir_device_init(struct usbredir_device *udev, int port,
			  struct usbredir_hub *hub);
void usbredir_device_allocate(struct usbredir_device *udev,
			      const char *devid,
			      struct socket *socket);
void usbredir_device_deallocate(struct usbredir_device *udev,
				bool stop, bool stoptx);
void usbredir_device_connect(struct usbredir_device *udev);
void usbredir_device_disconnect(struct usbredir_device *udev);
int usbredir_device_clear_port_feature(struct usbredir_hub *hub,
			       int rhport, u16 wValue);
int usbredir_device_port_status(struct usbredir_hub *hub, int rhport,
				char *buf);
int usbredir_device_set_port_feature(struct usbredir_hub *hub,
			       int rhport, u16 wValue);

/* redir.c */
struct usbredirparser * redir_parser_init(void *priv);

/* rx.c */
struct urb *rx_pop_urb(struct usbredir_device *udev, int seqnum);
int rx_loop(void *data);

/* tx.c */
void tx_urb(struct usbredir_device *udev, struct urb *urb);
int tx_loop(void *data);

/* urb.c */
int urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
			    gfp_t mem_flags);
int urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status);

/* Fast lookup functions */
static inline struct usbredir_hub *usbredir_hub_from_hcd(struct usb_hcd *hcd)
{
	return * (struct usbredir_hub **) hcd->hcd_priv;
}

#endif /* __USBREDIR_H */
