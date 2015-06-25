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


/* TODO - This should probably be over in include/uapi/linux */
/* usbredir device status - exported in device sysfs status */
/* TODO - why this status, and not just reuse port_status */
enum usbredir_device_status {
	/* vdev does not connect a remote device. */
	VDEV_ST_NULL,
	/* vdev is used, but the USB address is not assigned yet */
	VDEV_ST_NOTASSIGNED,
	/* vdev is used */
	VDEV_ST_USED,
	VDEV_ST_ERROR
};

struct usbredir_device {
	struct usb_device *usb_dev;
	struct usbredir_hub *hub;

	// TODO - a thoughtful look into the locks is overdue
	spinlock_t lock;

	enum usbredir_device_status status;
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


/* device.c */
void usbredir_device_init(struct usbredir_device *udev, int port);
void usbredir_device_destroy(struct usbredir_device *udev);
void usbredir_device_connect(struct usbredir_device *udev);
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


#if defined(HACK_AND_SLASH)

/* event handler */
#define USBREDIR_EH_SHUTDOWN	(1 << 0)
#define USBREDIR_EH_BYE		(1 << 1)
#define USBREDIR_EH_RESET	(1 << 2)
#define USBREDIR_EH_UNUSABLE	(1 << 3)

#define	VDEV_EVENT_REMOVED	(USBREDIR_EH_SHUTDOWN | USBREDIR_EH_BYE)
#define	VDEV_EVENT_DOWN		(USBREDIR_EH_SHUTDOWN | USBREDIR_EH_RESET)
#define	VDEV_EVENT_ERROR_TCP	(USBREDIR_EH_SHUTDOWN | USBREDIR_EH_RESET)
#define	VDEV_EVENT_ERROR_MALLOC	(USBREDIR_EH_SHUTDOWN | USBREDIR_EH_UNUSABLE)


/* Number of supported ports. Value has an upperbound of USB_MAXCHILDREN */
#define USBREDIR_NPORTS 8

struct usbredir_hcd {
	spinlock_t lock;

	u32 port_status[USBREDIR_NPORTS];

	unsigned resuming:1;
	unsigned long re_timeout;

	atomic_t aseqnum;

	/*
	 * NOTE:
	 * wIndex shows the port number and begins from 1.
	 * But, the index of this array begins from 0.
	 */
	struct usbredir_device vdev[USBREDIR_NPORTS];
};

/* hcd .c */
void hcd_connect_port(struct usbredir_device *vdev);
struct usbredir_device *find_devid(struct usbredir_hcd *uhcd,
				   const char *devid);

/* rx.c */
struct urb *pickup_urb_and_free_priv(struct usbredir_device *vdev, int seqnum);
int rx_loop(void *data);

/* tx.c */
void tx_urb(struct urb *urb);
int tx_loop(void *data);

/* event.c */
int usbredir_start_eh(struct usbredir_device *vdev);
void usbredir_stop_eh(struct usbredir_device *vdev);
void usbredir_event_add(struct usbredir_device *vdev, unsigned long event);
int usbredir_event_happened(struct usbredir_device *vdev);

/* redir.c */
struct usbredirparser * redir_parser_init(void *priv);

static inline struct usbredir_device *port_to_vdev(struct usbredir_hcd *uhcd,
						   __u32 port)
{
	return &uhcd->vdev[port];
}

static inline struct usb_hcd *usbredir_to_hcd(struct usbredir_hcd *usbredir)
{
	return container_of((void *) usbredir, struct usb_hcd, hcd_priv);
}

static inline struct device *usbredir_dev(struct usbredir_hcd *usbredir)
{
	return usbredir_to_hcd(usbredir)->self.controller;
}

static inline struct usbredir_device *udev_to_usbredir(struct usb_device *udev)
{
	int i;

	if (!udev)
		return NULL;

	for (i = 0; i < USBREDIR_NPORTS; i++)
		if (the_controller->vdev[i].udev == udev)
			return &the_controller->vdev[i];

	return NULL;
}

#endif

#endif /* __USBREDIR_H */
