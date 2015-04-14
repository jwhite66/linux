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

#ifndef __USBIP_USBREDIR_H
#define __USBIP_USBREDIR_H

#include <linux/device.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/wait.h>

#include "usbredirparser.h"

#define kthread_get_run(threadfn, data, namefmt, ...)			   \
({									   \
	struct task_struct *__k						   \
		= kthread_create(threadfn, data, namefmt, ## __VA_ARGS__); \
	if (!IS_ERR(__k)) {						   \
		get_task_struct(__k);					   \
		wake_up_process(__k);					   \
	}								   \
	__k;								   \
})

#define kthread_stop_put(k)		\
	do {				\
		kthread_stop(k);	\
		put_task_struct(k);	\
	} while (0)


/* event handler */
#define USBREDIR_EH_SHUTDOWN	(1 << 0)
#define USBREDIR_EH_BYE		(1 << 1)
#define USBREDIR_EH_RESET	(1 << 2)
#define USBREDIR_EH_UNUSABLE	(1 << 3)

#define	VDEV_EVENT_REMOVED	(USBREDIR_EH_SHUTDOWN | USBREDIR_EH_BYE)
#define	VDEV_EVENT_DOWN		(USBREDIR_EH_SHUTDOWN | USBREDIR_EH_RESET)
#define	VDEV_EVENT_ERROR_TCP	(USBREDIR_EH_SHUTDOWN | USBREDIR_EH_RESET)
#define	VDEV_EVENT_ERROR_MALLOC	(USBREDIR_EH_SHUTDOWN | USBREDIR_EH_UNUSABLE)

/* JPW TODO - This should probably be over in include/uapi/linux */
/* usbredir device status - exported in device sysfs status */
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
	struct usb_device *udev;

	// TODO - understand when/why to use locks, and follow that rule
	spinlock_t lock;

	enum usbredir_device_status status;

	struct socket *socket;
        struct usbredirparser *parser;

	struct task_struct *rx;
	struct task_struct *tx;

	unsigned long event;
	struct task_struct *eh;
	wait_queue_head_t eh_waitq;

	struct eh_ops {
		void (*shutdown)(struct usbredir_device *);
		void (*reset)(struct usbredir_device *);
		void (*unusable)(struct usbredir_device *);
	} eh_ops;

	/*
	 * devid specifies a remote usb device uniquely
	 */
	char *devid;

	/* Store information transmitted by the remote side */
	struct usb_redir_device_connect_header connect_header;
	struct usb_redir_interface_info_header info_header;
	struct usb_redir_ep_info_header ep_info_header;

	/* root-hub port to which this device is attached */
	__u32 rhport;

	/* lock for the below link lists */
	spinlock_t priv_lock;

	/* usbredir_priv is linked to one of them. */
	// TODO - rename these?
	struct list_head priv_tx;
	struct list_head priv_rx;

	/* usbredir_unlink is linked to one of them */
	struct list_head unlink_tx;
	struct list_head unlink_rx;

	/* tx thread sleeps for this queue */
	wait_queue_head_t waitq_tx;
};

/* urb->hcpriv, use container_of() */
struct usbredir_priv {
	unsigned long seqnum;
	struct list_head list;

	struct usbredir_device *vdev;
	struct urb *urb;
};

struct usbredir_unlink {
	/* seqnum of this request */
	unsigned long seqnum;

	struct list_head list;

	/* seqnum of the unlink target */
	unsigned long unlink_seqnum;
};

/* Number of supported ports. Value has an upperbound of USB_MAXCHILDREN */
#define USBREDIR_NPORTS 8

/* for usb_bus.hcpriv */
struct usbredir_hcd {
	spinlock_t lock;

	u32 port_status[USBREDIR_NPORTS];

	unsigned resuming:1;
	unsigned long re_timeout;

	atomic_t seqnum;

	/*
	 * NOTE:
	 * wIndex shows the port number and begins from 1.
	 * But, the index of this array begins from 0.
	 */
	struct usbredir_device vdev[USBREDIR_NPORTS];
};

extern struct usbredir_hcd *the_controller;
extern const struct attribute_group hub_attr_group;

/* hcd .c */
void rh_port_connect(int rhport, enum usb_device_speed speed);
int id_to_port(const char *devid);

/* rx.c */
struct urb *pickup_urb_and_free_priv(struct usbredir_device *vdev, __u32 seqnum);
int rx_loop(void *data);

/* tx.c */
int tx_loop(void *data);

/* event.c */
int usbredir_start_eh(struct usbredir_device *ud);
void usbredir_stop_eh(struct usbredir_device *ud);
void usbredir_event_add(struct usbredir_device *ud, unsigned long event);
int usbredir_event_happened(struct usbredir_device *ud);

/* redir.c */
struct usbredirparser * redir_parser_init(void *priv);

static inline struct usbredir_device *port_to_vdev(__u32 port)
{
	return &the_controller->vdev[port];
}

static inline struct usbredir_hcd *hcd_to_usbredir(struct usb_hcd *hcd)
{
	return (struct usbredir_hcd *) (hcd->hcd_priv);
}

static inline struct usb_hcd *usbredir_to_hcd(struct usbredir_hcd *usbredir)
{
	return container_of((void *) usbredir, struct usb_hcd, hcd_priv);
}

static inline struct device *usbredir_dev(struct usbredir_hcd *usbredir)
{
	return usbredir_to_hcd(usbredir)->self.controller;
}


#endif /* __USBIP_USBREDIR_H */
