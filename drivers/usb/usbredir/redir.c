/*
 * Copyright (C) 2015 Jeremy White
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


#include <linux/net.h>
#include <linux/kthread.h>
#include <net/sock.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/printk.h>

#include "usbredirparser.h"
#include "usbredir.h"


#define TODO_IMPLEMENT pr_err("Error: %s unimplemented.", __FUNCTION__);

static void redir_log(void *priv, int level, const char *msg)
{
	switch (level) {
	case usbredirparser_error:
		pr_err("%s", msg);
		break;

	case usbredirparser_warning:
		pr_warn("%s", msg);
		break;

	case usbredirparser_info:
		pr_info("%s", msg);
		break;

	default:
		pr_debug("%s", msg);
		break;
	}
}

static int redir_read(void *priv, uint8_t *data, int count)
{
	struct usbredir_device *vdev = (struct usbredir_device *) priv;
	struct msghdr msg;
	struct kvec iov;
	int rc;

	if (kthread_should_stop() || usbredir_event_happened(vdev))
		return -ESRCH;

	vdev->socket->sk->sk_allocation = GFP_NOIO;
	iov.iov_base    = data;
	iov.iov_len     = count;
	msg.msg_name    = NULL;
	msg.msg_namelen = 0;
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	msg.msg_flags = MSG_NOSIGNAL;

pr_debug("JPW enter read, max %d\n", count);
	rc = kernel_recvmsg(vdev->socket, &msg, &iov, 1, count, MSG_WAITALL);
	if (rc > 0) {
pr_debug("JPW read %d\n", rc);
		print_hex_dump_bytes("", DUMP_PREFIX_NONE, data, rc);
	}

	return rc;
}

static int redir_write(void *priv, uint8_t *data, int count)
{
	struct usbredir_device *vdev = (struct usbredir_device *) priv;
	struct msghdr msg;
	struct kvec iov;
	int rc;

	memset(&msg, 0, sizeof(msg));
	memset(&iov, 0, sizeof(iov));
	msg.msg_flags = MSG_NOSIGNAL | MSG_DONTWAIT;
	iov.iov_base = data;
	iov.iov_len  = count;
pr_debug("JPW writing %d\n", count);
	rc = kernel_sendmsg(vdev->socket, &msg, &iov, 1, count);
pr_debug("JPW wrote %d\n", rc);
	if (rc > 0) {
		print_hex_dump_bytes("redir_write", DUMP_PREFIX_NONE, data, rc);
	}
	if (rc != count) {
		pr_err("Error %d writing %d bytes\n", rc, count);
		usbredir_event_add(vdev, VDEV_EVENT_ERROR_TCP);
		return -1;
	}

	return rc;
}


/* Locking functions for use by multithread apps */
static void *redir_alloc_lock(void)
{
	struct semaphore *s = kmalloc(sizeof(*s), GFP_KERNEL);
	sema_init(s, 1);
	return s;
}

static void redir_lock(void *lock)
{
	while(down_interruptible((struct semaphore *) lock))
		;
}

static void redir_unlock(void *lock)
{
	up((struct semaphore *) lock);
}

static void redir_free_lock(void *lock)
{
	kfree(lock);
}


/* The below callbacks are called when a complete packet of the relevant
   type has been received.

   Note that the passed in packet-type-specific-header's lifetime is only
   guarenteed to be that of the callback.

   Control packets: */
static void redir_hello(void *priv,
    struct usb_redir_hello_header *hello)
{
	pr_debug("Hello!\n");
}

static enum usb_device_speed convert_speed(int speed)
{
	switch(speed) {
	case usb_redir_speed_low:   return USB_SPEED_LOW;
	case usb_redir_speed_full:  return USB_SPEED_FULL;
	case usb_redir_speed_high:  return USB_SPEED_HIGH;
	case usb_redir_speed_super: return USB_SPEED_SUPER;
	default:		    return USB_SPEED_UNKNOWN;
	}
}

static void redir_device_connect(void *priv,
    struct usb_redir_device_connect_header *device_connect)
{
	struct usbredir_device *vdev = (struct usbredir_device *) priv;
	// TODO: lock?
	vdev->connect_header = *device_connect;

	pr_debug("  class %2d subclass %2d protocol %2d",
           device_connect->device_class, device_connect->device_subclass,
           device_connect->device_protocol);
	pr_debug("  vendor 0x%04x product %04x\n",
           device_connect->vendor_id, device_connect->product_id);

	rh_port_connect(vdev->rhport, convert_speed(device_connect->speed));
}

static void redir_device_disconnect(void *priv)
{
	TODO_IMPLEMENT;
}

static void redir_reset(void *priv)
{
	TODO_IMPLEMENT;
}

static void redir_interface_info(void *priv,
    struct usb_redir_interface_info_header *info)
{
	struct usbredir_device *vdev = (struct usbredir_device *) priv;
	int i;

	vdev->info_header = *info;
	// TODO: lock?
	for (i = 0; i < info->interface_count; i++) {
		pr_debug("interface %d class %2d subclass %2d protocol %2d",
			info->interface[i], info->interface_class[i],
			info->interface_subclass[i], info->interface_protocol[i]);
	}
}

/* Macros to go from an endpoint address to an index for our ep array */
#define EP2I(ep_address) (((ep_address & 0x80) >> 3) | (ep_address & 0x0f))
#define I2EP(i) (((i & 0x10) << 3) | (i & 0x0f))

static void redir_ep_info(void *priv,
    struct usb_redir_ep_info_header *ep_info)
{
	struct usbredir_device *vdev = (struct usbredir_device *) priv;
	int i;

	// TODO - lock?
	vdev->ep_info_header = *ep_info;
	for (i = 0; i < 32; i++) {
		if (ep_info->type[i] != usb_redir_type_invalid) {
			pr_debug("endpoint: %02X, type: %d, interval: %d, interface: %d",
				I2EP(i), (int)ep_info->type[i], (int)ep_info->interval[i],
				(int)ep_info->interface[i]);
		}
	}
}

static void redir_set_configuration(void *priv,
    uint64_t id, struct usb_redir_set_configuration_header *set_configuration)
{
	TODO_IMPLEMENT;
}

static void redir_get_configuration(void *priv, uint64_t id)
{
	TODO_IMPLEMENT;
}

static void redir_configuration_status(void *priv,
    uint64_t id, struct usb_redir_configuration_status_header *configuration_status)
{
	TODO_IMPLEMENT;
}

static void redir_set_alt_setting(void *priv,
    uint64_t id, struct usb_redir_set_alt_setting_header *set_alt_setting)
{
	TODO_IMPLEMENT;
}

static void redir_get_alt_setting(void *priv,
    uint64_t id, struct usb_redir_get_alt_setting_header *get_alt_setting)
{
	TODO_IMPLEMENT;
}

static void redir_alt_setting_status(void *priv,
    uint64_t id, struct usb_redir_alt_setting_status_header *alt_setting_status)
{
	TODO_IMPLEMENT;
}

static void redir_start_iso_stream(void *priv,
    uint64_t id, struct usb_redir_start_iso_stream_header *start_iso_stream)
{
	TODO_IMPLEMENT;
}

static void redir_stop_iso_stream(void *priv,
    uint64_t id, struct usb_redir_stop_iso_stream_header *stop_iso_stream)
{
	TODO_IMPLEMENT;
}

static void redir_iso_stream_status(void *priv,
    uint64_t id, struct usb_redir_iso_stream_status_header *iso_stream_status)
{
	TODO_IMPLEMENT;
}

static void redir_start_interrupt_receiving(void *priv,
    uint64_t id, struct usb_redir_start_interrupt_receiving_header *start_interrupt_receiving)
{
	TODO_IMPLEMENT;
}

static void redir_stop_interrupt_receiving(void *priv,
    uint64_t id, struct usb_redir_stop_interrupt_receiving_header *stop_interrupt_receiving)
{
	TODO_IMPLEMENT;
}

static void redir_interrupt_receiving_status(void *priv,
    uint64_t id, struct usb_redir_interrupt_receiving_status_header *interrupt_receiving_status)
{
	TODO_IMPLEMENT;
}

static void redir_alloc_bulk_streams(void *priv,
    uint64_t id, struct usb_redir_alloc_bulk_streams_header *alloc_bulk_streams)
{
	TODO_IMPLEMENT;
}

static void redir_free_bulk_streams(void *priv,
    uint64_t id, struct usb_redir_free_bulk_streams_header *free_bulk_streams)
{
	TODO_IMPLEMENT;
}

static void redir_bulk_streams_status(void *priv,
    uint64_t id, struct usb_redir_bulk_streams_status_header *bulk_streams_status)
{
	TODO_IMPLEMENT;
}

static void redir_cancel_data_packet(void *priv, uint64_t id)
{
	TODO_IMPLEMENT;
}

static void redir_filter_reject(void *priv)
{
	TODO_IMPLEMENT;
}

static void redir_filter_filter(void *priv,
    struct usbredirfilter_rule *rules, int rules_count)
{
	TODO_IMPLEMENT;
}

static void redir_device_disconnect_ack(void *priv)
{
	TODO_IMPLEMENT;
}

static void redir_start_bulk_receiving(void *priv,
    uint64_t id, struct usb_redir_start_bulk_receiving_header *start_bulk_receiving)
{
	TODO_IMPLEMENT;
}

static void redir_stop_bulk_receiving(void *priv,
    uint64_t id, struct usb_redir_stop_bulk_receiving_header *stop_bulk_receiving)
{
	TODO_IMPLEMENT;
}

static void redir_bulk_receiving_status(void *priv,
    uint64_t id, struct usb_redir_bulk_receiving_status_header *bulk_receiving_status)
{
	TODO_IMPLEMENT;
}


static void redir_control_packet(void *priv,
    uint64_t id, struct usb_redir_control_packet_header *control_header,
    uint8_t *data, int data_len)
{
	struct usbredir_device *vdev = (struct usbredir_device *) priv;
	struct urb *urb;

	spin_lock(&vdev->priv_lock);
	urb = pickup_urb_and_free_priv(vdev, id);
	spin_unlock(&vdev->priv_lock);
	if (!urb) {
		pr_err("Error: control id %lu received with no matching"
		       " entry.\n",  (unsigned long) id);
		return;
	}

pr_debug("JPW handling control packet response, id %ld\n", (long) id);
pr_debug("tbuf len %d, data length %d:\n", urb->transfer_buffer_length, data_len);
print_hex_dump_bytes("", DUMP_PREFIX_NONE, data, data_len);

	// TODO - map statii correctly
	urb->status = control_header->status;
	urb->actual_length = min((u32) data_len, urb->transfer_buffer_length);

	// TODO - handle more than this flavor...
	if (urb->transfer_buffer)
		memcpy(urb->transfer_buffer, data, urb->actual_length);

	spin_lock(&the_controller->lock);
	usb_hcd_unlink_urb_from_ep(usbredir_to_hcd(the_controller), urb);
	spin_unlock(&the_controller->lock);

	usb_hcd_giveback_urb(usbredir_to_hcd(the_controller), urb, urb->status);
}

static void redir_bulk_packet(void *priv,
    uint64_t id, struct usb_redir_bulk_packet_header *bulk_header,
    uint8_t *data, int data_len)
{
	TODO_IMPLEMENT;
}

static void redir_iso_packet(void *priv,
    uint64_t id, struct usb_redir_iso_packet_header *iso_header,
    uint8_t *data, int data_len)
{
	TODO_IMPLEMENT;
}

static void redir_interrupt_packet(void *priv,
    uint64_t id, struct usb_redir_interrupt_packet_header *interrupt_header,
    uint8_t *data, int data_len)
{
	TODO_IMPLEMENT;
}

static void redir_buffered_bulk_packet(void *priv, uint64_t id,
    struct usb_redir_buffered_bulk_packet_header *buffered_bulk_header,
    uint8_t *data, int data_len)
{
	TODO_IMPLEMENT;
}


struct usbredirparser * redir_parser_init(void *priv)
{
	struct usbredirparser *parser;
	char version[40];

	uint32_t caps[USB_REDIR_CAPS_SIZE];

	parser = usbredirparser_create();

	parser->priv = priv;

	parser->log_func = redir_log;
	parser->read_func = redir_read;
	parser->write_func = redir_write;
	parser->device_connect_func = redir_device_connect;
	parser->device_disconnect_func = redir_device_disconnect;
	parser->reset_func = redir_reset;
	parser->interface_info_func = redir_interface_info;
	parser->ep_info_func = redir_ep_info;
	parser->set_configuration_func = redir_set_configuration;
	parser->get_configuration_func = redir_get_configuration;
	parser->configuration_status_func = redir_configuration_status;
	parser->set_alt_setting_func = redir_set_alt_setting;
	parser->get_alt_setting_func = redir_get_alt_setting;
	parser->alt_setting_status_func = redir_alt_setting_status;
	parser->start_iso_stream_func = redir_start_iso_stream;
	parser->stop_iso_stream_func = redir_stop_iso_stream;
	parser->iso_stream_status_func = redir_iso_stream_status;
	parser->start_interrupt_receiving_func =
		redir_start_interrupt_receiving;
	parser->stop_interrupt_receiving_func = redir_stop_interrupt_receiving;
	parser->interrupt_receiving_status_func =
		redir_interrupt_receiving_status;
	parser->alloc_bulk_streams_func = redir_alloc_bulk_streams;
	parser->free_bulk_streams_func = redir_free_bulk_streams;
	parser->bulk_streams_status_func = redir_bulk_streams_status;
	parser->cancel_data_packet_func = redir_cancel_data_packet;
	parser->control_packet_func = redir_control_packet;
	parser->bulk_packet_func = redir_bulk_packet;
	parser->iso_packet_func = redir_iso_packet;
	parser->interrupt_packet_func = redir_interrupt_packet;
	parser->alloc_lock_func = redir_alloc_lock;
	parser->lock_func = redir_lock;
	parser->unlock_func = redir_unlock;
	parser->free_lock_func = redir_free_lock;
	parser->hello_func = redir_hello;
	parser->filter_reject_func = redir_filter_reject;
	parser->filter_filter_func = redir_filter_filter;
	parser->device_disconnect_ack_func = redir_device_disconnect_ack;
	parser->start_bulk_receiving_func = redir_start_bulk_receiving;
	parser->stop_bulk_receiving_func = redir_stop_bulk_receiving;
	parser->bulk_receiving_status_func = redir_bulk_receiving_status;
	parser->buffered_bulk_packet_func = redir_buffered_bulk_packet;

	memset(caps, 0, sizeof(caps));
#if defined(USE_ALL_CAPS)
	usbredirparser_caps_set_cap(caps, usb_redir_cap_bulk_streams);
	usbredirparser_caps_set_cap(caps, usb_redir_cap_connect_device_version);
	usbredirparser_caps_set_cap(caps, usb_redir_cap_filter);
	usbredirparser_caps_set_cap(caps, usb_redir_cap_device_disconnect_ack);
	usbredirparser_caps_set_cap(caps, usb_redir_cap_ep_info_max_packet_size);
	usbredirparser_caps_set_cap(caps, usb_redir_cap_64bits_ids);
	usbredirparser_caps_set_cap(caps, usb_redir_cap_32bits_bulk_length);
	usbredirparser_caps_set_cap(caps, usb_redir_cap_bulk_receiving);
#endif

	sprintf(version, "kmodule v%s. Protocol %x",
		USBREDIR_MODULE_VERSION, USBREDIR_VERSION);
	usbredirparser_init(parser, version, caps, USB_REDIR_CAPS_SIZE, 0);

	return parser;
}

