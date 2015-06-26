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
 */

#include <linux/kthread.h>
#include <linux/slab.h>

#include "usbredir.h"

static struct usbredir_urb *get_next_cmd(struct usbredir_device *udev)
{
	struct usbredir_urb *uurb, *tmp;

	spin_lock(&udev->lists_lock);

	list_for_each_entry_safe(uurb, tmp, &udev->urblist_tx, list) {
		list_move_tail(&uurb->list, &udev->urblist_rx);
		spin_unlock(&udev->lists_lock);
		return uurb;
	}

	spin_unlock(&udev->lists_lock);

	return NULL;
}

static int send_cmd(struct usbredir_device *udev)
{
	struct usbredir_urb *uurb = NULL;

	size_t total_size = 0;

	/* TODO - lock? */

	while ((uurb = get_next_cmd(udev)) != NULL) {
		struct urb *urb = uurb->urb;
		__u8 type = usb_pipetype(urb->pipe);

		/*pr_debug("JPW urb: [pipe %x|type %d|stream_id %u|status %d|",
			urb->pipe, type, urb->stream_id, urb->status);
		pr_debug("tflags 0x%x|mapped sgs %d|num_sgs %d|tbuflen %u|",
			urb->transfer_flags, urb->num_mapped_sgs, urb->num_sgs,
			urb->transfer_buffer_length);
		pr_debug("complete %p|", urb->complete);
		pr_debug("pipedevice %x|", usb_pipedevice(urb->pipe));
		pr_debug("act len %u|st frame %d|num pack %d|int %d|err %d]\n",
			urb->actual_length, urb->start_frame,
			urb->number_of_packets, urb->interval,
			urb->error_count);*/

		if (type == PIPE_CONTROL && urb->setup_packet) {
			struct usb_ctrlrequest *ctrlreq =
				(struct usb_ctrlrequest *) urb->setup_packet;
			struct usb_redir_control_packet_header ctrl;

			ctrl.endpoint = usb_pipeendpoint(urb->pipe) |
					usb_pipein(urb->pipe);
			ctrl.request = ctrlreq->bRequest;
			ctrl.requesttype = ctrlreq->bRequestType;
			ctrl.status = 0;
			ctrl.value = le16_to_cpu(ctrlreq->wValue);
			ctrl.index = le16_to_cpu(ctrlreq->wIndex);
			ctrl.length = le16_to_cpu(ctrlreq->wLength);

			/*pr_debug("control request to endpoint 0x%x:\n",
				 ctrl.endpoint); */

			usbredirparser_send_control_packet(udev->parser,
				uurb->seqnum, &ctrl,
				usb_pipein(urb->pipe) ?
					NULL : urb->transfer_buffer,
				usb_pipein(urb->pipe) ?
					0 : urb->transfer_buffer_length);

		}

		if (type == PIPE_BULK) {
			struct usb_redir_bulk_packet_header bulk;

			bulk.endpoint = usb_pipeendpoint(urb->pipe) |
					usb_pipein(urb->pipe);
			bulk.status = 0;
			bulk.length = urb->transfer_buffer_length;
			bulk.stream_id = urb->stream_id;
			bulk.length_high = 0;

			/*pr_debug("bulk request to endpoint 0x%x:\n",
				 bulk.endpoint); */

			usbredirparser_send_bulk_packet(udev->parser,
				uurb->seqnum, &bulk,
				usb_pipein(urb->pipe) ?
					NULL : urb->transfer_buffer,
				usb_pipein(urb->pipe) ?
					0 : urb->transfer_buffer_length);
		}

	}

	return total_size;
}

static struct usbredir_unlink *get_next_unlink(struct usbredir_device *udev)
{
	struct usbredir_unlink *unlink, *tmp;

	spin_lock(&udev->lists_lock);

	list_for_each_entry_safe(unlink, tmp, &udev->unlink_tx, list) {
		list_move_tail(&unlink->list, &udev->unlink_rx);
		spin_unlock(&udev->lists_lock);
		return unlink;
	}

	spin_unlock(&udev->lists_lock);

	return NULL;
}

static int send_unlink(struct usbredir_device *udev)
{
	struct usbredir_unlink *unlink = NULL;

	size_t total_size = 0;

	while ((unlink = get_next_unlink(udev)) != NULL) {
		pr_debug("partially unimplemented: unlink request of ");
		pr_debug("seqnum %d, unlink seqnum %d\n",
			unlink->seqnum, unlink->unlink_seqnum);

		/* TODO - if the other side never responds, which it may
		        not do if the seqnum doesn't match, then we
		        never clear this entry.  That's probably not ideal */
		usbredirparser_send_cancel_data_packet(udev->parser,
						       unlink->unlink_seqnum);
	}

	return total_size;
}

void tx_urb(struct usbredir_device *udev, struct urb *urb)
{
	struct usbredir_urb *uurb;

	uurb = kzalloc(sizeof(struct usbredir_urb), GFP_ATOMIC);
	if (!uurb) {
		usbredir_device_disconnect(udev);
		usbredir_device_deallocate(udev, true, true);
		return;
	}

	spin_lock(&udev->lists_lock);

	uurb->seqnum = atomic_inc_return(&udev->hub->aseqnum);

	uurb->udev = udev;
	uurb->urb = urb;

	urb->hcpriv = (void *) uurb;

	list_add_tail(&uurb->list, &udev->urblist_tx);
	spin_unlock(&udev->lists_lock);

	wake_up_interruptible(&udev->waitq_tx);
}


int tx_loop(void *data)
{
	struct usbredir_device *udev = data;

	while (!kthread_should_stop() && atomic_read(&udev->active)) {
		if (usbredirparser_has_data_to_write(udev->parser)) {
			if (usbredirparser_do_write(udev->parser)) {
				/* TODO - need to think about this */
				break;
			}
		}

		if (send_cmd(udev) < 0)
			break;

		if (send_unlink(udev) < 0)
			break;

		wait_event_interruptible(udev->waitq_tx,
			 (!list_empty(&udev->urblist_tx) ||
			  !list_empty(&udev->unlink_tx) ||
			  kthread_should_stop() ||
			 usbredirparser_has_data_to_write(udev->parser)));
	}

	usbredir_device_disconnect(udev);
	usbredir_device_deallocate(udev, true, false);

	return 0;
}
