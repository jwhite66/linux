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

#include <linux/kthread.h>
#include <linux/slab.h>

#include "usbredir.h"

static struct usbredir_priv *get_next_cmd(struct usbredir_device *vdev)
{
	struct usbredir_priv *priv, *tmp;

	spin_lock(&vdev->priv_lock);

	list_for_each_entry_safe(priv, tmp, &vdev->priv_tx, list) {
		list_move_tail(&priv->list, &vdev->priv_rx);
		spin_unlock(&vdev->priv_lock);
		return priv;
	}

	spin_unlock(&vdev->priv_lock);

	return NULL;
}

static int send_cmd(struct usbredir_device *vdev)
{
	struct usbredir_priv *priv = NULL;

	size_t total_size = 0;

	while ((priv = get_next_cmd(vdev)) != NULL) {
		struct urb *urb = priv->urb;
		__u8 type = usb_pipetype(urb->pipe);

		pr_debug("JPW urb: [pipe %x|type %d|stream_id %u|status %d|",
			urb->pipe, type, urb->stream_id, urb->status);
		pr_debug("tflags 0x%x|mapped sgs %d|num_sgs %d|tbuflen %u|",
			urb->transfer_flags, urb->num_mapped_sgs, urb->num_sgs,
			urb->transfer_buffer_length);
		pr_debug("complete %p|", urb->complete);
		pr_debug("pipedevice %x|", usb_pipedevice(urb->pipe));
		pr_debug("act len %u|st frame %d|num pack %d|int %d|err %d]\n",
			urb->actual_length, urb->start_frame,
			urb->number_of_packets, urb->interval,
			urb->error_count);

		if (type == PIPE_CONTROL && urb->setup_packet) {
			struct usb_ctrlrequest *ctrlreq =
				(struct usb_ctrlrequest *) urb->setup_packet;
			struct usb_redir_control_packet_header ctrl;

			pr_debug("control request:\n");
			print_hex_dump_bytes("", DUMP_PREFIX_NONE,
				     ctrlreq, sizeof(*ctrlreq));

			ctrl.endpoint = usb_pipein(urb->pipe) ?
				USB_DIR_IN : USB_DIR_OUT;
			ctrl.request = ctrlreq->bRequest;
			ctrl.requesttype = ctrlreq->bRequestType;
			ctrl.status = 0;
			ctrl.value = le16_to_cpu(ctrlreq->wValue);
			ctrl.index = le16_to_cpu(ctrlreq->wIndex);
			ctrl.length = le16_to_cpu(ctrlreq->wLength);

			usbredirparser_send_control_packet(vdev->parser,
				priv->seqnum, &ctrl, NULL, 0);
		}

		if (urb->transfer_buffer_length && urb->transfer_buffer)
			print_hex_dump_bytes("", DUMP_PREFIX_NONE,
					     urb->transfer_buffer,
					     urb->transfer_buffer_length);
	}

	return total_size;
}

static struct usbredir_unlink *get_next_unlink(struct usbredir_device *vdev)
{
	struct usbredir_unlink *unlink, *tmp;

	spin_lock(&vdev->priv_lock);

	list_for_each_entry_safe(unlink, tmp, &vdev->unlink_tx, list) {
		list_move_tail(&unlink->list, &vdev->unlink_rx);
		spin_unlock(&vdev->priv_lock);
		return unlink;
	}

	spin_unlock(&vdev->priv_lock);

	return NULL;
}

static int send_unlink(struct usbredir_device *vdev)
{
	struct usbredir_unlink *unlink = NULL;

	size_t total_size = 0;

	while ((unlink = get_next_unlink(vdev)) != NULL) {
		pr_debug("partially unimplemented: unlink request of "
			 "seqnum %d, unlink seqnum %d\n",
			unlink->seqnum, unlink->unlink_seqnum);

		// TODO - if the other side never responds, which it may
		//        not do if the seqnum doesn't match, then we
		//        never clear this entry.  That's probably not ideal
		usbredirparser_send_cancel_data_packet(vdev->parser,
						       unlink->unlink_seqnum);
	}

	return total_size;
}


int tx_loop(void *data)
{
	struct usbredir_device *vdev = data;

	while (!kthread_should_stop()) {
		if (usbredirparser_has_data_to_write(vdev->parser)) {
			if (usbredirparser_do_write(vdev->parser)) {
				// TODO - need to think about this
				break;
			}
		}

		if (send_cmd(vdev) < 0)
			break;

		if (send_unlink(vdev) < 0)
			break;

		wait_event_interruptible(vdev->waitq_tx,
			 (!list_empty(&vdev->priv_tx) ||
			  !list_empty(&vdev->unlink_tx) ||
			  kthread_should_stop() ||
			 usbredirparser_has_data_to_write(vdev->parser)));
	}

	return 0;
}
