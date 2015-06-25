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

#include <linux/kthread.h>
#include <linux/slab.h>

#include "usbredir.h"

/* get URB from transmitted urb queue. */
struct urb *rx_pop_urb(struct usbredir_device *udev, int seqnum)
{
	struct usbredir_urb *uurb, *tmp;
	struct urb *urb = NULL;
	int status;

	spin_lock(&udev->lists_lock);

	list_for_each_entry_safe(uurb, tmp, &udev->urblist_rx, list) {
		if (uurb->seqnum != seqnum)
			continue;

		urb = uurb->urb;
		status = urb->status;

		pr_debug("usbredir: find urb %p vurb %p seqnum %u\n",
			 urb, uurb, seqnum);

		switch (status) {
		case -ENOENT:
			/* fall through */
		case -ECONNRESET:
			dev_info(&urb->dev->dev,
				 "urb %p was unlinked %ssynchronuously.\n", urb,
				 status == -ENOENT ? "" : "a");
			break;
		case -EINPROGRESS:
			/* no info output */
			break;
		default:
			dev_info(&urb->dev->dev,
				 "urb %p may be in a error, status %d\n", urb,
				 status);
		}

		list_del(&uurb->list);
		kfree(uurb);
		urb->hcpriv = NULL;

		break;
	}
	spin_unlock(&udev->lists_lock);

	return urb;
}

int rx_loop(void *data)
{
	struct usbredir_device *vdev = data;
	int rc;

	while (!kthread_should_stop()) {
		//if (usbredir_event_happened(vdev))
	        //		break;
		//		TODO: find a better way to see if we're done

		rc = usbredirparser_do_read(vdev->parser);
		if (rc != -EAGAIN) {
			pr_info("usbredir/rx:%d connection closed",
				vdev->rhport);
			//usbredir_event_add(vdev, VDEV_EVENT_DOWN);
			break;
		}
	}
	// TODO - signal that we're done in some way?

	return 0;
}
