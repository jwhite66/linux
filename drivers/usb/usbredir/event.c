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

#include "usbredir.h"

static int event_handler(struct usbredir_device *vdev)
{
	/*
	 * Events are handled by only this thread.
	 */
	while (usbredir_event_happened(vdev)) {
		pr_debug("pending event %lx\n", vdev->event);

		/*
		 * NOTE: shutdown must come first.
		 * Shutdown the device.
		 */
		if (vdev->event & USBREDIR_EH_SHUTDOWN) {
			vdev->eh_ops.shutdown(vdev);
			vdev->event &= ~USBREDIR_EH_SHUTDOWN;
		}

		/* Reset the device. */
		if (vdev->event & USBREDIR_EH_RESET) {
			vdev->eh_ops.reset(vdev);
			vdev->event &= ~USBREDIR_EH_RESET;
		}

		/* Mark the device as unusable. */
		if (vdev->event & USBREDIR_EH_UNUSABLE) {
			vdev->eh_ops.unusable(vdev);
			vdev->event &= ~USBREDIR_EH_UNUSABLE;
		}

		/* Stop the error handler. */
		if (vdev->event & USBREDIR_EH_BYE)
			return -1;
	}

	return 0;
}

static int event_handler_loop(void *data)
{
	struct usbredir_device *vdev = data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(vdev->eh_waitq,
					 usbredir_event_happened(vdev) ||
					 kthread_should_stop());
		if (event_handler(vdev) < 0)
			break;
	}

	return 0;
}

int usbredir_start_eh(struct usbredir_device *vdev)
{
	char pname[32];
	init_waitqueue_head(&vdev->eh_waitq);
	vdev->event = 0;

	sprintf(pname, "usbredir/eh:%d", vdev->rhport);
	vdev->eh = kthread_run(event_handler_loop, vdev, pname);
	if (IS_ERR(vdev->eh)) {
		pr_warn("Unable to start control thread\n");
		return PTR_ERR(vdev->eh);
	}

	return 0;
}

void usbredir_stop_eh(struct usbredir_device *vdev)
{
	if (vdev->eh == current)
		return; /* do not wait for myself */

	kthread_stop(vdev->eh);
}

void usbredir_event_add(struct usbredir_device *vdev, unsigned long event)
{
	unsigned long flags;

	spin_lock_irqsave(&vdev->lock, flags);
	vdev->event |= event;
	wake_up(&vdev->eh_waitq);
	spin_unlock_irqrestore(&vdev->lock, flags);
}

int usbredir_event_happened(struct usbredir_device *vdev)
{
	int happened = 0;

	spin_lock(&vdev->lock);
	if (vdev->event != 0)
		happened = 1;
	spin_unlock(&vdev->lock);

	return happened;
}
