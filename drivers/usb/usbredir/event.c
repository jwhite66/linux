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

static int event_handler(struct usbredir_device *ud)
{
	pr_debug("enter\n");

	/*
	 * Events are handled by only this thread.
	 */
	while (usbredir_event_happened(ud)) {
		pr_debug("pending event %lx\n", ud->event);

		/*
		 * NOTE: shutdown must come first.
		 * Shutdown the device.
		 */
		if (ud->event & USBREDIR_EH_SHUTDOWN) {
			ud->eh_ops.shutdown(ud);
			ud->event &= ~USBREDIR_EH_SHUTDOWN;
		}

		/* Reset the device. */
		if (ud->event & USBREDIR_EH_RESET) {
			ud->eh_ops.reset(ud);
			ud->event &= ~USBREDIR_EH_RESET;
		}

		/* Mark the device as unusable. */
		if (ud->event & USBREDIR_EH_UNUSABLE) {
			ud->eh_ops.unusable(ud);
			ud->event &= ~USBREDIR_EH_UNUSABLE;
		}

		/* Stop the error handler. */
		if (ud->event & USBREDIR_EH_BYE)
			return -1;
	}

	return 0;
}

static int event_handler_loop(void *data)
{
	struct usbredir_device *ud = data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(ud->eh_waitq,
					 usbredir_event_happened(ud) ||
					 kthread_should_stop());
		pr_debug("wakeup\n");

		if (event_handler(ud) < 0)
			break;
	}

	return 0;
}

int usbredir_start_eh(struct usbredir_device *ud)
{
	init_waitqueue_head(&ud->eh_waitq);
	ud->event = 0;

	ud->eh = kthread_run(event_handler_loop, ud, "USBREDIR_EH");
	if (IS_ERR(ud->eh)) {
		pr_warn("Unable to start control thread\n");
		return PTR_ERR(ud->eh);
	}

	return 0;
}

void usbredir_stop_eh(struct usbredir_device *ud)
{
	if (ud->eh == current)
		return; /* do not wait for myself */

	kthread_stop(ud->eh);
	pr_debug("USBREDIR_EH has finished\n");
}

void usbredir_event_add(struct usbredir_device *ud, unsigned long event)
{
	unsigned long flags;

	spin_lock_irqsave(&ud->lock, flags);
	ud->event |= event;
	wake_up(&ud->eh_waitq);
	spin_unlock_irqrestore(&ud->lock, flags);
}

int usbredir_event_happened(struct usbredir_device *ud)
{
	int happened = 0;

	spin_lock(&ud->lock);
	if (ud->event != 0)
		happened = 1;
	spin_unlock(&ud->lock);

	return happened;
}
