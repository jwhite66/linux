/*
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

int vhci_tx_loop(void *data)
{
	struct usbredir_device *vdev = data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(vdev->waitq_tx,
					 (!list_empty(&vdev->priv_tx) ||
					  !list_empty(&vdev->unlink_tx) ||
					  kthread_should_stop()));

		pr_debug("pending urbs ?, now wake up\n");
	}

	return 0;
}
