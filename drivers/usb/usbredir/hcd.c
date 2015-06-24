#if defined(HACK_AND_SLASH)

static const char * const bit_desc[] = {
	"CONNECTION",		/*0*/
	"ENABLE",		/*1*/
	"SUSPEND",		/*2*/
	"OVER_CURRENT",		/*3*/
	"RESET",		/*4*/
	"R5",			/*5*/
	"R6",			/*6*/
	"R7",			/*7*/
	"POWER",		/*8*/
	"LOWSPEED",		/*9*/
	"HIGHSPEED",		/*10*/
	"PORT_TEST",		/*11*/
	"INDICATOR",		/*12*/
	"R13",			/*13*/
	"R14",			/*14*/
	"R15",			/*15*/
	"C_CONNECTION",		/*16*/
	"C_ENABLE",		/*17*/
	"C_SUSPEND",		/*18*/
	"C_OVER_CURRENT",	/*19*/
	"C_RESET",		/*20*/
	"R21",			/*21*/
	"R22",			/*22*/
	"R23",			/*23*/
	"R24",			/*24*/
	"R25",			/*25*/
	"R26",			/*26*/
	"R27",			/*27*/
	"R28",			/*28*/
	"R29",			/*29*/
	"R30",			/*30*/
	"R31",			/*31*/
};

static void dump_port_status_diff(u32 prev_status, u32 new_status)
{
	int i = 0;
	u32 bit = 1;

	pr_debug("status prev -> new: %08x -> %08x\n", prev_status, new_status);
	while (bit) {
		u32 prev = prev_status & bit;
		u32 new = new_status & bit;
		char change;

		if (!prev && new)
			change = '+';
		else if (prev && !new)
			change = '-';
		else
			change = ' ';

		if (prev || new)
			pr_debug(" %c%s\n", change, bit_desc[i]);
		bit <<= 1;
		i++;
	}
	pr_debug("\n");
}


static u32 speed_to_portflag(enum usb_device_speed speed)
{
	switch(speed) {
	case usb_redir_speed_low:   return USB_PORT_STAT_LOW_SPEED;
	case usb_redir_speed_high:  return USB_PORT_STAT_HIGH_SPEED;

	case usb_redir_speed_full:
	case usb_redir_speed_super:
	default:		    return 0;
	}
}

// TODO - no thought at all to Super speed stuff...
void hcd_connect_port(struct usbredir_device *vdev)
{
	u32 *port = vdev->uhcd->port_status + vdev->rhport;

	pr_debug("hcd_connect_port %s\n", vdev->devid);
	spin_lock(&vdev->uhcd->lock);

	*port |= USB_PORT_STAT_CONNECTION | (1 << USB_PORT_FEAT_C_CONNECTION);
	*port |= speed_to_portflag(vdev->connect_header.speed);

	spin_unlock(&vdev->uhcd->lock);

	usb_hcd_poll_rh_status(usbredir_to_hcd(vdev->uhcd));
}

struct usbredir_device *find_devid(struct usbredir_hcd *uhcd, const char *devid)
{
	int i;
	struct usbredir_device *vdev = NULL;

	spin_lock(&uhcd->lock);
	for (i = 0; i < USBREDIR_NPORTS; i++) {
		vdev = port_to_vdev(uhcd, i);
		spin_lock(&vdev->lock);
		if (vdev->devid && strcmp(vdev->devid, devid) == 0) {
			spin_unlock(&vdev->lock);
			break;
		}
		spin_unlock(&vdev->lock);
	}
	spin_unlock(&uhcd->lock);

	if (i >= USBREDIR_NPORTS)
		return NULL;

	return vdev;
}

static void rh_port_disconnect(struct usbredir_device *vdev)
{
	pr_debug("rh_port_disconnect %s\n", vdev->devid);

	spin_lock(&vdev->uhcd->lock);

	vdev->uhcd->port_status[vdev->rhport] &= ~USB_PORT_STAT_CONNECTION;
	vdev->uhcd->port_status[vdev->rhport] |=
					(1 << USB_PORT_FEAT_C_CONNECTION);

	spin_unlock(&vdev->uhcd->lock);
	usb_hcd_poll_rh_status(usbredir_to_hcd(vdev->uhcd));
}

#define PORT_C_MASK				\
	((USB_PORT_STAT_C_CONNECTION		\
	  | USB_PORT_STAT_C_ENABLE		\
	  | USB_PORT_STAT_C_SUSPEND		\
	  | USB_PORT_STAT_C_OVERCURRENT		\
	  | USB_PORT_STAT_C_RESET) << 16)

static inline void hub_descriptor(struct usb_hub_descriptor *desc)
{
	memset(desc, 0, sizeof(*desc));
// TODO - where do these magic numbers come from?
	desc->bDescriptorType = 0x29;
	desc->bDescLength = 9;
	desc->wHubCharacteristics = __constant_cpu_to_le16(
		HUB_CHAR_INDV_PORT_LPSM | HUB_CHAR_COMMON_OCPM);
	desc->bNbrPorts = USBREDIR_NPORTS;
	desc->u.hs.DeviceRemovable[0] = 0xff;
	desc->u.hs.DeviceRemovable[1] = 0xff;
}

static void usbredir_device_unlink_cleanup(struct usbredir_device *vdev)
{
	struct usbredir_unlink *unlink, *tmp;

	spin_lock(&vdev->uhcd->lock);
	spin_lock(&vdev->priv_lock);

	list_for_each_entry_safe(unlink, tmp, &vdev->unlink_tx, list) {
		pr_debug("unlink cleanup tx %d\n", unlink->unlink_seqnum);
		list_del(&unlink->list);
		kfree(unlink);
	}

	while (!list_empty(&vdev->unlink_rx)) {
		struct urb *urb;

		unlink = list_first_entry(&vdev->unlink_rx, struct usbredir_unlink,
			list);

		/* give back URB of unanswered unlink request */
		pr_debug("unlink cleanup rx %d\n", unlink->unlink_seqnum);

		urb = pickup_urb_and_free_priv(vdev, unlink->unlink_seqnum);
		if (!urb) {
			pr_info("the urb (seqnum %d) was already given back\n",
				unlink->unlink_seqnum);
			list_del(&unlink->list);
			kfree(unlink);
			continue;
		}

		urb->status = -ENODEV;

		usb_hcd_unlink_urb_from_ep(usbredir_to_hcd(vdev->uhcd), urb);

		list_del(&unlink->list);

		spin_unlock(&vdev->priv_lock);
		spin_unlock(&vdev->uhcd->lock);

		usb_hcd_giveback_urb(usbredir_to_hcd(vdev->uhcd), urb,
				     urb->status);

		spin_lock(&vdev->uhcd->lock);
		spin_lock(&vdev->priv_lock);

		kfree(unlink);
	}

	spin_unlock(&vdev->priv_lock);
	spin_unlock(&vdev->uhcd->lock);
}

/*
 * The important thing is that only one context begins cleanup.
 * This is why error handling and cleanup become simple.
 * We do not want to consider race condition as possible.
 */
static void usbredir_shutdown_connection(struct usbredir_device *vdev)
{
	if (vdev->socket) {
		pr_debug("shutdown socket %p\n", vdev->socket);
		kernel_sock_shutdown(vdev->socket, SHUT_RDWR);
	}

	/* kill threads related to this dev */
	if (vdev->rx) {
		kthread_stop(vdev->rx);
		vdev->rx = NULL;
	}
	if (vdev->tx) {
		kthread_stop(vdev->tx);
		vdev->tx = NULL;
	}

	/* active connection is closed */
	if (vdev->socket) {
		sockfd_put(vdev->socket);
		vdev->socket = NULL;
	}

	usbredir_device_unlink_cleanup(vdev);

	/*
	 * rh_port_disconnect() is a trigger of ...
	 *   usb_disable_device():
	 *	disable all the endpoints for a USB device.
	 *   usb_disable_endpoint():
	 *	disable endpoints. pending urbs are unlinked(dequeued).
	 *
	 * NOTE: After calling rh_port_disconnect(), the USB device drivers of a
	 * detached device should release used urbs in a cleanup function (i.e.
	 * xxx_disconnect()). Therefore, usbredir_hcd does not need to release
	 * pushed urbs and their private data in this function.
	 *
	 * NOTE: usbredir_dequeue() must be considered carefully. When shutting down
	 * a connection, shutdown_connection() expects usbredir_dequeue()
	 * gives back pushed urbs and frees their private data by request of
	 * the cleanup function of a USB driver. When unlinking a urb with an
	 * active connection, usbredir_dequeue() does not give back the urb which
	 * is actually given back by rx_loop after receiving its return pdu.
	 *
	 */
	rh_port_disconnect(vdev);

	pr_info("disconnect device\n");
}


static void usbredir_device_reset(struct usbredir_device *vdev)
{
	spin_lock(&vdev->lock);

	if (vdev->devid)
		kfree(vdev->devid);
	vdev->devid  = NULL;

	usb_put_dev(vdev->udev);
	vdev->udev = NULL;

	if (vdev->socket) {
		// TODO - kernel shut down?
		sockfd_put(vdev->socket);
		vdev->socket = NULL;
	}
	// TODO - clean up parser and threads?
	vdev->status = VDEV_ST_NULL;

	spin_unlock(&vdev->lock);
}

static void usbredir_device_unusable(struct usbredir_device *vdev)
{
	spin_lock(&vdev->lock);
	vdev->status = VDEV_ST_ERROR;
	spin_unlock(&vdev->lock);
}

static void usbredir_device_init(struct usbredir_device *vdev, int port)
{
	memset(vdev, 0, sizeof(*vdev));

	vdev->rhport = port;
	vdev->status = VDEV_ST_NULL;
	spin_lock_init(&vdev->lock);

	INIT_LIST_HEAD(&vdev->priv_rx);
	INIT_LIST_HEAD(&vdev->priv_tx);
	INIT_LIST_HEAD(&vdev->unlink_tx);
	INIT_LIST_HEAD(&vdev->unlink_rx);
	spin_lock_init(&vdev->priv_lock);

	init_waitqueue_head(&vdev->waitq_tx);

	// TODO - why callbacks?  Why not just invoke directly?
	vdev->eh_ops.shutdown = usbredir_shutdown_connection;
	vdev->eh_ops.reset = usbredir_device_reset;
	vdev->eh_ops.unusable = usbredir_device_unusable;

	usbredir_start_eh(vdev);
}

static struct hc_driver usbredir_hc_driver = {
	.description	= driver_name,
	.product_desc	= driver_desc,
	.hcd_priv_size	= sizeof(struct usbredir_hcd),

	// TODO = what other flags are available and what of USB3?
	.flags		= HCD_USB2,

	.start		= usbredir_start,
	.stop		= usbredir_stop,

	.urb_enqueue	= urb_enqueue,
	.urb_dequeue	= urb_dequeue,

	.get_frame_number = get_frame_number,

	.hub_status_data = hub_status,
	.hub_control    = hub_control,
	.bus_suspend	= bus_suspend,
	.bus_resume	= bus_resume,
};


#if defined OLD_LOGIC_TO_CREATE_DEVICE
	/* TODO - compare/contrast with usb_create_shared_hcd */

	/*
	 * Allocate and initialize hcd.
	 * Our private data is also allocated automatically.
	 */
	hcd = usb_create_hcd(&usbredir_hc_driver, &pdev->dev,
			     dev_name(&pdev->dev));
	if (!hcd) {
		pr_err("create hcd failed\n");
		return -ENOMEM;
	}
	// TODO - review if we want to has_tt, and anything like it...
	hcd->has_tt = 1;

	/* this is private data for usbredir_hcd */
	the_controller = hcd_to_usbredir(hcd);

	/*
	 * Finish generic HCD structure initialization and register.
	 * Call the driver's reset() and start() routines.
	 */
	ret = usb_add_hcd(hcd, 0, 0);
	if (ret != 0) {
		pr_err("usb_add_hcd failed %d\n", ret);
		usb_put_hcd(hcd);
		the_controller = NULL;
		return ret;
	}

	return 0;
}

/*
 * The USBREDIR 'device' is 'virtual'; not real plug&play hardware.
 * We need to add this virtual device as a platform device arbitrarily:
 *	1. platform_device_register()
 */
static void the_pdev_release(struct device *dev)
{
}

static struct platform_device the_pdev = {
	/* should be the same name as driver_name */
	.name = driver_name,
	.id = -1, // TODO - this will have to not be -1 when we go general
	.dev = {
		.release = the_pdev_release,
	},
};


#endif

static int usbredir_hcd_remove(struct platform_device *pdev)
{
	struct usb_hcd	*hcd;

	hcd = platform_get_drvdata(pdev);
	if (!hcd)
		return 0;

	/*
	 * Disconnects the root hub,
	 * then reverses the effects of usb_add_hcd(),
	 * invoking the HCD's stop() methods.
	 */
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	the_controller = NULL;

	return 0;
}

#ifdef CONFIG_PM

/* TODO:  what should happen under suspend/resume? */
static int usbredir_hcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct usb_hcd *hcd;
	int rhport = 0;
	int connected = 0;
	int ret = 0;

	hcd = platform_get_drvdata(pdev);

	spin_lock(&the_controller->lock);

	for (rhport = 0; rhport < USBREDIR_NPORTS; rhport++)
		if (the_controller->port_status[rhport] &
		    USB_PORT_STAT_CONNECTION)
			connected += 1;

	spin_unlock(&the_controller->lock);

	if (connected > 0) {
		dev_info(&pdev->dev,
			 "We have %d active connection%s. Do not suspend.\n",
			 connected, (connected == 1 ? "" : "s"));
		ret =  -EBUSY;
	} else {
		dev_info(&pdev->dev, "suspend usbredir_hcd");
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	}

	return ret;
}

static int usbredir_hcd_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	hcd = platform_get_drvdata(pdev);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	usb_hcd_poll_rh_status(hcd);

	return 0;
}

#else

#define usbredir_hcd_suspend	NULL
#define usbredir_hcd_resume	NULL

#endif

/* JPW END HACKING */
#endif



