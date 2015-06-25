ccflags-$(CONFIG_USBREDIR_DEBUG) := -DDEBUG

obj-$(CONFIG_USBREDIR) += usbredir.o
#usbredir-y := redir.o event.o usbredirparser.o strtok_r.o usbredirfilter.o sysfs.o hcd.o rx.o tx.o
usbredir-y := main.o sysfs.o hub.o urb.o device.o redir.o tx.o rx.o usbredirparser.o strtok_r.o usbredirfilter.o
