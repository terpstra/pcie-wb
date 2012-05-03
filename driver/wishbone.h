#ifndef WISHBONE_H
#define WISHBONE_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/cdev.h>

#define WISHBONE_VERSION "0.1"
#define WISHONE_MAX_DEVICES 32	/* default only */

typedef unsigned int wb_addr_t;
typedef unsigned int wb_data_t;

struct wishbone;
struct wishbone_operations 
{
	void (*cycle)(struct wishbone* wb, int on);
	void (*write)(struct wishbone* wb, wb_addr_t addr, wb_data_t);
	wb_data_t (*read)(struct wishbone* wb, wb_addr_t addr);
};

/* One per wishbone backend hardware */
struct wishbone 
{
	char name[32];
	const struct wishbone_operations* wops;
	struct device *parent;
	
	/* internal: */
	dev_t dev;
	struct cdev cdev;
	struct list_head list;
	struct device *device;
};

/* One per open of character device */
struct etherbone_context
{
	struct wishbone* wishbone;
	struct fasync_struct *fasync;
	char buf[1024];
};

int wishbone_register(struct wishbone* wb);
int wishbone_unregister(struct wishbone* wb);

#endif
