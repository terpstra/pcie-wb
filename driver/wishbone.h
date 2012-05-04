#ifndef WISHBONE_H
#define WISHBONE_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/cdev.h>

#define WISHBONE_VERSION "0.1"
#define WISHONE_MAX_DEVICES 32	/* default only */

#define ETHERBONE_BCA	0x80
#define ETHERBONE_RCA	0x40
#define ETHERBONE_RFF	0x20
#define ETHERBONE_CYC	0x08
#define ETHERBONE_WCA	0x04
#define ETHERBONE_WFF	0x02

/* Implementation assumes these have the same size: */
typedef unsigned int wb_addr_t;
typedef unsigned int wb_data_t;

struct wishbone;
struct wishbone_operations 
{
	void (*cycle)(struct wishbone* wb, int on);
	void (*byteenable)(struct wishbone* wb, unsigned char mask);
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

#define RING_SIZE	8192
#define RING_INDEX(x)	((x) & (RING_SIZE-1))
#define RING_POS(x)	((x) & (RING_SIZE*2-1))

/* One per open of character device */
struct etherbone_context
{
	struct wishbone* wishbone;
	struct fasync_struct *fasync;
	struct mutex mutex;
	wait_queue_head_t waitq;
	
	enum { header, idle, cycle } state;
	unsigned int sent, processed, received; /* sent <= processed <= received */
	
	unsigned char buf[RING_SIZE]; /* Ring buffer */
};

#define RING_READ_LEN(ctx)   RING_POS((ctx)->processed - (ctx)->sent)
#define RING_PROC_LEN(ctx)   RING_POS((ctx)->received  - (ctx)->processed)
#define RING_WRITE_LEN(ctx)  RING_POS((ctx)->sent + RING_SIZE - (ctx)->received)
#define RING_POINTER(ctx, idx) (&(ctx)->buf[RING_INDEX((ctx)->idx)])

int wishbone_register(struct wishbone* wb);
int wishbone_unregister(struct wishbone* wb);

#endif
