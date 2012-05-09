#include <linux/module.h>

#include <linux/fs.h>
#include <linux/aio.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/socket.h>
#include <linux/device.h>
#include <linux/sched.h>

#include "wishbone.h"

/* Module parameters */
static unsigned int max_devices = WISHONE_MAX_DEVICES;

/* Module globals */
static LIST_HEAD(wishbone_list); /* Sorted by ascending minor number */
static DEFINE_MUTEX(wishbone_mutex);
static struct class *wishbone_class;
static dev_t wishbone_dev_first;

/* Compiler should be able to optimize this to one inlined instruction */
static inline wb_data_t eb_to_cpu(unsigned char* x)
{
	switch (sizeof(wb_data_t)) {
	case 8: return be64_to_cpu(*(wb_data_t*)x);
	case 4: return be32_to_cpu(*(wb_data_t*)x);
	case 2: return be16_to_cpu(*(wb_data_t*)x);
	case 1: return *(wb_data_t*)x;
	}
}

/* Compiler should be able to optimize this to one inlined instruction */
static inline void eb_from_cpu(unsigned char* x, wb_data_t dat)
{
	switch (sizeof(wb_data_t)) {
	case 8: *(wb_data_t*)x = cpu_to_be64(dat); break;
	case 4: *(wb_data_t*)x = cpu_to_be32(dat); break;
	case 2: *(wb_data_t*)x = cpu_to_be16(dat); break;
	case 1: *(wb_data_t*)x = dat;              break;
	}
}

static void etherbone_process(struct etherbone_context* context)
{
	struct wishbone *wb;
	const struct wishbone_operations *wops;
	unsigned int size, left, i, record_len;
	unsigned char *buf;
	
	if (context->state == header) {
		if (context->received < 8) {
			/* no-op */
			return;
		}
		
		context->buf[0] = 0x4E;
		context->buf[1] = 0x6F;
		context->buf[2] = 0x12; /* V.1 Probe-Response */
		context->buf[3] = (sizeof(wb_addr_t)<<4) | sizeof(wb_data_t);
		/* Echo back bytes 4-7, the probe identifier */
		context->processed = 8;
		context->state = idle;
	}
	
	buf = &context->buf[0];
	wb = context->wishbone;
	wops = wb->wops;
	
	i = RING_INDEX(context->processed);
	size = RING_PROC_LEN(context);
	
	for (left = size; left >= 4; left -= record_len) {
		unsigned char flags, be, wcount, rcount;
		
		/* Determine record size */
		flags  = buf[i+0];
		be     = buf[i+1];
		wcount = buf[i+2];
		rcount = buf[i+3];
		
		record_len = 1 + wcount + rcount + (wcount > 0) + (rcount > 0);
		record_len *= sizeof(wb_data_t);
		
		if (left < record_len) break;
		
		/* Configure byte enable and raise cycle line */
		wops->byteenable(wb, be);
		if (context->state == idle) {
			wops->cycle(wb, 1);
			context->state = cycle;
		}

		/* Process the writes */
		if (wcount > 0) {
			wb_addr_t base_address;
			unsigned char j;
			int wff = flags & ETHERBONE_WFF;
			int wca = flags & ETHERBONE_WCA;
			
			/* Erase the header */
			eb_from_cpu(buf+i, 0);
			i = RING_INDEX(i + sizeof(wb_data_t));
			base_address = eb_to_cpu(buf+i);
			
			if (wca) {
				for (j = wcount; j > 0; --j) {
					eb_from_cpu(buf+i, 0);
					i = RING_INDEX(i + sizeof(wb_data_t));
				}
			} else {
				for (j = wcount; j > 0; --j) {
					eb_from_cpu(buf+i, 0);
					i = RING_INDEX(i + sizeof(wb_data_t));
					wops->write(wb, base_address, eb_to_cpu(buf+i));
					
					if (!wff) base_address += sizeof(wb_data_t);
				}
			}
		}
		
		buf[i+0] = (flags & ETHERBONE_CYC) | 
		           (((flags & ETHERBONE_RFF) != 0) ? ETHERBONE_WFF : 0) |
		           (((flags & ETHERBONE_BCA) != 0) ? ETHERBONE_WCA : 0);
		buf[i+1] = be;
		buf[i+2] = rcount; /* rcount -> wcount */
		buf[i+3] = 0;
		
		if (rcount > 0) {
			unsigned char j;
			int rca = flags & ETHERBONE_RCA;
			
			/* Move past header, and leave BaseRetAddr intact */
			i = RING_INDEX(i + sizeof(wb_data_t) + sizeof(wb_data_t));
			
			if (rca) {
				for (j = rcount; j > 0; --j) {
					eb_from_cpu(buf+i, wops->read_cfg(wb, eb_to_cpu(buf+i)));
					i = RING_INDEX(i + sizeof(wb_data_t));
				}
			} else {
				for (j = rcount; j > 0; --j) {
					eb_from_cpu(buf+i, wops->read(wb, eb_to_cpu(buf+i)));
					i = RING_INDEX(i + sizeof(wb_data_t));
				}
			}
		}
		
		if ((flags & ETHERBONE_CYC) == 0) {
			wops->cycle(wb, 0);
			context->state = idle;
		}
	}
	
	context->processed = RING_POS(context->processed + size - left);
}

static int char_open(struct inode *inode, struct file *filep)
{	
	struct etherbone_context *context;
	
	context = kmalloc(sizeof(struct etherbone_context), GFP_KERNEL);
	if (!context) return -ENOMEM;
	
	context->wishbone = container_of(inode->i_cdev, struct wishbone, cdev);
	context->fasync = 0;
	mutex_init(&context->mutex);
	init_waitqueue_head(&context->waitq);
	context->state = header;
	context->sent = 0;
	context->processed = 0;
	context->received = 0;
	
	filep->private_data = context;
	
	return 0;
}

static int char_release(struct inode *inode, struct file *filep)
{
	struct etherbone_context *context = filep->private_data;
	
	/* Did the bad user forget to drop the cycle line? */
	if (context->state == cycle) {
		context->wishbone->wops->cycle(context->wishbone, 0);
	}
	
	kfree(context);
	return 0;
}

static ssize_t char_aio_read(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
	struct file *filep = iocb->ki_filp;
	struct etherbone_context *context = filep->private_data;
	unsigned int len, iov_len, ring_len, buf_len;
	
	iov_len = iov_length(iov, nr_segs);
	if (unlikely(iov_len == 0)) return 0;
	
	mutex_lock(&context->mutex);
	
	ring_len = RING_READ_LEN(context);
	len = min_t(unsigned int, ring_len, iov_len);
	
	/* How far till we must wrap?  */
	buf_len = sizeof(context->buf) - RING_INDEX(context->sent);
	
	if (buf_len < len) {
		memcpy_toiovecend(iov, RING_POINTER(context, sent), 0, buf_len);
		memcpy_toiovecend(iov, &context->buf[0],            buf_len, len-buf_len);
	} else {
		memcpy_toiovecend(iov, RING_POINTER(context, sent), 0, len);
	}
	context->sent = RING_POS(context->sent + len);
	
	mutex_unlock(&context->mutex);
	
	/* Wake-up polling descriptors */
	wake_up_interruptible(&context->waitq);
	kill_fasync(&context->fasync, SIGIO, POLL_OUT);
	
	if (len == 0 && (filep->f_flags & O_NONBLOCK) != 0)
		return -EAGAIN;
	
	return len;
}

static ssize_t char_aio_write(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
	struct file *filep = iocb->ki_filp;
	struct etherbone_context *context = filep->private_data;
	unsigned int len, iov_len, ring_len, buf_len;
	
	iov_len = iov_length(iov, nr_segs);
	if (unlikely(iov_len == 0)) return 0;
	
	mutex_lock(&context->mutex);
	
	ring_len = RING_WRITE_LEN(context);
	len = min_t(unsigned int, ring_len, iov_len);
	
	/* How far till we must wrap?  */
	buf_len = sizeof(context->buf) - RING_INDEX(context->received);
	
	if (buf_len < len) {
		memcpy_fromiovecend(RING_POINTER(context, received), iov, 0, buf_len);
		memcpy_fromiovecend(&context->buf[0],                iov, buf_len, len-buf_len);
	} else {
		memcpy_fromiovecend(RING_POINTER(context, received), iov, 0, len);
	}
	context->received = RING_POS(context->received + len);
	
	/* Process buffers */
	etherbone_process(context);
	
	mutex_unlock(&context->mutex);
	
	/* Wake-up polling descriptors */
	wake_up_interruptible(&context->waitq);
	kill_fasync(&context->fasync, SIGIO, POLL_IN);
	
	if (len == 0 && (filep->f_flags & O_NONBLOCK) != 0)
		return -EAGAIN;
	
	return len;
}

static unsigned int char_poll(struct file *filep, poll_table *wait)
{
	unsigned int mask = 0;
	struct etherbone_context *context = filep->private_data;
	
	poll_wait(filep, &context->waitq, wait);
	
	mutex_lock(&context->mutex);
	
	if (RING_READ_LEN (context) != 0) mask |= POLLIN  | POLLRDNORM;
	if (RING_WRITE_LEN(context) != 0) mask |= POLLOUT | POLLWRNORM;
	
	mutex_unlock(&context->mutex);
	
	return mask;
}

static int char_fasync(int fd, struct file *file, int on)
{
	struct etherbone_context* context;
	context = file->private_data;

        /* No locking - fasync_helper does its own locking */
        return fasync_helper(fd, file, on, &context->fasync);
}

static const struct file_operations etherbone_fops = {
        .owner          = THIS_MODULE,
        .llseek         = no_llseek,
        .read           = do_sync_read,
        .aio_read       = char_aio_read,
        .write          = do_sync_write,
        .aio_write      = char_aio_write,
        .open           = char_open,
        .poll           = char_poll,
        .release        = char_release,
        .fasync         = char_fasync,
};

int wishbone_register(struct wishbone* wb)
{
	struct list_head *list_pos;
	struct device *device;
	dev_t dev;
	
	INIT_LIST_HEAD(&wb->list);
	
	mutex_lock(&wishbone_mutex);
	
	/* Search the list for gaps, stopping past the gap.
	 * If we overflow the list (ie: not gaps), minor already points past end.
	 */
	dev = wishbone_dev_first;
	list_for_each(list_pos, &wishbone_list) {
		struct wishbone *entry =
			container_of(list_pos, struct wishbone, list);
		
		if (entry->dev != dev) {
			/* We found a gap! */
			break;
		} else {
			/* Run out of minors? */
			if (MINOR(dev) == max_devices) goto fail_out;
			
			/* Try the next minor */
			dev = MKDEV(MAJOR(dev), MINOR(dev) + 1);
		}
	}
	
	/* Connect the file operations with the cdev */
	cdev_init(&wb->cdev, &etherbone_fops);
	wb->cdev.owner = THIS_MODULE;
	
	/* Connect the major/minor number to the cdev */
	if (cdev_add(&wb->cdev, dev, 1)) goto fail_out;
	
	/* Create the sysfs entry */
	device = device_create(wishbone_class, wb->parent, dev, NULL, wb->name, MINOR(dev));
	if (IS_ERR(device)) goto fail_del;
	
	/* Insert the device into the gap */
	wb->dev = dev;
	wb->device = device;
	list_add_tail(&wb->list, list_pos);
	
	mutex_unlock(&wishbone_mutex);
	return 0;

fail_del:
	cdev_del(&wb->cdev);
fail_out:
	mutex_unlock(&wishbone_mutex);
	return -ENOMEM;
}

int wishbone_unregister(struct wishbone* wb)
{
	if (WARN_ON(list_empty(&wb->list)))
		return -EINVAL;
	
	mutex_lock(&wishbone_mutex);
	list_del(&wb->list);
	device_destroy(wishbone_class, wb->dev);
	cdev_del(&wb->cdev);
	mutex_unlock(&wishbone_mutex);
	
	return 0;
}

static int __init wishbone_init(void)
{
	int err;
	dev_t overflow;
	
	overflow = MKDEV(0, max_devices-1);
	if (MINOR(overflow) != max_devices-1) {
		err = -ENOMEM;
		goto fail_last;
	}
	
	wishbone_class = class_create(THIS_MODULE, "wb");
	if (IS_ERR(wishbone_class)) {
		err = PTR_ERR(wishbone_class);
		goto fail_last;
	}
	
	if (alloc_chrdev_region(&wishbone_dev_first, 0, max_devices, "wb") < 0) {
		err = -EIO;
		goto fail_class;
	}
	
	return 0;

fail_class:
	class_destroy(wishbone_class);
fail_last:
	return err;
}

static void __exit wishbone_exit(void)
{
	unregister_chrdev_region(wishbone_dev_first, max_devices);
	class_destroy(wishbone_class);
}

MODULE_AUTHOR("Wesley W. Terpstra <w.terpstra@gsi.de>");
MODULE_DESCRIPTION("Wishbone character device class");
module_param(max_devices, int, 0644);
MODULE_PARM_DESC(max_devices, "Maximum number of attached wishbone devices");
MODULE_LICENSE("GPL");
MODULE_VERSION(WISHBONE_VERSION);

EXPORT_SYMBOL(wishbone_register);
EXPORT_SYMBOL(wishbone_unregister);

module_init(wishbone_init);
module_exit(wishbone_exit);
