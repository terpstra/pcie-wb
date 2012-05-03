#include <linux/module.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/device.h>

#include "wishbone.h"

/* Module parameters */
static unsigned int max_devices = WISHONE_MAX_DEVICES;

/* Module globals */
static LIST_HEAD(wishbone_list); /* Sorted by ascending minor number */
static DEFINE_MUTEX(wishbone_mutex);
static struct class *wishbone_class;
static dev_t wishbone_dev_first;

static int char_open(struct inode *inode, struct file *filep)
{	
	struct etherbone_context* context;
	
	context = kzalloc(sizeof(struct etherbone_context), GFP_KERNEL);
	if (!context) return -ENOMEM;
	
	context->wishbone = container_of(inode->i_cdev, struct wishbone, cdev);
	filep->private_data = context;
	
	return 0;
}

static int char_release(struct inode *inode, struct file *filep)
{
	kfree(filep->private_data);
	return 0;
}

static ssize_t char_aio_read(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
	return 0;
}

static ssize_t char_aio_write(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
	return 0;
}

static unsigned int char_poll(struct file *filp, poll_table *wait)
{
	return 0;
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
