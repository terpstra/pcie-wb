
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/aer.h>
#include <linux/sched.h> 

#include <asm/io.h>		/*outb iowrite8 */
#include <asm/spinlock.h>

#include "pcie_wb.h"

#define FALSE 0
#define TRUE 1

typedef float __m128 __attribute__ ((__vector_size__ (16)));

static int  major_num  = 0;		/* major number for char device */
static int  minor_num  = 0;		/* minor number for char device */
static struct cdev *cdev_char;

struct pci_resource *pci_res0 = NULL;	/* ptrs to resource objects */
struct pci_resource *pci_res1 = NULL;

char name_str[] = "pcie_wb";
char myirq = -1;						/* IRQ from PCI config reg */
int instance_count = 0;					/* counting connected processes for housekeeping */

static DECLARE_WAIT_QUEUE_HEAD(event_wq);
static DEFINE_SPINLOCK(snapshot_lock);
static DEFINE_SPINLOCK(period_lock);

unsigned long short_buffer = 0;
unsigned long volatile short_head;
volatile unsigned long short_tail;


static inline void short_incr_bp(volatile unsigned long *index, int delta)
{
	unsigned long new = *index + delta;
	barrier();  /* Don't optimize these two together */
	*index = (new >= (short_buffer + PAGE_SIZE)) ? short_buffer : new;
}

struct pci_resource {
	int bar;						/* BAR number from 0 to 5 */
	unsigned long start;			/* start addr of BAR */
	unsigned long end;				/* end addr of BAR */
	unsigned int size;
	void *m_addr;					/* remapped addr */
	int is_mem;
};

irq_handler_t my_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	return (irq_handler_t)IRQ_HANDLED;
}


int char_open (struct inode *inode, struct file *filp)
{	
	//iowrite16(arg >> 16, bptr + TMR_0_PERIODH);
	//wmb();

	return 0;
}

int char_release (struct inode *inode, struct file *filp)
{

	return 0;
}

ssize_t char_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	unsigned char *bptr = pci_res0->m_addr;
	int value = 0x41424344;

	/* copy snapshot value from timer */
	/* should be atomic */
	spin_lock(&snapshot_lock);
	rmb();	// has to be executed before reading
	spin_unlock(&snapshot_lock);

	if (copy_to_user(buf, &value, 4))
		return -EFAULT;

	return 4;
}

ssize_t char_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	unsigned char *bptr = pci_res0->m_addr;
	uint32_t value;
	__m128 x;

	if (copy_from_user(&value, buf, 4))
		return -EFAULT;
	
	/* copy snapshot value from timer */
	/* should be atomic */
	spin_lock(&snapshot_lock);
	iowrite32(value, bptr + 8);
	iowrite32(value, bptr + 12);
	iowrite16(value, bptr + 18);
	iowrite8(value, bptr + 17);
	ioread16(bptr + 16);
//	iowrite32(value, bptr + 12);
//	iowrite32(value, bptr + 16);
//	iowrite16(value, bptr + 18);
//	iowrite8(value, bptr + 25);
	rmb();	// has to be executed before reading
	spin_unlock(&snapshot_lock);

	return 1;
}

unsigned int char_poll(struct file *filp, poll_table *wait)
{
	return POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM;
}

struct file_operations char_fops = {
	.owner	 = THIS_MODULE,
	.read	 = char_read,
	.write	 = char_write,
	.poll	 = char_poll,
	.open	 = char_open,
	.release = char_release,
};

static struct pci_device_id ids[] = {
	{ PCI_DEVICE(ALTERA_ID, DEVICE_ID), },
	{ PCI_DEVICE(0x1002, 0x4750), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

static int probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	/* Do probing type stuff here.  
	 * Like calling request_region();
	 * reading BARs
	 * reading IRQ
	 * register char dev
	 */
	dev_t dev_char;
	int result;
	int is_mem;
	u8 revision;

	dev_char = MKDEV(major_num, minor_num);

	pci_read_config_byte(pdev, PCI_REVISION_ID, &revision);
	if (revision != 0x01) {
		printk(KERN_ALERT "%s: revision ID wrong!\n", name_str);
		return -ENODEV;
	}

	/* enable pcie error reporting */
/*
	result = pci_enable_pcie_error_reporting(pdev);
	if (result) {
		printk(KERN_ALERT "Could not enable pcie error reporting.\n");
		return -ENODEV;
	}
*/
	/* enable message signaled interrupts */
	if (pci_enable_msi(pdev) != 0) {
		/* resort to legacy interrupts */
		printk(KERN_ALERT "Could not enable MSI interrupting, staying with legacy.\n");
		return -ENODEV;
	}

	pci_res0 = kmalloc(sizeof(struct pci_resource), GFP_KERNEL);	/* mem for object */
	if (!pci_res0) {
		printk(KERN_ALERT "%s: could not alloc mem for pci_res0!\n", name_str);
		goto err_pci_res0;
	}

	/*init of pci_res0 */
	pci_res0->bar = 0;
	pci_res0->start = pci_resource_start(pdev, 0);
	pci_res0->end = pci_resource_end(pdev, 0);
	pci_res0->size = pci_res0->end - pci_res0->start + 1;

	is_mem = pci_resource_flags(pdev, 0);
	is_mem = is_mem & IORESOURCE_MEM;
	pci_res0->is_mem = is_mem;

	printk(KERN_ALERT "%s: BAR0  0x%lx - 0x%lx\n", name_str, pci_res0->start, pci_res0->end);

	if (!request_mem_region(pci_res0->start, pci_res0->size, name_str)) {
		printk(KERN_ALERT "%s: request_mem_region failed\n", name_str);
		goto err_mem_region;
	}
	
	pci_res0->m_addr = ioremap_nocache(pci_res0->start, pci_res0->size);
	printk(KERN_ALERT "%s: ioremap to %lx\n", name_str, (unsigned long)pci_res0->m_addr);

	/* get irq number from pci */
	myirq = pdev->irq;
	printk(KERN_ALERT "%s: using IRQ number %d\n", name_str, myirq);

	/* alloc DEV_COUNT device number */
	result  = alloc_chrdev_region (&dev_char, FIRSTMINOR, DEV_COUNT, name_str);
	if(result < 0) {
		printk(KERN_ALERT "%s: ERROR unable to get major number\n", name_str);
		goto err_alloc_chrdev;
	} else {
		major_num = MAJOR(dev_char);
		printk(KERN_ALERT "%s: My major nummber is: %d\n", name_str, major_num);
	};

	// allocate cdev structures
	cdev_char = cdev_alloc();

	if(cdev_char == NULL) {
		printk("%s: ERROR allocate cdev structur failure\n", name_str);
		goto err_cdev_alloc;
	}

	cdev_char->owner  = THIS_MODULE;
	cdev_char->ops    = &char_fops;

	result = cdev_add(cdev_char, dev_char, 1);
	if(result < 0){
		printk("%s: cdev_add dev_char failed\n", name_str);
		goto err_alloc_chrdev;
	} else
		printk("%s: added cdev_irq\n", name_str);

	short_buffer = __get_free_pages(GFP_KERNEL,0);  //never fails
	short_head = short_tail = short_buffer;

	return pci_enable_device(pdev);

	/* cleaning up */
	err_cdev_alloc:
		unregister_chrdev_region (MKDEV(major_num, minor_num), DEV_COUNT);
	err_alloc_chrdev:
		iounmap(pci_res0->m_addr);
		release_mem_region(pci_res0->start, pci_res0->size);
	err_mem_region:
		kfree(pci_res0);
	err_pci_res0:
		return -EIO;
}

static void remove(struct pci_dev *pdev)
{
	
	/* clean up any allocated resources and stuff here.
	 * like call release_mem_region();
	 */

	printk(KERN_ALERT "%s: releasing resources\n", name_str);
	if (cdev_char)
		cdev_del(cdev_char);
	if (major_num) {
		unregister_chrdev_region (MKDEV(major_num, minor_num), DEV_COUNT);
		printk(KERN_ALERT "%s: released major number %i\n", name_str, major_num);
	}
	pci_disable_msi(pdev);

	iounmap(pci_res0->m_addr);
	release_mem_region(pci_res0->start, pci_res0->size);
	printk(KERN_ALERT "%s: released io 0x%lx\n", name_str, pci_res0->start);
	if (pci_res0 != NULL)
		kfree(pci_res0);
}

static struct pci_driver pci_driver = {
	.name = name_str,
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};

static int __init pcie_wb_init(void)
{
	return pci_register_driver(&pci_driver);
}

static void __exit pcie_wb_exit(void)
{	
	pci_unregister_driver(&pci_driver);
}

MODULE_LICENSE("GPL");

module_init(pcie_wb_init);
module_exit(pcie_wb_exit);
