
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

#include <asm/io.h>		/*outb iowrite8 */
#include <asm/spinlock.h>

#include "pci_char.h"

#define FALSE 0
#define TRUE 1

int use_mem = FALSE;			/* mem or io port */
int use_msi = FALSE;			/* Message Signaled Interrupts enabled */

static int  major_num  = 0;		/* major number for char device */
static int  minor_num  = 0;		/* minor number for char device */
static struct cdev *cdev_irq;
static struct cdev *cdev_rd;

struct pci_resource *pci_res0 = NULL;	/* ptrs to resource objects */
struct pci_resource *pci_res1 = NULL;

char name_str[] = "pci_char";
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

	int ret;
	int snapl_value;
	int snaph_value;
	int snap_value;
	unsigned char *bptr = pci_res0->m_addr;
	//int written;
	
	mb();
	/* check if our device has triggered */
	ret = ioread32(bptr + TMR_0_BASE);mb();
	if (!(ret & 0x1))
		return IRQ_NONE;


	/* copy snapshot value from timer */
	/* should be atomic */
	spin_lock(&snapshot_lock);
	/* trigger the latch of the counter value */
	iowrite16(0x0, bptr + TMR_0_SNAPL);
	rmb();	// has to be executed before reading
	snapl_value = ioread16(bptr + TMR_0_SNAPL);rmb();
	snaph_value = ioread16(bptr + TMR_0_SNAPH);rmb();
	spin_unlock(&snapshot_lock);
	snap_value = (int)(snaph_value << 16) | snapl_value;
	//printk(KERN_ALERT "%s: snap_value = 0x%x\n", name_str, snap_value);
	//snap_value = snap_value | (snaph_value << 32);
	//written = sprintf((char *)short_head,"%08x\n", snap_value);
	*(u32 *)short_head = snap_value;

	short_incr_bp(&short_head, sizeof(u32));

	/* wake up the reading process */
	wake_up_interruptible(&event_wq);

	/* clear the interrupting bit in the pio */
	//iowrite32(0x0, bptr + PIO_BASE + EDGE_CAPTURE);
	/* clear pending bit in edge capture register */
	mb();
	//iowrite32(0x0, bptr + TMR_EDGE_CAPTURE);
	iowrite16(0x0, bptr + TMR_0_BASE);mb();



	//printk(KERN_ALERT "%s: irq handler called\n", name_str);
	return (irq_handler_t)IRQ_HANDLED;
}


int pci_skel_open (struct inode *inode, struct file *filp)
{	
	
	int result;
	unsigned char *bptr = pci_res0->m_addr;
	int arg = IRQ_RATE;

	instance_count++;

	iowrite16(arg, bptr + TMR_0_PERIODL);
	wmb();
	iowrite16(arg >> 16, bptr + TMR_0_PERIODH);

	/* we have a irq number and the first process is connecting */
	if (myirq >= 0 && instance_count == 1) {
		result = request_irq(	myirq,
 					(irq_handler_t)my_irq_handler,
					IRQF_SHARED,
					name_str,
					pci_res0 /* used as dev_id */
					);
		if (result) {
			printk(KERN_ALERT "%s: can't get IRQ %d\n", name_str, myirq);
			myirq = -1;
		} else {
			/* enable irq */
			/* enable MSI in MSI Control Register of PCI Endpoint */
			result = ioread32(bptr + PCIE_BASE + MSI);
			iowrite32(result |= (1UL << 16), bptr + PCIE_BASE + MSI);
			
			/* enable AVL_IRQ in MSI Control Register of PCI Endpoint */
			result = ioread32(bptr + PCIE_BASE + MSI);
			iowrite32(result |= (1UL << 7), bptr + PCIE_BASE + MSI);

			mb();

			iowrite16(0x7, bptr + TMR_0_CTRL);

			ioread32(bptr + PCIE_BASE); /* dummy read */
			mb();

		}
	}
					
	return 0;
}

int pci_rd_open (struct inode *inode, struct file *filp)
{
	return 0;
}

int pci_irq_release (struct inode *inode, struct file *filp)
{
	unsigned char *bptr = pci_res0->m_addr;
	int result;

	instance_count--;

	/* we have a irq number and the last connected process is to be released*/
	if (myirq && instance_count == 0)
	{
		free_irq(myirq, pci_res0 /* dev_id */);

		mb();
		/* disable timer irq */
		iowrite16(0x0, bptr + TMR_0_CTRL);
		mb();

		/* disable AVL_IRQ in MSI Control Register of PCI Endpoint */
		result = ioread32(bptr + PCIE_BASE + MSI);
		iowrite32(result |= (0UL << 7), bptr + PCIE_BASE + MSI);
		/* disable MSI in MSI Control Register of PCI Endpoint */
		result = ioread32(bptr + PCIE_BASE + MSI);
		iowrite32(result |= (0UL << 16), bptr + PCIE_BASE + MSI);

		ioread32(bptr + PCIE_BASE); /* dummy read */
		mb();

	}
	return 0;
}

int pci_rd_release (struct inode *inode, struct file *filp)
{

	return 0;
}

ssize_t pci_blocking_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int count0;

	
	/* there is no data so we are going to wait. */
	/* but only if there is no wakeup call pending */
	wait_event_interruptible(event_wq, short_head != short_tail);

	/* now we should have at least one event in the buffer */

	/* count0 is the number of readable data bytes */
	count0 = short_head - short_tail;
	if (count0 < 0) /* wrapped */
		count0 = short_buffer + PAGE_SIZE - short_tail;
	//printk(KERN_ALERT "%s: read count0 = %d\n", name_str, count0);
	if (count0 < count) count = count0;
	if (copy_to_user(buf, (char *)short_tail, count))
		return -EFAULT;
	short_incr_bp (&short_tail, count);
	return count;
}

ssize_t pci_nonblocking_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int snapl_value;
	int snaph_value;
	int snap_value;
	unsigned char *bptr = pci_res0->m_addr;

	/* copy snapshot value from timer */
	/* should be atomic */
	spin_lock(&snapshot_lock);
	/* trigger the latch of the counter value */
	iowrite16(0x0, bptr + TMR_0_SNAPL);
	rmb();	// has to be executed before reading
	snapl_value = ioread16(bptr + TMR_0_SNAPL);rmb();
	snaph_value = ioread16(bptr + TMR_0_SNAPH);rmb();
	spin_unlock(&snapshot_lock);
	snap_value = (int)(snaph_value << 16) | snapl_value;

	if (copy_to_user(buf, &snap_value, 4))
			return -EFAULT;

	return 4;
}

ssize_t pci_skel_write(struct file *filp, const char __user *buf, size_t count,
		loff_t *f_pos)
{
  		return 0;
}

int pci_ioctl(struct inode *inode, struct file *filep, unsigned int cmd,  unsigned long arg) {
	
	int retval = 0;
	unsigned char *bptr = pci_res0->m_addr;
	
	if(_IOC_TYPE(cmd) != PCI_CHAR_MAGIC) return -ENOTTY;
	if(_IOC_NR(cmd) > PCI_CHAR_MAXNR) return -ENOTTY;
	
	switch(cmd) {
		/*case SET_PERIOD:
			//printk(KERN_ALERT "%s: periodl 0x%x\n", name_str, arg);
			 should be atomic
			spin_lock(&period_lock);
			iowrite16(arg, bptr + TMR_0_PERIODL);
			wmb();
			iowrite16(arg >> 16, bptr + TMR_0_PERIODH);
			spin_unlock(&period_lock);
			break;
		case GET_PERIOD:
			spin_lock(&period_lock);
			retval = ioread16(bptr + TMR_0_PERIODL);
			rmb();
			retval = ioread16(bptr + TMR_0_PERIODH) << 16 | retval;
			spin_unlock(&period_lock);
			printk(KERN_ALERT "%s: period value is 0x%x\n", name_str, retval);
			return retval;
			break;*/
			
		default:
			return -ENOTTY;
	}
	return retval;
}


unsigned int pci_skel_poll(struct file *filp, poll_table *wait)
{
	return POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM;
}

struct file_operations pci_irq_fops = {
	.owner	 = THIS_MODULE,
	.read	 = pci_blocking_read,
	.write	 = pci_skel_write,
	.ioctl   = pci_ioctl,		
	.poll	 = pci_skel_poll,
	.open	 = pci_skel_open,
	.release = pci_irq_release,
};

struct file_operations pci_rd_fops = {
	.owner	 = THIS_MODULE,
	.read	 = pci_nonblocking_read,
	.poll	 = pci_skel_poll,
	.open	 = pci_rd_open,
	.release = pci_rd_release,
};

static struct pci_device_id ids[] = {
	{ PCI_DEVICE(ALTERA_ID, DEVICE_ID), },
	{ PCI_DEVICE(0x1002, 0x4750), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

static unsigned char skel_get_revision(struct pci_dev *dev)
{
	u8 revision;

	pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
	return revision;
}

static int probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	/* Do probing type stuff here.  
	 * Like calling request_region();
	 * reading BARs
	 * reading IRQ
	 * register char dev
	 */
	int result;
	int is_mem;
	dev_t dev_irq = MKDEV(major_num, minor_num);
	dev_t dev_rd = MKDEV(major_num, minor_num);

	if (skel_get_revision(pdev) != 0x01) {
		printk(KERN_ALERT "%s: revision ID wrong!\n", name_str);
		return -ENODEV;
	}

	/* enable pcie error reporting */
	result = pci_enable_pcie_error_reporting(pdev);
	if (result) {
		printk(KERN_ALERT "Could not enable pcie error reporting.\n");
	}

	/* enable message signaled interrupts */
	result = pci_enable_msi(pdev);
	/* could not use MSI? */
	if (result) {
	/* resort to legacy interrupts */
		printk(KERN_ALERT "Could not enable MSI interrupting, staying with legacy.\n");
	/* MSI enabled, remember for cleanup */
	} else {
		use_msi = TRUE;
		printk(KERN_ALERT "Enabled MSI interrupting.\n");
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
	result  = alloc_chrdev_region (&dev_irq, FIRSTMINOR, DEV_COUNT, name_str);
	if(result < 0) {
		printk(KERN_ALERT "%s: ERROR unable to get major number\n", name_str);
		goto err_alloc_chrdev;
	} else {
		major_num = MAJOR(dev_irq);
		printk(KERN_ALERT "%s: My major nummber is: %d\n", name_str, major_num);
	};

	// assign dev_rd the next minor number
	dev_rd = MKDEV(major_num, MINOR(dev_irq) + 1);

	// allocate cdev structures
	cdev_irq = cdev_alloc();
	cdev_rd = cdev_alloc();

	if((cdev_irq == NULL) || (cdev_rd == NULL)){
		printk("%s: ERROR allocate cdev structur failure\n", name_str);
		goto err_cdev_alloc;
	}

	cdev_irq->owner  = THIS_MODULE;
	cdev_irq->ops    = &pci_irq_fops;

	cdev_rd->owner  = THIS_MODULE;
	cdev_rd->ops    = &pci_rd_fops;

	result = cdev_add(cdev_irq, dev_irq, 1);
	if(result < 0){
		printk("%s: cdev_add dev_irq failed\n", name_str);
		goto err_alloc_chrdev;
	} else
		printk("%s: added cdev_irq\n", name_str);

	result = cdev_add(cdev_rd, dev_rd, 1);
	if(result < 0){
		printk("%s: cdev_add dev_rd failed\\n", name_str);
		goto err_cdev_add;
	} else
		printk("%s: added cdev_rd\n", name_str);


	short_buffer = __get_free_pages(GFP_KERNEL,0);  //never fails
	short_head = short_tail = short_buffer;

	return pci_enable_device(pdev);

	/* cleaning up */
	err_cdev_add:
		cdev_del(cdev_irq);
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
	if (cdev_rd)
		cdev_del(cdev_rd);
	if (cdev_irq)
		cdev_del(cdev_irq);
	if (major_num) {
		unregister_chrdev_region (MKDEV(major_num, minor_num), DEV_COUNT);
		printk(KERN_ALERT "%s: released major number %i\n", name_str, major_num);
	}
	if (use_msi)
		pci_disable_msi(pdev);

	iounmap(pci_res0->m_addr);
	release_mem_region(pci_res0->start, pci_res0->size);
	printk(KERN_ALERT "%s: released io 0x%lx\n", name_str, pci_res0->start);
	if (pci_res0 != NULL)
		kfree(pci_res0);
}

static int err_handler(struct pci_dev *pdev) {
	printk(KERN_ALERT "%s:  err handler called!\n", name_str);
	return 0;
}

static struct pci_driver pci_driver = {
	.name = name_str,
	.id_table = ids,
	.probe = probe,
	.remove = remove,
	.err_handler = err_handler,
};

static int __init pci_skel_init(void)
{
	return pci_register_driver(&pci_driver);
}

static void __exit pci_skel_exit(void)
{	
	pci_unregister_driver(&pci_driver);
}

MODULE_LICENSE("GPL");

module_init(pci_skel_init);
module_exit(pci_skel_exit);
