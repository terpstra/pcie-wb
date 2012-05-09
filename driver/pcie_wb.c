
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
#include <linux/miscdevice.h>

#include <asm/io.h>
#include <asm/spinlock.h>
#include <asm/byteorder.h>

#include "pcie_wb.h"
#include "wishbone.h"

#if defined(__BIG_ENDIAN)
#define endian_addr(width, shift) (sizeof(wb_data_t)-width)-shift
#elif defined(__LITTLE_ENDIAN)
#define endian_addr(width, shift) shift
#else
#error "unknown machine byte order (endian)"
#endif

static unsigned int debug = 0;

static void wb_cycle(struct wishbone* wb, int on)
{
	struct pcie_wb_dev* dev;
	unsigned char* control;
	
	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;
	
	if (on) mutex_lock(&dev->mutex);
	
	iowrite32(on?0x80000000UL:0, control + CONTROL_REGISTER_HIGH);
	
	if (!on) mutex_unlock(&dev->mutex);
}

static void wb_byteenable(struct wishbone* wb, unsigned char be)
{
	struct pcie_wb_dev* dev;
	
	dev = container_of(wb, struct pcie_wb_dev, wb);
	
	switch (be) {
	case 0x1:
		dev->width = 1;
		dev->shift = 0;
		dev->low_addr = endian_addr(1, 0);
		break;
	case 0x2:
		dev->width = 1;
		dev->shift = 8;
		dev->low_addr = endian_addr(1, 1);
		break;
	case 0x4:
		dev->width = 1;
		dev->shift = 16;
		dev->low_addr = endian_addr(1, 2);
		break;
	case 0x8:
		dev->width = 1;
		dev->shift = 24;
		dev->low_addr = endian_addr(1, 3);
		break;
	case 0x3:
		dev->width = 2;
		dev->shift = 0;
		dev->low_addr = endian_addr(2, 0);
		break;
	case 0xC:
		dev->width = 2;
		dev->shift = 16;
		dev->low_addr = endian_addr(2, 2);
		break;
	case 0xF:
		dev->width = 4;
		dev->shift = 0;
		dev->low_addr = endian_addr(4, 0);
		break;
	default:
		/* noop -- ignore the strange bitmask */
		break;
	}
}

static void wb_write(struct wishbone* wb, wb_addr_t addr, wb_data_t data)
{
	struct pcie_wb_dev* dev;
	unsigned char* control;
	unsigned char* window;
	wb_addr_t window_offset;
	
	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;
	window = dev->pci_res[1].addr;
	
	window_offset = addr & WINDOW_HIGH;
	if (window_offset != dev->window_offset) {
		iowrite32(window_offset, control + WINDOW_OFFSET_LOW);
		dev->window_offset = window_offset;
	}
	
	switch (dev->width) {
	case 4:	
		if (unlikely(debug)) printk(KERN_ALERT PCIE_WB ": iowrite32(0x%x, 0x%x)\n", data, addr);
		iowrite32(data, window + (addr & WINDOW_LOW)); 
		break;
	case 2: 
		if (unlikely(debug)) printk(KERN_ALERT PCIE_WB ": iowrite16(0x%x, 0x%x)\n", data >> dev->shift, addr + dev->low_addr);
		iowrite16(data >> dev->shift, window + (addr & WINDOW_LOW) + dev->low_addr); 
		break;
	case 1: 
		if (unlikely(debug)) printk(KERN_ALERT PCIE_WB ": iowrite8(0x%x, 0x%x)\n", data >> dev->shift, addr + dev->low_addr);
		iowrite8 (data >> dev->shift, window + (addr & WINDOW_LOW) + dev->low_addr); 
		break;
	}
}

static wb_data_t wb_read(struct wishbone* wb, wb_addr_t addr)
{
	wb_data_t out;
	struct pcie_wb_dev* dev;
	unsigned char* control;
	unsigned char* window;
	wb_addr_t window_offset;
	
	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;
	window = dev->pci_res[1].addr;
	
	window_offset = addr & WINDOW_HIGH;
	if (window_offset != dev->window_offset) {
		iowrite32(window_offset, control + WINDOW_OFFSET_LOW);
		dev->window_offset = window_offset;
	}
	
	switch (dev->width) {
	case 4:	
		if (unlikely(debug)) printk(KERN_ALERT PCIE_WB ": ioread32(0x%x)\n", addr);
		out = ((wb_data_t)ioread32(window + (addr & WINDOW_LOW)));
		break;
	case 2: 
		if (unlikely(debug)) printk(KERN_ALERT PCIE_WB ": ioread32(0x%x)\n", addr + dev->low_addr);
		out = ((wb_data_t)ioread16(window + (addr & WINDOW_LOW) + dev->low_addr)) << dev->shift;
		break;
	case 1: 
		if (unlikely(debug)) printk(KERN_ALERT PCIE_WB ": ioread32(0x%x)\n", addr + dev->low_addr);
		out = ((wb_data_t)ioread8 (window + (addr & WINDOW_LOW) + dev->low_addr)) << dev->shift;
		break;
	default: // technically should be unreachable
		out = 0;
		break;
	}

	mb(); /* ensure serial ordering of non-posted operations for wishbone */
	
	return out;
}

static wb_data_t wb_read_cfg(struct wishbone *wb, wb_addr_t addr)
{
	wb_data_t out;
	struct pcie_wb_dev* dev;
	unsigned char* control;
	
	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;
	
	switch (addr) {
	case 0:  out = ioread32(control + ERROR_FLAG_HIGH);   break;
	case 4:  out = ioread32(control + ERROR_FLAG_LOW);    break;
	case 8:  out = ioread32(control + SDWB_ADDRESS_HIGH); break;
	case 12: out = ioread32(control + SDWB_ADDRESS_LOW);  break;
	default: out = 0; break;
	}
	
	mb(); /* ensure serial ordering of non-posted operations for wishbone */
	
	return out;
}

static const struct wishbone_operations wb_ops = {
	.cycle      = wb_cycle,
	.byteenable = wb_byteenable,
	.write      = wb_write,
	.read       = wb_read,
	.read_cfg   = wb_read_cfg,
};

#if 0
static irq_handler_t irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	return (irq_handler_t)IRQ_HANDLED;
}
#endif

static int setup_bar(struct pci_dev* pdev, struct pcie_wb_resource* res, int bar)
{
	/*init of pci_res0 */
	res->start = pci_resource_start(pdev, bar);
	res->end = pci_resource_end(pdev, bar);
	res->size = res->end - res->start + 1;
	
	if (debug)
		printk(KERN_ALERT PCIE_WB "/BAR%d  0x%lx - 0x%lx\n", bar, res->start, res->end);

	// is_mem = pci_resource_flags(pdev, 0);
 	// is_mem = is_mem & IORESOURCE_MEM;

	if (!request_mem_region(res->start, res->size, PCIE_WB)) {
		printk(KERN_ALERT PCIE_WB "/BAR%d: request_mem_region failed\n", bar);
		return -ENOMEM;
	}
	
	res->addr = ioremap_nocache(res->start, res->size);
	if (debug)
		printk(KERN_ALERT PCIE_WB "/BAR%d: ioremap to %lx\n", bar, (unsigned long)res->addr);
	
	return 0;
}

static void destroy_bar(struct pcie_wb_resource* res)
{
	if (debug)
		printk(KERN_ALERT "released io 0x%lx\n", res->start);
		
	iounmap(res->addr);
	release_mem_region(res->start, res->size);
}

static int probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	/* Do probing type stuff here.  
	 * Like calling request_region();
	 * reading BARs
	 * reading IRQ
	 * register char dev
	 */
	u8 revision;
	struct pcie_wb_dev *dev;
	unsigned char* control;

	pci_read_config_byte(pdev, PCI_REVISION_ID, &revision);
	if (revision != 0x01) {
		printk(KERN_ALERT PCIE_WB ": revision ID wrong!\n");
		goto fail_out;
	}

	dev = kmalloc(sizeof(struct pcie_wb_dev), GFP_KERNEL);
	if (!dev) {
		printk(KERN_ALERT PCIE_WB ": could not allocate memory for pcie_wb_dev structure!\n");
		goto fail_out;
	}
	
	/* Initialize structure */
	dev->pci_dev = pdev;
	dev->wb.wops = &wb_ops;
	strcpy(dev->wb.name, PCIE_WB "%d");
	dev->wb.parent = &pdev->dev;
	mutex_init(&dev->mutex);
	dev->window_offset = 0;
	dev->low_addr = 0;
	dev->width = 4;
	dev->shift = 0;
	pci_set_drvdata(pdev, dev);
	
	/* enable message signaled interrupts */
	if (pci_enable_msi(pdev) != 0) {
		/* resort to legacy interrupts */
		printk(KERN_ALERT PCIE_WB ": could not enable MSI interrupting\n");
		goto fail_free;
	}

	if (setup_bar(pdev, &dev->pci_res[0], 0) < 0) goto fail_msi;
	if (setup_bar(pdev, &dev->pci_res[1], 1) < 0) goto fail_bar0;
	
	if (wishbone_register(&dev->wb) < 0) {
		printk(KERN_ALERT PCIE_WB ": could not register wishbone bus\n");
		goto fail_bar1;
	}
	
	/* Initialize device */
	control = dev->pci_res[0].addr;
	iowrite32(0, control + WINDOW_OFFSET_LOW);
	iowrite32(0, control + CONTROL_REGISTER_HIGH);

	return pci_enable_device(pdev);

	/* cleaning up */
fail_bar1:
	destroy_bar(&dev->pci_res[1]);
fail_bar0:
	destroy_bar(&dev->pci_res[0]);
fail_msi:	
	pci_disable_msi(pdev);
fail_free:
	kfree(dev);
fail_out:
	return -EIO;
}

static void remove(struct pci_dev *pdev)
{
	/* clean up any allocated resources and stuff here.
	 * like call release_mem_region();
	 */

	struct pcie_wb_dev *dev;
	
	dev = pci_get_drvdata(pdev);
	wishbone_unregister(&dev->wb);
	
	destroy_bar(&dev->pci_res[1]);
	destroy_bar(&dev->pci_res[0]);
	
	pci_disable_msi(pdev);

	kfree(dev);
}

static struct pci_device_id ids[] = {
	{ PCI_DEVICE(PCIE_WB_VENDOR_ID, PCIE_WB_DEVICE_ID), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

static struct pci_driver pcie_wb_driver = {
	.name = PCIE_WB,
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};

static int __init pcie_wb_init(void)
{
	return pci_register_driver(&pcie_wb_driver);
}

static void __exit pcie_wb_exit(void)
{	
	pci_unregister_driver(&pcie_wb_driver);
}

MODULE_AUTHOR("Stefan Rauch <s.rauch@gsi.de>");
MODULE_DESCRIPTION("GSI Etherbone to Wishbone bridge driver");
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debugging information");
MODULE_LICENSE("GPL");
MODULE_VERSION(PCIE_WB_VERSION);

module_init(pcie_wb_init);
module_exit(pcie_wb_exit);
