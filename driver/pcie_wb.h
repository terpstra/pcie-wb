#ifndef PCIE_WB_DRIVER_H
#define PCIE_WB_DRIVER_H

#include "wishbone.h"

#define PCIE_WB "pcie_wb"
#define PCIE_WB_VERSION	"0.1"

#define PCIE_WB_VENDOR_ID	0x1172
#define	PCIE_WB_DEVICE_ID	0x0004

#define CONTROL_REGISTER_HIGH	0
#define CONTROL_REGISTER_LOW	4
#define ERROR_FLAG_HIGH		8
#define ERROR_FLAG_LOW		12
#define WINDOW_OFFSET_HIGH	16
#define WINDOW_OFFSET_LOW	20

#define WINDOW_HIGH	0xFFFF0000UL
#define WINDOW_LOW	0x0000FFFCUL

/* One per BAR */
struct pcie_wb_resource {
	unsigned long start;			/* start addr of BAR */
	unsigned long end;			/* end addr of BAR */
	unsigned long size;			/* size of BAR */
	void *addr;				/* remapped addr */
};

/* One per physical card */
struct pcie_wb_dev {
	struct pci_dev* pci_dev;
	struct pcie_wb_resource pci_res[2];
	int    pci_irq[4];
	
	struct wishbone wb;
	struct mutex mutex; /* only one user can open a cycle at a time */
	unsigned int window_offset;
	unsigned int low_addr, width, shift;
};

#endif
