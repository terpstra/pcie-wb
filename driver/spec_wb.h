#ifndef SPEC_WB_DRIVER_H
#define SPEC_WB_DRIVER_H

#include "wishbone.h"

#define SPEC_WB "spec_wb"
#define SPEC_WB_VERSION	"0.1"

#define SPEC_WB_VENDOR_ID	0x10dc
#define	SPEC_WB_DEVICE_ID	0x018d

#define WB_BAR		0
#define WB_OFFSET	0x80000
#define WB_LOW		0x3fffc

/* One per BAR */
struct spec_wb_resource {
	unsigned long start;			/* start addr of BAR */
	unsigned long end;			/* end addr of BAR */
	unsigned long size;			/* size of BAR */
	void *addr;				/* remapped addr */
};

/* One per physical card */
struct spec_wb_dev {
	struct pci_dev* pci_dev;
	struct spec_wb_resource pci_res[3];
	int    pci_irq[4];
	
	struct wishbone wb;
	struct mutex mutex; /* only one user can open a cycle at a time */
	unsigned int window_offset;
	unsigned int low_addr, width, shift;
};

#endif
