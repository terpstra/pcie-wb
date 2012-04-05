#ifndef PCI_CHAR_DRIVER_H_
#define PCI_CHAR_DRIVER_H_

#endif /*PCI_CHAR_DRIVER_H_*/

#include <linux/ioctl.h>

#define PCI_CHAR_MAGIC	'S'

#define SET_PERIOD			_IO(PCI_CHAR_MAGIC, 0)
#define GET_PERIOD			_IO(PCI_CHAR_MAGIC, 1)

#define PCI_CHAR_MAXNR	3

#define	ALTERA_ID 0x1172
#define DEVICE_ID 0x0004
#define FIRSTMINOR 0
#define DEV_COUNT 2

#define ONCHIP_RAM_OFFSET 	0x4000
#define SCU_ADDR			0x4030
#define SCU_TIMING			0x4040
#define SCU_WR_DATA 		0x4020
#define SCU_CTRL			0x4050
#define SCU_RD_DATA			0x4060
#define SCU_STATUS			0x4070
#define SCU_MASTER			0x1000000

#define	SCU_RD				'R'
#define SCU_WR				'W'
#define SCU_TM				'T'


#define PCIE_BASE			0x20000
#define MSI					0x050
#define TMR_BASE			0x24000
#define TMR_DIR				(TMR_BASE +	0x8)
#define TMR_EDGE_CAPTURE	(TMR_BASE + 0x18)
#define TMR_IRQ_MASK		(TMR_BASE +	0x10)
#define TMR_SNAP		0x24020
#define TMR_0_BASE		0x24080
#define TMR_0_CTRL		(TMR_0_BASE + 0x8)
#define TMR_0_PERIODL	(TMR_0_BASE + 0x10)
#define TMR_0_PERIODH	(TMR_0_BASE + 0x18)
#define TMR_0_SNAPL		(TMR_0_BASE + 0x20)
#define TMR_0_SNAPH		(TMR_0_BASE + 0x28)


#define EVENT_BUFFER_SIZE	100
#define IRQ_RATE			1250000



struct scu_cycle_data {
	unsigned int data;				/* 16 bit data */
	unsigned int addr;				/* 20 bit addr */
	char type;						/* SCU_RD, SCU_WR, SCU_TM */
};
