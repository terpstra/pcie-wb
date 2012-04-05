/*
 * syscalls.c
 * Program to illustrate common system calls. Doesn't actually
 * perform any useful function, but will later be expanded into
 * a program which does.
 */

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdlib.h>

#include "pci_char_driver.h"

#define FALSE 0
#define TRUE 1


#define BUFSIZE		1


int main()
{
  	int fd;						/* device file descriptor */
  	int i = 0, j = 0;
  	unsigned int wbuf[BUFSIZE];			/* buffer to hold data */
  	unsigned int rbuf[BUFSIZE];
  	int status;					/* return status of system calls */
  	char up = TRUE;
  	struct scu_cycle_data *scu_buf = malloc(sizeof(struct scu_cycle_data));

  	/* open device */
  	status = fd = open("/dev/pci", O_RDWR);
  	if (status == -1) {
    	perror("error opening /dev/pci");
    	exit(1);
  	}

	
	
	
	
 	/* reset of the SCU Macro */
	status = ioctl(fd, PCI_CHAR_RESET);
	if (status == -1) {
		perror("error from PCI_CHAR_RESET");
	    exit(1);
	}
	
	scu_buf->addr = 0x00002;
  		scu_buf->addr <<= 2;
  		scu_buf->data = 0xAFFE;
  		scu_buf->type = SCU_WR;
  		status = write(fd, scu_buf, sizeof(struct scu_cycle_data));
  		if (status == -1) {
  			perror("error writing to SCU bus!");
  			exit(1);
  	}
  	
  	//for (j=0; j<1000; j++) {
  		status = read(fd, scu_buf, sizeof(struct scu_cycle_data));
  		if (status == -1) {
  			perror("error reading from SCU bus!");
  			exit(1);
  		}
		
  	//}
  

  /* close the device */
  status = close(fd);
  if (status == -1) {
    perror("error closing /dev/pci");
    exit(1);
  }

  /* and exit */
  return(0);
}
