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

#include "pci_char.h"

#define FALSE 0
#define TRUE 1
#define BUFCOUNT 100
#define TICK	8	/* 8 ns */


int main()
{
  int fd_irq;					/* device file descriptor for blocking read */
  int fd_rd;					/* device file descriptor for non-blocking read */
  unsigned int cnt_value_irq;	/* buffer to hold data */
  unsigned int cnt_value_rd;	/* buffer to hold data */
  unsigned int diff_buffer[BUFCOUNT];
  int status;					/* return status of system calls */
  int i;
  int arg;

  /* open irq device */
  status = fd_irq = open("/dev/pci_char0", O_RDWR);
  if (status == -1) {
    perror("error opening /dev/pci_char0");
    return(1);
  }
  
  /* open rd device after irq device */
  status = fd_rd = open("/dev/pci_char1", O_RDWR);
  if (status == -1) {
    perror("error opening /dev/pci_char1");
    return(1);
  }

  /* one tick is 8ns, so 1250000 ticks give 10µs */
  arg = 1250000;	// 0.01s / 8*E-9s
  //status = ioctl(fd_irq, SET_PERIOD, arg);
   // if (status == -1) {
  	//  perror("error from SET_PERIOD ioctl");
   //     return(1);
   // }

  //status = ioctl(fd_irq, GET_PERIOD, arg);
 // if (status == -1) {
	//  perror("error from GET_PERIOD ioctl");
   //   return(1);
  //} else
	//  printf("Period value is %d\n", status);


  for (i = 0; i < BUFCOUNT; i++) {
	  // blocking read
	  status = read(fd_irq, &cnt_value_irq, sizeof(int));
	  if (status == -1) {
		perror("error reading from fd_irq");
		return(1);
	  }

	  // non-blocking read
	  status = read(fd_rd, &cnt_value_rd, sizeof(int));
	  if (status == -1) {
		  perror("error reading from fd_rd");
		  return(1);
	   }

	  diff_buffer[i] = abs(cnt_value_irq-cnt_value_rd);

  }

  for (i = 0; i < BUFCOUNT; i++) {
	  printf("%d\n", diff_buffer[i]);
  }

  close(fd_irq);
  close(fd_rd);

  return(0);

  /* write some data */
//  *buf = 1;
//  while(1) {
//  	status = write(fd, buf, 1);
//  	if (status == -1) {
//  		perror("error writing to /dev/pci_char0");
//   		exit(1);
//  	}


  	
  	//usleep(125000);			/* wait for 250ms */
  //}

  /* close the device */
//  status = close(fd_irq);
//  if (status == -1) {
//    perror("error closing /dev/pci_char0");
//    return(1);
//  }

  /* and exit */

}
