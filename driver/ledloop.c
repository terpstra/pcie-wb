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

#define FALSE 0
#define TRUE 1


int main()
{
  int fd;						/* device file descriptor */
  //int arg;					/* argument for ioctl call */
  unsigned char buf[1];			/* buffer to hold data */
  int status;					/* return status of system calls */
  char up = TRUE;

  /* open device */
  status = fd = open("/dev/pci_char0", O_RDWR);
  if (status == -1) {
    perror("error opening /dev/pci_char0");
    exit(1);
  }

  /* set a parameter using ioctl call */
  
  //arg = 8000; /* sampling rate */
  /*status = ioctl(fd, SOUND_PCM_WRITE_RATE, &arg);
  if (status == -1) {
    perror("error from SOUND_PCM_WRITE_RATE ioctl");
    exit(1);
  }*/

  /* read some data */
  //status = read(fd, buf, sizeof(buf));
  //if (status == -1) {
  //  perror("error reading from /dev/dsp");
  //  exit(1);
  //}

  /* write some data */
  *buf = 1;
  while(1) {
  	status = write(fd, buf, 1);
  	if (status == -1) {
  		perror("error writing to /dev/pci_char0");
   		exit(1);
  	}

	if (buf[0] == 0x80)		/* turn if highest bit is set */
		up = FALSE;
	else if (buf[0] == 0x01)
		up = TRUE;

	if (up == TRUE)
  		buf[0] <<= 1;
	else
		buf[0] >>= 1;
  	
  	usleep(125000);			/* wait for 250ms */
  }

  /* close the device */
  status = close(fd);
  if (status == -1) {
    perror("error closing /dev/pci_char0");
    exit(1);
  }

  /* and exit */
  return(0);
}
