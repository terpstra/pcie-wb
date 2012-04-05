/*
 *  Accessing a PCI memory region using mmap() on /dev/mem
 *
 *  	Compile with 'gcc -Wall -O file.c -lpci -o peeker' (-O *is* mandatory)
 *
 *	Run with './peeker x y z' arguments can be derived from 'lspci -tvv'
 *
 *	Make sure (with lspci -vvs x:y.z) that "Region 0" is memory and that 
 *	 "Region 1" is I/O.
 */

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <pci/pci.h>
#include <sys/io.h>
#include <sys/mman.h>
#include <string.h>

#include "pci_char.h"

#define PCI_MEM_LEN (1<<18) 

#define COUNT 100000

int main( int argc, char *argv[]){


  unsigned char* ptr_to_pci_mem;
	unsigned int pci_mem_addr;
 	int buffer[COUNT];
	int i;
	FILE* fd;


  /* Check IO permissions to be able to open /dev/mem
   */
  if(iopl(3)){
    printf("Cannot get I/O permissions (being root helps)\n");
    return -1;
  }

  pci_mem_addr = 0xfeac0000;
  //pci_mem_addr += SCU_MASTER;

  fd = fopen ( "/dev/mem", "r+w");

  /* ...and map the PCI memory area (man 2 mmap for more info)
   */ 
  ptr_to_pci_mem =  mmap(	NULL,
					 						PCI_MEM_LEN,
					 						PROT_READ|PROT_WRITE,
			 								MAP_SHARED,
										 	fileno(fd), pci_mem_addr);



  /* Voila! Two bytes of PCI memory out from the board
   * You can also use the memcpy() family of functions
   */
  printf("Memory pointer: %p\n", ptr_to_pci_mem);
  printf("PCI memory @%#x\n", pci_mem_addr);
  for (i = 0; i< 30; i++) {
	printf("addr %p : %#x\n",ptr_to_pci_mem + PCIE_BASE + 0x800+ i*4, *(ptr_to_pci_mem + PCIE_BASE + 0x800 + i*4) );
  }
  //printf("The 2nd byte on PCI memory is : %#x\n", *(ptr_to_pci_mem+2) );
  
  //memcpy(buffer, ptr_to_pci_mem, COUNT);

//for (i=0; i < COUNT; i+=1) {
 // 	buffer[i] = *(ptr_to_pci_mem++);
	//	//*(ptr_to_pci_mem+1) = i;
  //}
  

  munmap(ptr_to_pci_mem, PCI_MEM_LEN);
  fclose(fd);
  return 0;
}
