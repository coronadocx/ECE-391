#include "pcb.h"
#include "lib.h"


/*
 *	get_pcb_address
 *	INPUT: none
 *	OUTPUT: none
 *	RETURN VALUE: address to the current stack's PCB
 */
pcb* get_pcb_address(){

  /* Here, we try to calculate which PCB this system call belongs to
   * We do this by taking the Top of Stack Pointer and ANDing it with a
     bitmask to get the nearest 8KB aligned page
     */
    pcb* current_pcb;
    int32_t x;


    current_pcb =(pcb*)(((int) &x) & (int32_t)(PCB_MASK));

  /* Bit-Mask 0xFFE000 should give us an 8KB aligned address of the PCB */
    return current_pcb;

};
