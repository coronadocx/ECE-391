#include "paging.h"


static uint32_t page_directory[PAGE_SIZE] __attribute__((aligned(4096))); //Single page directory for system, 1024 entries
static uint32_t page_table_0M_4M[PAGE_SIZE] __attribute__((aligned(4096)));   //Page table for memory block 0-4MB, 1024 entries
static uint32_t new_page_table[PAGE_SIZE] __attribute__((aligned(4096)));   //Page table for memory block 0-4MB, 1024 entries
// The allignment allows there to be no first few bits in the address to the page table hence can be used to set specific bits/ eg: enable


/*
 * 	initialize_paging
 *
 *	INPT: none
 *	OUTPUT: none
 *	RETURN VALUE: none
 *	SIDE EFFECTS: Changes x86 control registers to allow paging
 *				  Populates one page directory
 *				  Populates 1023 page tables (the last one is for the kernel)
 *
 */
void paging_initialize()
{
	uint32_t i;
	//NOTE Only allocate ram when the system actually needs it --> Don't make 4MB of page tables, dummy

    //Populate the first page table
    for (i=0; i<PAGE_SIZE; i++) {
        switch(i) {
            case VMEM_PAGE :
                page_table_0M_4M[i] = (VID_START_ADDR | VMEM_OR_MASK);
                break;

            case T1_BUF_PAGE :
                page_table_0M_4M[i] = (T1_BUF_ADDR | VMEM_OR_MASK);
                break;

            case T2_BUF_PAGE :
                page_table_0M_4M[i] = (T2_BUF_ADDR | VMEM_OR_MASK);
                break;

            case T3_BUF_PAGE :
                page_table_0M_4M[i] = (T3_BUF_ADDR | VMEM_OR_MASK);
                break;

            default:
                page_table_0M_4M[i] = 0x0; // Set other pages as 'not present'

        }

    }


    //Populate the page_directory, identity map the video memory and the kernel memory
    for (i=0; i<PAGE_SIZE; i++) {
    	if (i==0)
    		page_directory[i] = (((uint32_t) page_table_0M_4M) | PTABLE_OR_MASK);	//Points to the first page table
    	else if (i==1)
    		page_directory[i] = (KER_START_ADDR | KMEM_OR_MASK);	//Points to the kernel page
    	else
    		page_directory[i] = 0x0;	//Set every other entry as "not present"
    }


    /*
     * 	Enable Paging
     *
     *	PG  = 1 - bit 31 of CR0
     * 	To set PE and PG:
     *		OR CR0 with 0x80000001
     *
     *	PAE = 0 - bit 5 of CR4
     *	PSE = 1 - bit 4 of CR4
     *  To clear PAE:
     *		AND CR4 with 0xFFFFFFF7
     * 	To set PSE:
     *		OR CR4 with 0x00000010
     */


    asm volatile("                      \n\
    	movl	%0, %%eax	            \n\
    	movl 	%%eax, %%cr3			\n\
        movl    %%cr4, %%eax            \n\
    	orl		$0x00000010, %%eax		\n\
    	movl 	%%eax, %%cr4 			\n\
        movl    %%cr0, %%eax            \n\
        orl     $0x80000000, %%eax      \n\
        movl    %%eax, %%cr0            \n\
    	"
    	:
    	: "r"(page_directory)
      : "eax","memory", "cc"
    );
}


/*
 *  paging_change_process
 *
 *  INPUT:          pid - ID of the destination process, (1-6)
 *  OUTPUT:         none
 *  RETURN VALUE:   0 on success, -1 on bad pid
 *  SIDE EFFECTS:   Changes physical mapping of v128MB
 *
 */
uint32_t paging_change_process(uint32_t pid)
{
    pid -= 1; // Is 1-indexed (1-6), want 0-indexed (0-5)

    if (pid < PIDMIN || pid > PIDMAX)
        return -1;

    //Point user memory (v128MB) to given process (p8MB+)
    page_directory[USER_MEM_LOCATION] = ((USR_START_ADDR + (PROCESS_SIZE*pid)) | UMEM_OR_MASK);

    _flush();   //Flush the TLB


    return 0;
}


/*
 *  mapvirtualtovideomemory
 *
 *  INPUT:          None
 *  OUTPUT:         None
 *  RETURN VALUE:   0
 *  SIDE EFFECTS:   Maps Virtual Memory 133MB to video memory by creating a new page table
 *
 */
uint32_t paging_vidmap()
{
    int32_t i;

    // Initialize new paging table
    for (i = 0; i < PAGE_SIZE; i++){
        if (i == ONEMB_PAGETABLE)
            new_page_table[i] = (VID_START_ADDR | UMEM_OR_MASK);
        else
            new_page_table[i] = 0x0;
    }

    // Assign page table to proper directory entry
    page_directory[VMEM_33] = (((uint32_t) new_page_table) | PTABLE_OR_MASK_USER);
    
    _flush();   // Flush the TLB

    return 0;
}


/*
 *  _flush
 *  INPUT: none
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper function, flushes the TLB
 */
void _flush()
{
    asm volatile("                      \n\
        movl    %%cr3, %%eax            \n\
        movl    %%eax, %%cr3            \n\
        "
        :
        :
        :"eax", "cc"
    );

    return;   
}

/*
 *  paging_set_write_to_videomem
 *  INPUT:  none
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: changes paging structure such virtural video memory maps to 
 *          physical video memory.
 */
void paging_set_write_to_videomem()
{
    page_table_0M_4M[VMEM_PAGE] = (VID_START_ADDR | VMEM_OR_MASK);
    new_page_table[ONEMB_PAGETABLE] = (VID_START_ADDR | UMEM_OR_MASK);
    
    _flush();
    
    return;
}


/*
 *  paging_set_write_to_buffer
 *  INPUT:  buffer - number (0-2) indicating which buffer to remap to
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: changes paging structure such that virtural video memory maps to 
 *          the provided backup buffer in physical memory.
 */
void paging_set_write_to_buffer(int32_t buffer)
{
    uint32_t addr;
    switch(buffer) {
        case 0:
            addr = T1_BUF_ADDR;
            break;

        case 1:
            addr = T2_BUF_ADDR;
            break;

        case 2:
            addr = T3_BUF_ADDR;
            break;

        default:
            addr = T1_BUF_ADDR;
    }

    page_table_0M_4M[VMEM_PAGE] = (addr | VMEM_OR_MASK);
    new_page_table[ONEMB_PAGETABLE] = (addr | UMEM_OR_MASK);

    _flush();

    return; 
}



