/* c file for rtc */

#include "i8259.h"
#include "rtc.h"

/*
 * init_rtc
 *   DESCRIPTION: Function to initialize the RTC, including setting the correct
 *                Registers A and B and setting the frame rate
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: None
 *   SIDE EFFECTS:  Initializes the RTC, and sets the current frequency
 *                  Given by the lower 4 bits of register A
 */
void init_rtc(){
/* referenced wiki.osdev.org/RTC to understand the rtc */
outb(REG_B,RTC_CMD_PORT);
char prev = inb(RTC_DATA_PORT);
outb(REG_B,RTC_CMD_PORT);
outb(prev|RTC_MASK,RTC_DATA_PORT);
enable_irq(RTC_IRQ_NO);
  return;
}


/*
 * 	rtc_read
 *	DESCRIPTION: Blocks current process until next RTC interrupt
 * 	INPUT:	fd		- RTC file descriptor
 * 	OUTPUT:	buf		- UNUSED
 *			nbytes	- UNUSED
 * 	RETURN VALUE: Returns 0 once an interrupt has occurred
 * 	SIDE EFFECT: NONE
 *  NOTE: This does not read the current RTC frequency
 */
int32_t rtc_read(int32_t fd,void* buf,int32_t nbytes)
{
  rtc_interrupt_occurred = 0;

	while  (rtc_interrupt_occurred==0) { //wait for rtc interrupt to occur
	}

	return 0;
}

/*
 * 	rtc_write
 * 	DESCRIPTION: Changes the RTC's frequency
 * 	INPUT:	fd		- File descriptor for the RTC
 * 			buf		- Pointer to a 4-byte integer specifying the interrupt rate
 *	OUTPUT: nbytes  - Holds the number of bytes written (always 4 on success)
 *	RETURN VALUE: 0 on success, -1 on failure
 *	SIDE EFFECT: Modifies the RTC
 */
int32_t rtc_write(int32_t fd,const void * buf, int32_t nbytes)
{
	int32_t tmp;
	int8_t rate = INIT_RATE;
	int8_t prev;
  int32_t* buf2= (int32_t*) buf;

	//Make sure input is valid
	if ( buf2 == NULL) {
		return -1;
	}
  else if(*((int32_t*)buf2) & ((*buf2)-1) || *buf2 > MAX_FREQ){
    return -1;
  }

	// Calculate the RTC rate setting
	tmp = MAX_FREQ;
	while (tmp != *buf2){
		rate++;
		tmp = tmp >> 1;
	}

	// Change RTC to calculated rate
	cli();										// Disable interrupts
	outb(REG_A_NMI_OFF,RTC_CMD_PORT);			// Select A, NMI off
	prev = inb(RTC_DATA_PORT);					// Collect old state
	outb(REG_A_NMI_OFF,RTC_CMD_PORT);			// Select A again
	outb((prev & MASK_LOWER) | rate,RTC_DATA_PORT);	// Write new rate to RTC
	sti();										// Enable interrupts

	return 0;
}


/*
 * 	rtc_open
 * 	DESCRIPTION: Sets RTC frequency to 2Hz then returns
 * 	INPUT: filename - UNUSED
 *	OUTPUT: none
 *	RETURN VALUE: 0 always
 *	SIDE EFFECT: Modities the
 */
int32_t rtc_open(const uint8_t * filename)
{
	//Set the RTC to 2Hz

  int32_t default_freq = DEFAULT_FREQ;
  rtc_write(TEMP_ZERO,&default_freq,TEMP_ZERO);

	return 0;
}


/*	NOTE: Will be implemented in later checkpoint
 * 	rtc_close
 *  DESCRIPTION: Does nothing
 * 	INPUT: none
 *	OUTPUT:none
 *	RETURN VALUE: 0 always
 *	SIDE EFFECT: none
 */
int32_t rtc_close(int32_t fd)
{
	return 0;
}



/*
	For CP 5 -- How to virturalize RTC:
	Set RTC to highest frequency
	Have a handful of counters for a number of different RTCs
	each interrupt, increment the counter
	when a counter reaches a certain value (which emulates lower frequency), reset and raise a new interrupt
*/
