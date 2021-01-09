#include "pit.h"
#include "lib.h"
#include "i8259.h"


// void init_pit(){
// /* referenced wiki.osdev.org/RTC to understand the rtc */
// outb(REG_B,RTC_CMD_PORT);
// char prev = inb(RTC_DATA_PORT);
// outb(REG_B,RTC_CMD_PORT);
// outb(prev|RTC_MASK,RTC_DATA_PORT);
// enable_irq(RTC_IRQ_NO);
//   return;
// }


/*
 *	init_pit
 *	INPUT: uint32_t rate
 *	OUTPUT: none
 *  SIDE EFFECTS: Sets the rate of the PIT
 *	RETURN VALUE: none
 */
void init_pit(uint32_t rate){

uint32_t divisor = 1193182 /rate;
outb(0x36,0x43);
outb((uint8_t) (divisor & 0xFF),0x40);
outb((uint8_t) ( (divisor>>8) & 0xFF ),0x40);
enable_irq(0);
printf("Printing in pit");
}
