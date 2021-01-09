/* i8259.c - Functions to interact with the 8259 interrupt controller
 * vim:ts=4 noexpandtab
 */

#include "i8259.h"



/* Interrupt masks to determine which interrupts are enabled and disabled */
uint8_t master_mask=MASK_VALUE; /* IRQs 0-7  */
uint8_t slave_mask=MASK_VALUE;  /* IRQs 8-15 */


/*
 * i8259_init
 *   DESCRIPTION: Initialize the 8259 PIC
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  writes to slave and master ports to initialize the pic, initially masks all interupts
 */
void i8259_init(void) {

  // unsigned long flags;

  outb(MASK_VALUE,MASTER_8259_PORT_DATA);
  outb(MASK_VALUE,SLAVE_8259_PORT_DATA);

 //set up master
  outb(ICW1,MASTER_8259_PORT);
  outb(ICW2_MASTER,MASTER_8259_PORT_DATA); // master is at port 0x20 so this is that +1;
  outb(ICW3_MASTER ,MASTER_8259_PORT_DATA); // slave mapped to masters irq2
  outb(ICW4,MASTER_8259_PORT_DATA);

 //set up slave
  outb(ICW1,SLAVE_8259_PORT);
  outb(ICW2_SLAVE,SLAVE_8259_PORT_DATA); // slave is at port 0xA0 so this is that +1
  outb(ICW3_SLAVE,SLAVE_8259_PORT_DATA);
  outb(ICW4,SLAVE_8259_PORT_DATA);


  outb(master_mask,MASTER_8259_PORT_DATA);
  outb(slave_mask,SLAVE_8259_PORT_DATA);


}
/*
 * enable_irq
 *   DESCRIPTION:  Enable (unmask) the specified IRQ
 *   INPUTS: irq_number as uint32_t to enable
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  Enables interupts at the specificied irq
 */


void enable_irq(uint32_t irq_num) {
  /* Declaring port variable */
  int port=0x00;

 /* Master PIC handling */
if(irq_num>=0 && irq_num<SLAVE_IRQ_START){
  port = MASTER_8259_PORT_DATA;
  /* Unmask IRQ bit by bit shifting and negating */
  master_mask &= ~(1<<irq_num);   // REFERENCED THE LINUX INTIALIZATION CODE FROM BOOTLIN V5.0
  outb(master_mask,port);
}

/* Slave PIC handling */
if(irq_num>=SLAVE_IRQ_START && irq_num<SLAVE_IRQ_LAST){
  port = MASTER_8259_PORT_DATA;
  /* Unmasking Slave PIC port on Master */
  master_mask &= ~(1<<IRQ_SLAVE);
  outb(master_mask,port);
  port = SLAVE_8259_PORT_DATA;    // REFERENCED THE LINUX INTIALIZATION CODE FROM BOOTLIN V5.0
  /* Unmask IRQ bit by bit shifting and negating */
  slave_mask &= ~(1<<(irq_num-SLAVE_IRQ_START));
  outb(slave_mask,port);

}


}

/*
 * disable_irq
 *   DESCRIPTION:   Disable (mask) the specified IRQ
 *   INPUTS: irq_number as uint32_t to disable
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  disables  interupts at the specificied irq
 */

void disable_irq(uint32_t irq_num) {
  int port=0x00;  // set initially to zero and based on irq number decide what port to write to
if(irq_num>=0 && irq_num<SLAVE_IRQ_START){  // if irq number between 0 and slave irq start (8 )  write to master port to enable the required interrupt
  port =MASTER_8259_PORT_DATA;
  master_mask |= 1<<irq_num; // construct the master mask required to enable the interupt
    outb(master_mask,port);
}
if(irq_num>=SLAVE_IRQ_START && irq_num<SLAVE_IRQ_LAST){  // if irq number between slave irqstart  and slave irq last (15)  write to slave  port to enable the interrupt
  port=MASTER_8259_PORT_DATA;
  master_mask |= 1<<(2);    // REFERENCED THE LINUX INTIALIZATION CODE FROM BOOTLIN V5.0
  outb(master_mask,port); // write to master port
  port=SLAVE_8259_PORT_DATA;
  slave_mask |= 1<<(irq_num-SLAVE_IRQ_START); // construct the slave mask required to enable the interupt
  outb(slave_mask,port);  // write to slave port
}


}


/*
 * send_eoi
 *   DESCRIPTION:    Send end-of-interrupt signal for the specified IRQ
 *   INPUTS: irq_number for which eoi should be sent. type is an uint32_t
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  sends eoi at that port so that pic can process next interupts
 */
void send_eoi(uint32_t irq_num) {
 int returnvl = EOI | irq_num;
 if(irq_num>=SLAVE_IRQ_START){   // decide which port to send eoi to .
    returnvl = EOI | (irq_num-SLAVE_IRQ_START); // determine the returnvl that should be sent as EOI
    outb(returnvl,SLAVE_8259_PORT);

  }
  else{
 outb(returnvl,MASTER_8259_PORT);
}
}
