/* the .c file for the iterupt descriptor table */

#include "idt.h"
#include "x86_desc.h"
#include "lib.h"
#include "keyboard.h"
#include "linkage.h"
#include "i8259.h"
#include "rtc.h"
#include "sys_call.h"
#include "scheduler.h"



// idt index for keyboard and rtc
#define KEYBOARD  0x21
#define RTC 0x28
// Assembly functions that handle the stack when keyboard or rtc interuppts are generated
/*
 * handle0
 *   DESCRIPTION: function exception handler for divide by zero
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle0(){
    printf("general Protection exception\n");
    while(1);
}
/*
 * handle1
 *   DESCRIPTION: function exception handler for debug exception
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle1(){
    printf("Debug exception\n");
    while(1);
}
/*
 * handle2
 *   DESCRIPTION: function exception handler for NMI Interupt
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle2(){
    printf("NMI Interupt\n");
      while(1);

}
/*
 * handle3
 *   DESCRIPTION: function exception handler for Breakpoint Exception
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle3(){
    printf("Breakpoint Exception\n");
      while(1);
}
/*
 * handle4
 *   DESCRIPTION: function exception handler for Overflow Exception
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle4(){
    printf("Overflow Exception\n");
      while(1);
}
/*
 * handle5
 *   DESCRIPTION: function exception handler for BOUND Range Exceeded Exception
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle5(){
    printf("BOUND Range Exceeded Exception\n");
      while(1);
}
/*
 * handle6
 *   DESCRIPTION: function exception handler for Invalid Opcode Exception
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle6(){
    printf("Invalid Opcode Exception\n");
      while(1);
}
/*
 * handle7
 *   DESCRIPTION: function exception handler for Device Not Available Exception
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle7(){
    printf("Device Not Available Exception\n");
      while(1);
}
/*
 * handle8
 *   DESCRIPTION: function exception handler for Double Fault Exception
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle8(){
    printf("Double Fault Exception\n");
      while(1);
}
/*
 * handle9
 *   DESCRIPTION: function exception handler for Coprocessor Segment Overrun
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle9(){
    printf("Coprocessor Segment Overrun\n");
      while(1);
}
/*
 * handle10
 *   DESCRIPTION: function exception handler for Invalid TSS exception
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle10(){
    printf("Invalid TSS exception\n");
      while(1);
}
/*
 * handle14
 *   DESCRIPTION: function exception handler for Page Fault exception"
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void handle14(){
    printf("Page Fault exception\n");
int address;
    asm volatile("                \n\
  movl	%0, %%eax	            \n\
  movl 	%%cr2, %%eax			    \n\
  "
  : "=a"(address)
);
printf("The Exception occurred at:%x",address);

      halt('P');
}
/*
 * commonhandler
 *   DESCRIPTION: Temporary common function to handle all other exceptions from 1-32 for which handlers are not currently defined
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  handles exception and sqaushes program
 */
void commonehandler(){
  printf("exception raised");
    while(1);
}
/*
 * keyboard handler
 *   DESCRIPTION: handles the keyboard interupt
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS:  calls check input in keyboard.c
 */
void keyboard_handler(){
    check_input();
}
/*
 * rtc_handler
 *   DESCRIPTION: function that handles rtc interupts
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS: sets the appropriate ports and handles an rtc interupt
 */
void rtchandler(){
  /* referenced wiki.osdev.org/RTC#Interrupts_and_Registers_C*/

  rtc_interrupt_occurred = 1;
  outb(REG_C,RTC_CMD_PORT);
  inb(RTC_DATA_PORT);
  test_interrupts();
  send_eoi(RTC_IRQ_NO);
  send_eoi(IRQ_SLAVE);


}

/*
 * pit_handler()
 *   DESCRIPTION: function that handles PIT interrupts
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS: handles periodic PIT interrupts
 */
void pithandler(){

  /* Sending end of interrupt */
  send_eoi(0);
  scheduler_next();
}


/*
 * initialize_IDT
 *   DESCRIPTION: function that sets up the IDT
 *   INPUTS: None
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS: IDT is set up with appropriate values
 */

void initialize_IDT(){

   int i=0;

 for(i=0;i<EXCEPTION_COUNT;i++){
   idt[i].dpl=0;
   idt[i].present=1;
   idt[i].seg_selector=KERNEL_CS;
   idt[i].reserved4=0;
   idt[i].reserved3=0; //Changed this bit to make it an interrupt gate
   idt[i].reserved2=1;
   idt[i].reserved1=1;
   idt[i].reserved0=0;
   idt[i].size=1;

   /* Filling IDT entries with Exception Handlers */
   if(i==0)
   SET_IDT_ENTRY(idt[i],handle0);
   else if(i==1)          // here the magic numbers 1,2,3,4,5,6 etc are used to determine the appropriate handler for that index in the IDT
  SET_IDT_ENTRY(idt[i],handle1);
   else if(i==2)
   SET_IDT_ENTRY(idt[i],handle2);
   else if(i==3)
   SET_IDT_ENTRY(idt[i],handle3);
   else if(i==4)
   SET_IDT_ENTRY(idt[i],handle4);
   else if(i==5)
   SET_IDT_ENTRY(idt[i],handle5);
   else if(i==6)
   SET_IDT_ENTRY(idt[i],handle6);
   else if(i==7)
    SET_IDT_ENTRY(idt[i],handle7);
   else if(i==8)
     SET_IDT_ENTRY(idt[i],handle8);
   else if(i==14)
    SET_IDT_ENTRY(idt[i],handle14);
    else if(i==9)
    SET_IDT_ENTRY(idt[i],handle9);
   else if(i == 13)
     SET_IDT_ENTRY(idt[i],handle0);
   else
     SET_IDT_ENTRY(idt[i],commonehandler);

 }
/* setting the keyboard handler */
idt[KEYBOARD].dpl=0;
idt[KEYBOARD].present=1;
idt[KEYBOARD].seg_selector=KERNEL_CS;
idt[KEYBOARD].reserved4=0;
idt[KEYBOARD].reserved3=1;
idt[KEYBOARD].reserved2=1;
idt[KEYBOARD].reserved1=1;
idt[KEYBOARD].reserved0=0;
idt[KEYBOARD].size=1;
SET_IDT_ENTRY(idt[KEYBOARD],keyboardhandlerasm);

/* setting the RTC handler */
 idt[RTC].dpl=0;
 idt[RTC].present=1;
 idt[RTC].seg_selector=KERNEL_CS;
 idt[RTC].reserved4=0;
 idt[RTC].reserved3=1;
 idt[RTC].reserved2=1;
 idt[RTC].reserved1=1;
 idt[RTC].reserved0=0;
 idt[RTC].size=1;
 SET_IDT_ENTRY(idt[RTC],rtchandlerasm);

/* Setting up the System Call Entries */
idt[SYSTEMCALLNO].present=1;
idt[SYSTEMCALLNO].dpl=3; // called from user using INT
idt[SYSTEMCALLNO].seg_selector=KERNEL_CS;
idt[SYSTEMCALLNO].reserved4=0;
idt[SYSTEMCALLNO].reserved3=1;
idt[SYSTEMCALLNO].reserved2=1;
idt[SYSTEMCALLNO].reserved1=1;
idt[SYSTEMCALLNO].reserved0=0;
idt[SYSTEMCALLNO].size=1;
SET_IDT_ENTRY(idt[SYSTEMCALLNO],systemcallasm);


/* Setting up the Pit Handler */
idt[PITNO].present=1;
idt[PITNO].dpl=3; // called from user using INT
idt[PITNO].seg_selector=KERNEL_CS;
idt[PITNO].reserved4=0;
idt[PITNO].reserved3=0;
idt[PITNO].reserved2=1;
idt[PITNO].reserved1=1;
idt[PITNO].reserved0=0;
idt[PITNO].size=1;
SET_IDT_ENTRY(idt[PITNO],pithandlerasm);











}
