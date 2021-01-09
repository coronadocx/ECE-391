/* Assembly linkage for interrupt handlers */

/* This is explicitly done to save FLAGS and registers on the stack
*  upon encountering an interrupt
*/
.data
  /* some data allocations */
  USER_DS =0x002B
  ARGSIZE = 12
  USER_CS =0x0023
  ESPARG = 0x83ffffc
  MAX_SYS_CALL=10
  MIN_SYS_CALL=0

.text

/* Defining global keyboard and rtc handlers */
.global keyboardhandlerasm
.global rtchandlerasm
.global systemcallasm
.global contextswitchasm
.global restoreparent
.global pithandlerasm
.global save_esp_ebpasm

/*
 * 	save_esp_ebpasm
 *	DESCRIPTION: Assembly linkage function for saving the values of ebp and esp
 *	INPUT: 	esp			- ESP of process
 *			    ebp	    - EBP of process  
 *	OUTPUT: none
 *	RETURN: none
 */
save_esp_ebpasm:
xorl %edx, %edx
xorl %ecx, %ecx
movl %esp, %ecx
movl %ebp, %edx


pushl %edx
pushl %ecx

call save_esp_ebp

addl $8, %esp
ret

/*
 * 	contextswitchasm
 *	DESCRIPTION: Assembly linkage function for context switches
 *	INPUT: 	eipval			- address to be returned to
 *			current_process	- number for the current process (1-5)
 *	OUTPUT: none
 *	RETURN: none
 */
contextswitchasm:
    xorl %edx,%edx
    movl 8(%esp),%edx
    xorl %ecx,%ecx
    movl %esp,%ecx

    /* code to save parent process stuff */
    movl $USER_DS, %eax
    movw %ax, %ds
    movw %ax, %es
    movw %ax, %fs
    movw %ax, %gs

    pushl %ebp
    pushl %ecx
    pushl %edx
    call save_parent_registers
    add $ARGSIZE,%esp

    xorl %edx,%edx
    movl 4(%esp),%edx

    pushl $USER_DS
    pushl $ESPARG
    pushfl
    popl %eax
    orl $0x200, %eax
    pushl %eax
    pushl $USER_CS
    pushl %edx


    iret

    ret


/*
 * 	restoreparent
 *	DESCRIPTION: Assembly linkage function to return to parent process
 *	INPUT: 	ebp		-	stack base pointer
 * 			esp		-	stack pointer
 *
 * 	OUTPUT: status	- 	return argument to be sent to parent
 *	RETURN: uint32_t
*/
restoreparent:
     /* extract args and restore the parents */
     movl 4(%esp),%ebp  // move first arg
     movl 8(%esp),%esp // move second arg
     movl 12(%esp),%eax // move third arg

     ret

/* keyboardhandlerasm
* INPUTS -- none
* OUTPUTS -- none
* RETURN Value -- none
* Side Effects -- Saves registers and flags, calls the C keyboard handler,
* pops all registers and flags and does an iret to return to the main program at the
* location it was interrupted at
*/
keyboardhandlerasm:
   pushal
   call keyboard_handler
   popal
   iret


/* pithandlerasm
 * INPUTS -- none
 * OUTPUTS -- none
 * RETURN Value -- none
 * Side Effects -- calls c pithandler
 */
pithandlerasm:
      # Saving esp and ebp

      pushal
      call pithandler
      popal
      iret

/* rtchandlerasm
 * INPUTS -- none
 * OUTPUTS -- none
 * RETURN Value -- none
 * Side Effects -- Saves registers and flags, calls the C rtc handler,
 * pops all registers and flags and does an iret to return to the main program at the
 * location it was interrupted at
 */
rtchandlerasm:
      pushal

      call rtchandler

      popal

      iret



/* halt_handlerasm
 * linkage function to call halt
 */



halt_handlerasm:
   call halt
   jmp RETURN


/* execute_handlerasm
    * linkage function to call execute
*/

execute_handlerasm:
   call execute

   jmp RETURN



/* read_handlerasm
 * linkage function to call read
*/
read_handlerasm:
  call read
  jmp RETURN




  /* write_handlerasm
   * linkage function to call write
   */
write_handlerasm:
  call write
  jmp RETURN



/* open_handlerasm
 * linkage function to call open
*/
open_handlerasm:
  call open
  jmp RETURN

/* close_handlerasm
 * linkage function to call close
 */
close_handlerasm:
    call close
    jmp RETURN


/* getargs_handlerasm
 * linkage function to call getargs
 */
getargs_handlerasm:
    call getargs
    jmp RETURN



/* vidmap_handlerasm
 * linkage function to call close
 */
vidmap_handlerasm:
    call vidmap
    jmp RETURN

/* set_handlerasm
* linkage function to call close
*/
set_handlerasm:
  call set_handler
  jmp RETURN


/* sigreturn_handlerasm
* linkage function to call close
*/
sigreturn_handlerasm:
  call sigreturn
  jmp RETURN



/*
 * 	systemcallasm
 *	DESCRIPTION: Assembly linkage function for systemcalls.
 *	INPUT: 	none
 *	OUTPUT: none
 *	RETURN: none
*/
systemcallasm:

          cmpl $MIN_SYS_CALL,%eax
          jle  DEFAULTERROR
          cmpl $MAX_SYS_CALL,%eax
          jg   DEFAULTERROR
          pushfl
        #  pushal
        # Manually push all values of registers

          pushl %edx
          pushl %ecx
          pushl %ebx

          pushl %esi
          pushl %edi
          pushl %ebp

        # %eax not saved here


          pushl %edx
          pushl %ecx
          pushl %ebx

          subl $1,%eax


          jmp  *jumptable(,%eax,4) // magic number 4 is just the number of bytes

DEFAULTERROR:
          movl $-1,%eax
          iret

RETURN:
        # Tearing down syscall args
          add $ARGSIZE,%esp
    /* Popal first and then PopFl */
    # popal
    # Restoring all values except %eax
    # We need %eax for the return value
         popl %ebp
         popl %edi
         popl %esi

         popl %ebx
         popl %ecx
         popl %edx

         popfl
         iret
jumptable:
   .long halt_handlerasm,execute_handlerasm,read_handlerasm,write_handlerasm,open_handlerasm,close_handlerasm,getargs_handlerasm,vidmap_handlerasm,set_handlerasm,sigreturn_handlerasm
.end
