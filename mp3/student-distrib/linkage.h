#ifndef _LINKAGE_H
#define _LINKAGE_H

#include "types.h"
#include "pcb.h"
#include "x86_desc.h"
#include "pit.h"
#define USER_DS     0x002B

void save_esp_ebpasm();
/*
 *	pithandlerasm
 * 	DESCRIPTION: Assembly linkage function for pit
 *	INPUT: 	none
 *	OUTPUT: none
 *	RETURN: none
 */

 void pithandlerasm();
/*
 *	keyboardhandlerasm
 * 	DESCRIPTION: Assembly linkage function for keyboard.
 *	INPUT: 	none
 *	OUTPUT: none
 *	RETURN: none
 */
void keyboardhandlerasm();


/*
 * 	rtchandlerasm
 *	DESCRIPTION: Assembly linkage function for rtc.
 *	INPUT: 	none
 *	OUTPUT: none
 *	RETURN: none
 */
void rtchandlerasm();


/*
 * 	systemcallasm
 *	DESCRIPTION: Assembly linkage function for systemcalls.
 *	INPUT: 	none
 *	OUTPUT: none
 *	RETURN: none
 */
void systemcallasm();


/*
 * 	contextswitchasm
 *	DESCRIPTION: Assembly linkage function for context switches
 *	INPUT: 	eipval			- address to be returned to
 *			current_process	- number for the current process (1-5)
 *	OUTPUT: none
 *	RETURN: none
 */
void contextswitchasm(uint32_t eipval,pcb* current_process);


/*
 * 	restoreparent
 *	DESCRIPTION: Assembly linkage function to return to parent process
 *	INPUT: 	ebp		-	stack base pointer
 * 			esp		-	stack pointer
 *
 * 	OUTPUT: status	- 	return argument to be sent to parent
 *	RETURN: uint32_t
 */
uint32_t restoreparent(uint32_t ebp,uint32_t esp,uint32_t status);

#endif
