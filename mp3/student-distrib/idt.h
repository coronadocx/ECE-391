/* header file for the iterupt descriptor table */

#ifndef IDT_H
#define IDT_H

#include "types.h"

#define EXCEPTION_COUNT 32
#define SYSTEMCALLNO 0x80
#define PITNO 0x20
/* function that sets up the IDT */
void initialize_IDT();

#endif
