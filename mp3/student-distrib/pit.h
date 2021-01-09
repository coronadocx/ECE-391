#ifndef PIT_H
#define PIT_H
#include "lib.h"

/* function that handles PIT interrupts */
void pithandler();
/* Initializes the PIT and sets the rate */
void init_pit(uint32_t rate);


#endif
