#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "lib.h"
#include "pcb.h"
#include "sys_call.h"

#define NUM_TERMS	3
#define KEYBOARD_BUFFER_LENGTH 128
#define FIRST_CURRENT_T 2
#define FIRST_VISIBLE_T 0


typedef struct terminal_struct{
	int8_t		lb[KEYBOARD_BUFFER_LENGTH];
	int32_t 	noc;
	int32_t 	esp;
	int32_t		ebp;
	int32_t   pid;
	int32_t screen_x;
	int32_t screen_y;
}terminal_t;

typedef struct sched{
  	terminal_t 	terminals[NUM_TERMS];
  	uint8_t * 	vid_bufs[NUM_TERMS];
  	int32_t 	visable_term;	// Terminal being viewed
  	int32_t 	current_term;	// Terminal being scheduled
  	pcb *		terminals_pcb[NUM_TERMS];	// Addresses for the PCBs
  	uint16_t	ss0[NUM_TERMS];	// Store terminal TSS values
  	uint16_t	esp0[NUM_TERMS];	// Store terminal TSS values
  	uint8_t		is_on[NUM_TERMS];
  	uint8_t		exe_bytes[SIZEOFOPERATIONSTABLE];
}scheduler_t;

/* sets the process ID of the relevant global_scheduler object */
void set_pid(uint32_t pid);
/* creates and populates global_scheduler object */
void init_global_scheduler();
/* switches which terminal is being viewed by the user */
void switch_terminals(int32_t next_terminal);
/* gets the noc value of the visible terminal */
int get_current_noc();
/* gets the value for the current terminal */
int32_t get_current_terminal();
/* gets the value for the visible terminal */
int32_t get_visable_terminal();
/* Copies values from the global linebuffer of visible terminal into the linebuffer for keyboard */
void set_line_buffer(char linebuffer[KEYBOARD_BUFFER_LENGTH]);
/* Copies values from the linebuffer into the global linebuffer for visible terminal */
void set_global_buffer(char linebuffer[KEYBOARD_BUFFER_LENGTH],int numberofchars);
/* Schedules next terminal to be processed, works with the PIT handler interrupts */
void scheduler_next();
/* Saves the registers %esp and %ebp before running a new process */
void save_esp_ebp(int32_t esp,int32_t ebp);

/* Gets the process ID of scheduled terminal */
uint32_t get_current_pid();
/* gets the current visible terminal screen_y */
int32_t get_global_screen_y();
/* gets the current visible terminal screen_x */
int32_t get_global_screen_x();
/* sets the current visible terminal screen_y */
void set_global_screen_y();
/* sets the current visible terminal screen_x */
void set_global_screen_x();
/* sets the current terminal screen_y */
void set_current_screen_y();
/* sets the current terminal screen_x */
void set_current_screen_x();
/* gets the current terminal screen_y */
int32_t get_current_screen_y();
/* gets the current terminal screen_y */
int32_t get_current_screen_x();


#endif
