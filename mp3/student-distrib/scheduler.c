#include "scheduler.h"
#include "lib.h"
#include "paging.h"
#include "pcb.h"
#include "sys_call.h"
#include "filesystem.h"
#include "x86_desc.h"
#include "linkage.h"

static scheduler_t global_scheduler;

/*
 *  init_global_scheduler
 *  INPUT:  none
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: creates and populates global_scheduler object
 */
void init_global_scheduler()
{
  uint32_t i;
  dentry_t dir_entry;

  // uint8_t exe_bytes[SIZEOFOPERATIONSTABLE];
  int8_t exe_name[] = "shell";

  // Read Data
  (void)read_dentry_by_name(exe_name, &dir_entry);

  // Set bytes 24-27
  (void)read_data(dir_entry.inode_num, STARTOFEIPINEXECUTABLE, global_scheduler.exe_bytes, SIZEOFOPERATIONSTABLE);

  // Create PCBs for the 3 initial shells, set up paging for first terminal
  for (i=0; i<NUM_TERMS; i++) {

    // Initialize rest of scheduler object
    global_scheduler.terminals[i].noc = 0;
    memset(global_scheduler.terminals[i].lb,0,KEYBOARD_BUFFER_LENGTH);
    global_scheduler.terminals[i].pid=i+1;
    global_scheduler.terminals[i].screen_x=0;
    global_scheduler.terminals[i].screen_y=0;
    global_scheduler.vid_bufs[i] = (uint8_t*)(T1_BUF_ADDR + VMEM_SIZE*i);
    global_scheduler.is_on[i] = 0;
  }

  paging_change_process(1); // Make paging look at shell 1 initially

  global_scheduler.current_term = FIRST_CURRENT_T;
  global_scheduler.visable_term = FIRST_VISIBLE_T;

  return;
}

/*
 *  switch_terminals
 *  INPUT:  next_terminal
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: switches which terminal is being viewed by the user
 *  SIDE EFFECT: modifies video memory and terminal video buffers
 */
void switch_terminals(int32_t next_terminal)
{
  cli();
  int32_t  prev_terminal;
  uint8_t* prev_terminal_buf;
  uint8_t* next_terminal_buf;
  uint8_t* vmem;

  // Get previous terminal and its buffer
  prev_terminal = global_scheduler.visable_term;
  prev_terminal_buf = global_scheduler.vid_bufs[prev_terminal];

  if (next_terminal == prev_terminal)
    return;

  // Set next terminal and get its buffer
  global_scheduler.visable_term = next_terminal;

  update_cursor();
  next_terminal_buf = global_scheduler.vid_bufs[next_terminal];

  vmem = (uint8_t*)VID_START_ADDR;

  // // cli();
  paging_set_write_to_videomem();
  memcpy(prev_terminal_buf, vmem, VMEM_SIZE);   // Savem vmem to previous terminal buffer
  memcpy(vmem, next_terminal_buf, VMEM_SIZE);   // Write next terminal buffer to vmem
  if (global_scheduler.current_term != global_scheduler.visable_term)
    paging_set_write_to_buffer(global_scheduler.current_term);
  sti();


}


/*
 *  get_current_noc
 *  INPUT:  none
 *  OUTPUT: none
 *  RETURN: the noc value of the current terminal
 *  EFFECT: helper, gets the noc value of the current terminal
 */
int32_t get_current_noc()
{
  return global_scheduler.terminals[global_scheduler.visable_term].noc;
}


/*
 *  get_current_terminal()
 *  INPUT:  none
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper, gets the value for the current terminal
 */
int32_t get_current_terminal()
{
  return global_scheduler.current_term;
}

/*
 *  set_current_screen_x(int x)
 *  INPUT:  int x - x Value to set the current activate terminal screen_x
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper, Sets the active terminall screenx
 */

void set_current_screen_x(int x)
{
  global_scheduler.terminals[global_scheduler.current_term].screen_x=x;
}

/*
 *  set_current_screen_y(int y)
 *  INPUT:  int y - y Value to set the current activate terminal screen_y
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper, Sets the active terminall screen_y
 */

void set_current_screen_y(int y)
{
  global_scheduler.terminals[global_scheduler.current_term].screen_y=y;
}

/*
 *  get_current_screen_x()
 *  INPUT:  none
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper, gets the current active terminal screen_X
 */

int32_t get_current_screen_x()
{
  return global_scheduler.terminals[global_scheduler.current_term].screen_x;
}


/*
 *  get_current_screen_y()
 *  INPUT:  none
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper, gets the current active terminal screen_y
 */

int32_t get_current_screen_y()
{
  return global_scheduler.terminals[global_scheduler.current_term].screen_y;
}

/*
 *  get_visile_terminal()
 *  INPUT:  none
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper, gets the value for the visible terminal
 */

int32_t get_visable_terminal()
{
  return global_scheduler.visable_term;
}

/*
 *  set_global_screen_x(int x)
 *  INPUT:  int x
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper, sets the value for the visible terminal screen_x
 */

void set_global_screen_x(int x)
{
  global_scheduler.terminals[global_scheduler.visable_term].screen_x=x;
}

/*
 *  set_global_screen_y(int y)
 *  INPUT:  int y
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper, sets the value for the visible terminal screen_y
 */

void set_global_screen_y(int y)
{
  global_scheduler.terminals[global_scheduler.visable_term].screen_y=y;
}

/*
 *  get_global_screen_x()
 *  INPUT:  none
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper, gets the current visible terminal screen_y
 */

int32_t get_global_screen_x()
{
  return global_scheduler.terminals[global_scheduler.visable_term].screen_x;
}

/*
 *  get_global_screen_y()
 *  INPUT:  none
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: helper, gets the current visible terminal screen_y
 */

int32_t get_global_screen_y()
{
  return global_scheduler.terminals[global_scheduler.visable_term].screen_y;
}

/*
 *  set_line_buffer
 *  INPUT:  char linebuffer[128]
 *  OUTPUT: TODO
 *  RETURN: none
 *  EFFECT: Copies values from the global linebuffer into the linebuffer for keyboard
 */
void set_line_buffer(char linebuffer[KEYBOARD_BUFFER_LENGTH])
{
  memcpy(linebuffer,global_scheduler.terminals[global_scheduler.visable_term].lb,global_scheduler.terminals[global_scheduler.visable_term].noc);
}

/*
 *  set_global_buffer
 *  INPUT:  char linebuffer[128], int numberofcharss
 *  OUTPUT: TODO
 *  RETURN: none
 *  EFFECT:  Copies values from the linebuffer into the global linebuffer for visible terminal
 */
void set_global_buffer(char linebuffer[KEYBOARD_BUFFER_LENGTH],int numberofchars)
{
  global_scheduler.terminals[global_scheduler.visable_term].noc=numberofchars;
  memcpy(global_scheduler.terminals[global_scheduler.visable_term].lb,linebuffer,numberofchars);
}
void set_pid(uint32_t pid){
    global_scheduler.terminals[global_scheduler.current_term].pid=pid;
}


/*
 *  set_current_pid
 *  INPUT:  none
 *  OUTPUT: TODO
 *  RETURN: none
 *  EFFECT:  Gets the process ID of scheduled terminal
 */
uint32_t get_current_pid(){
    return global_scheduler.terminals[global_scheduler.current_term].pid;
}
/*
 *  scheduler_next
 *  INPUT:  none
 *  OUTPUT: none
 *  RETURN: none
 *  EFFECT: Increments the counter which keeps track of which terminal is
 *          being processed. Then, depending on which terminal is being viewed,
 *          may change paging structure to show or hide accordingly
 */
void scheduler_next()
{
  // cli();
  save_esp_ebpasm();
  uint32_t pid;
  uint8_t exe_name[] = "shell";
  // uint32_t newx,newy;

  global_scheduler.current_term = (global_scheduler.current_term+1)%NUM_TERMS;  // Set next terminal


  pid = global_scheduler.terminals[global_scheduler.current_term].pid;




  // If current terminal is being viewed
  if (global_scheduler.current_term == global_scheduler.visable_term){
    paging_set_write_to_videomem(); // Have virtural map to video memory
  }
  else{
    paging_set_write_to_buffer(global_scheduler.current_term);  // Have virtural map to buffer
  }

  if (global_scheduler.is_on[global_scheduler.current_term] == 0){
    global_scheduler.is_on[global_scheduler.current_term] = 1;
    if(execute(exe_name)<0){
      printf("failed");
    }

  }

  else{
    // Switch to next process
    paging_change_process(pid);

    tss.ss0 = KERNEL_DS;
    tss.esp0 = (END_KMEM - (pid*KERNEL_MEM_SIZE));


    (void)restoreparent(
      global_scheduler.terminals[global_scheduler.current_term].ebp,
      global_scheduler.terminals[global_scheduler.current_term].esp,
      0
      );
  }
  // sti();
}

//helper function
/*
 * save_esp_ebp
 *   DESCRIPTION: Saves the registers %esp and %ebp before running a new process
 *   INPUTS: pcb* a -> Address of the pcb structure
 *          int32_t esp -> esp value
 *          int32_t ebp -> ebp value
 *   OUTPUTS:None
 *   RETURN VALUE: None
 *   SIDE EFFECTS: Saves the %ebp and %esp registers in the pcb struct
 */

void save_esp_ebp(int32_t esp,int32_t ebp){
  global_scheduler.terminals[global_scheduler.current_term].esp=esp;
  global_scheduler.terminals[global_scheduler.current_term].ebp=ebp;
  return;
}
