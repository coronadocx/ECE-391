#ifndef PCB_H
#define PCB_H

#include "types.h"
#include "rtc.h"
#include "filesystem.h"
#include "terminal.h"

#define PCB_MASK          0xFFE000
#define FD_ARR_SIZE       8
#define PCB_MAX_ARGS          128

/* Operation Jumptable Struct */
typedef struct operation_table{
 int32_t (*open)(int8_t* filename);
 int32_t (*read)(int32_t fd,void* buf, int32_t nbytes);
 int32_t (*write)(int32_t fd,const void* buf, int32_t nbytes);
 int32_t (*close)(int32_t fd);
}otable_t;

/* struct for a file descriptor */
typedef struct file_descriptor{
  void** operationstable;
  int32_t inode_num;
  int32_t file_pos;
  uint8_t flags[4]; // Define file type
}file_desc;

/* PCB struct decleration*/
typedef struct task_struct{
  file_desc fd_array[FD_ARR_SIZE];
  struct task_struct* parent;
  int32_t process_id;
  int32_t parent_esp;
  int32_t parent_ebp;
  int32_t parent_process_id;
  int32_t arg_size;                 // Chars in arg array
  uint8_t arg_arr[PCB_MAX_ARGS];    // Arg array
  int32_t status ;

}pcb;

/* Function to get the address of the current pcb */
 pcb* get_pcb_address();

#endif
