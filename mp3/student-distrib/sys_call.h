#ifndef SYS_CALL_H
#define SYS_CALL_H
#include "types.h"
#include "pcb.h"


#define PCB_MASK    0xFFE000
#define END_KMEM    0x800000
#define PCB_SIZE    0x002000
#define KERNEL_MEM_SIZE 0x2000

#define IN_USE_INDEX  0
#define FTYPE_INDEX   1
#define RSVD_INDEX_0  2
#define RSVD_INDEX_1  3


#define FILE_OPS_OPEN  0
#define FILE_OPS_READ  1
#define FILE_OPS_WRITE 2
#define FILE_OPS_CLOSE 3

#define NUMBEROFPROCESSESSUPPORTED 6
#define ENTRIESINFDARRAY 8
#define SIZEOFOPERATIONSTABLE 4
#define NOTINUSE 0
#define INUSE 1
#define MAXBUFSIZEEXECUTE 128

#define STARTOFEIPINEXECUTABLE 24
#define EXTENDEDPAGESIZE 0x400000
#define VIRTUALADDRESSFOREXECUTABLEDATA 0x8048000
#define INVALIDORFAIL -1
#define FILETYPEDEVICE 0
#define FILETYPEDIRECTORY 1
#define FILETYPEGENERAL 2
#define DEFAULTFILEOFFSET 0
#define METADATAEXECUTESIZE 40
#define FDSTART 2
#define FDEND 7


#define VMEM_133MB 0x8500000
#define SHIFT_FOR_PAGEDIRIDX 22
#define MASK_FOR_PAGEDIRIDX 0xFFC00000
#define CHECK_USER_VMEM 32
#define NUMBER_BYTES_INT 4

#define FD_STDIN        0
#define FD_STDOUT       1

#define NUM_TERMINALS   3




#define W_SPACE	0

extern int8_t processes_running[NUMBEROFPROCESSESSUPPORTED];


/* Function to handle the file descriptor on an open system call */
int32_t fd_array_handle_open(pcb* curr_pcb, int32_t file_type, int32_t inode_num);
/* Function to handle the file descriptor on a close system call */
int32_t fd_array_handle_close(int32_t fd, pcb* curr_pcb);
/* halt system call. tearsdown stack and restores to parent process */
int32_t halt(uint8_t status);
/* execute system call. Starts a process and runs it */
int32_t execute(const uint8_t* command);
/* Write system call. Calls the appropriate write function from file ops table */
int32_t write(int32_t fd, const void*buf,int32_t nbytes);
/* read system call. Calls the appropriate read function from file ops table */
int32_t read(int32_t fd,void*buf,int32_t nbytes);
/* close  system call. tearsdown stack and closes the file in the fd array */
int32_t close(int32_t fd);
/* open system call. opens a file in the fd array */
int32_t open(const uint8_t* filename);
/* Reads the program's command line arguments into a buffer */
int32_t getargs(uint8_t * buf, int32_t nbytes);
/* Maps the text mode video memory into user space at a pre-set virtal address */
int32_t vidmap(uint8_t ** screen_start);
/* Set_handler system call which currently only returns -1. Will be done for EC */
int32_t set_handler(int32_t signum,void* handler_address);
/* sigreturn system call which currently only returns -1. Will be done for EC */
int32_t sigreturn(void);

void set_up_stdin(pcb* current_process);
void set_up_stdout(pcb* current_process);
void set_up_fdsandargs(pcb* current_process,int32_t arg_size,int32_t arg_idx,int8_t arg_arr[MAXBUFSIZEEXECUTE]);
// void set_process(int32_t pid);

#endif
