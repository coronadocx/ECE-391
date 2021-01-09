#include "linkage.h"
#include "sys_call.h"
#include "filesystem.h"
#include "pcb.h"
#include "x86_desc.h"
#include "paging.h"
#include "types.h"
#include "rtc.h"
#include "lib.h"
#include "scheduler.h"
/* Tables of function pointers */
void* rtctable[SIZEOFOPERATIONSTABLE]={&rtc_open,&rtc_read,&rtc_write,&rtc_close};
void* filetable[SIZEOFOPERATIONSTABLE]={&fs_open,&fs_read,&fs_write,&fs_close};
void* directorytable[SIZEOFOPERATIONSTABLE]={&dir_open,&dir_read,&dir_write,&dir_close};
void* stdin_table[SIZEOFOPERATIONSTABLE]={&terminal_open,&terminal_read,NULL,&terminal_close};
void* stdout_table[SIZEOFOPERATIONSTABLE]={&terminal_open,NULL,&terminal_write,&terminal_close};
int8_t processes_running[NUMBEROFPROCESSESSUPPORTED] = {0,0,0,0,0,0};
//int8_t processes_running[NUMBEROFPROCESSESSUPPORTED] = {NOTINUSE,NOTINUSE}; //For a maximum of two programs



/*
 *  set_up_stdin
 *   DESCRIPTION: Function to handle the stdin file descriptor
 *   INPUTS: pcb* curr_pcb -- Address of Current Process Control Block
 *   OUTPUTS:None
 *   RETURN VALUE: NONE
 *   SIDE EFFECTS: sets fd entry 0 in  current pcb
 */
void set_up_stdin(pcb* current_process){
  // SETS UP STD IN
  current_process->fd_array[0].operationstable=stdin_table;
  current_process->fd_array[0].inode_num=INVALIDORFAIL;
  current_process->fd_array[0].file_pos=DEFAULTFILEOFFSET;
  current_process->fd_array[0].flags[0]=INUSE;

}
/*
 *  set_up_stdout
 *   DESCRIPTION: Function to handle the stdout file descriptor
 *   INPUTS: pcb* curr_pcb -- Address of Current Process Control Block
 *   OUTPUTS:None
 *   RETURN VALUE: NONE
 *   SIDE EFFECTS: sets fd entry 1 in  current pcb
 */
void set_up_stdout(pcb* current_process){
  // SETS UP STD IN
  current_process->fd_array[1].operationstable=stdout_table;
  current_process->fd_array[1].inode_num=INVALIDORFAIL;
  current_process->fd_array[1].file_pos=DEFAULTFILEOFFSET;
  current_process->fd_array[1].flags[0]=INUSE;
}
/*
 * set parent
 *   DESCRIPTION: Function to handle the file descriptor on an open system call
 *   INPUTS: pcb* curr_pcb -- Address of Current Process Control Block
 *           current_pid -- id for process
 *           parent_pcb -- Address of parent process control block
             parent_pid -- address of parent id
 *   OUTPUTS:None
 *   RETURN VALUE: none
 *   SIDE EFFECTS: sets parent for current process
 */
void set_parent(pcb* current_process,int32_t current_pid,pcb* parent_pcb,int32_t parent_pid){
  if(current_pid!=1){
    current_process->parent =parent_pcb ;
    current_process->parent_process_id=parent_pid;
  }
  else{
    current_process->parent=NULL;
  }
}
/*
 *  set_up_fdsandargs
 *   DESCRIPTION: Function to handle the file descriptor array and set up the args for current process
 *   INPUTS: pcb* curr_pcb -- Address of Current Process Control Block
 *           int32_t arg_size -- argument size
             int32_t arg_idx -- index from where args start
             int32_t arg_arr -- the argument array
 *   OUTPUTS:None
 *   RETURN VALUE: None
 *   SIDE EFFECTS:set up the args for current process
 */
void set_up_fdsandargs(pcb* current_process,int32_t arg_size,int32_t arg_idx,int8_t arg_arr[MAXBUFSIZEEXECUTE]){
  int i=2;
  int j;
  /* sets the other file descriptors to not be in use */
  while(i<ENTRIESINFDARRAY){
  current_process->fd_array[i].file_pos=DEFAULTFILEOFFSET;
  current_process->fd_array[i].flags[0]=NOTINUSE;
  i=i+1;
  }

  // Clean the old buffer because we do not create new structures
  for (i=0; i< PCB_MAX_ARGS; i++) {
    current_process->arg_arr[i] = '\0';
  }

  // Store the rest of the command as args in the current process
  current_process->arg_size = arg_size;
  for (i=0, j=arg_idx; j < arg_size && arg_arr[j] != '\0'; i++,j++) { // W_SPACE ? it was zero anyways
    current_process->arg_arr[i] = arg_arr[j];
  }
}
/*
 * fd_array_handle_open
 *   DESCRIPTION: Function to handle the file descriptor on an open system call
 *   INPUTS: pcb* curr_pcb -- Address of Current Process Control Block
 *           file_type -- File type of file
 *           inode_num -- Inode Number of file
 *   OUTPUTS:None
 *   RETURN VALUE: Returns fd of file on success, else -1
 *   SIDE EFFECTS:
 */

int32_t fd_array_handle_open(pcb* curr_pcb, int32_t file_type, int32_t inode_num){

  if(curr_pcb == NULL)
    return INVALIDORFAIL;;

  if(file_type != 0 && file_type != 1 && file_type != 2)
    return INVALIDORFAIL;

  int i;
  /* i starts at 2 because 0 and 1 are occupied by stdin and stdout */
  for(i = FDSTART; i < ENTRIESINFDARRAY; i++){

    /* Found an empty file descriptor spot */
    if(curr_pcb->fd_array[i].flags[IN_USE_INDEX] == NOTINUSE){
      break;
    }

  }

  /* If no file descriptor is free, return -1 */
  if(i >= ENTRIESINFDARRAY){
    return INVALIDORFAIL;
  }

  /* Populating the file descriptor struct */
  /* File operations table pointer */
  if(file_type == FILETYPEDEVICE){ // file type 0 is rtc device type
    curr_pcb->fd_array[i].operationstable = (rtctable);
  }
  else if(file_type == FILETYPEDIRECTORY){ // file type 1 is directoy
    curr_pcb->fd_array[i].operationstable = (directorytable);
  }
  else if(file_type == FILETYPEGENERAL){ // file type 2 is regular file
    curr_pcb->fd_array[i].operationstable =  (filetable);
  }

  /* Inode */
  curr_pcb->fd_array[i].inode_num = inode_num;

  /* File position */
  curr_pcb->fd_array[i].file_pos = DEFAULTFILEOFFSET;

  /* Filling flags */
  curr_pcb->fd_array[i].flags[IN_USE_INDEX] = INUSE;
  curr_pcb->fd_array[i].flags[FTYPE_INDEX] = file_type;

  return i;
}

   /*
    * fd_array_handle_close
    *   DESCRIPTION: Function to handle the file descriptor on a close  system call
    *   INPUTS: int32_t fd -- Index of file to be closed
    *           pcb* curr_pcb -- Address of Current Process Control Block
    *   OUTPUTS:None
    *   RETURN VALUE: Returns 0 on successful closure, else -1
    *   SIDE EFFECTS:
    */

int32_t fd_array_handle_close(int32_t fd, pcb* curr_pcb){

  if(fd<0){
    return INVALIDORFAIL;
  }
  /* Check for valid fd */
  if(fd < FDSTART || fd > FDEND)
    return INVALIDORFAIL;

  if(curr_pcb == NULL)
    return INVALIDORFAIL;

  /* Populate and mark fd as not in use */
  curr_pcb->fd_array[fd].operationstable = NULL;
  curr_pcb->fd_array[fd].inode_num = 0;
  curr_pcb->fd_array[fd].file_pos = DEFAULTFILEOFFSET;
  curr_pcb->fd_array[fd].flags[IN_USE_INDEX] = NOTINUSE;
  curr_pcb->fd_array[fd].flags[FTYPE_INDEX] = 0;
  curr_pcb->fd_array[fd].flags[RSVD_INDEX_0] = 0;
  curr_pcb->fd_array[fd].flags[RSVD_INDEX_1] = 0;

  /* Return 0 on success */
  return 0;


}

/*
 * open
 *   DESCRIPTION: open system call. opens a file in the fd array
 *   INPUTS: filename
 *   OUTPUTS:None
 *   RETURN VALUE: int32_t fd -> file descriptor index of opened file
 *   SIDE EFFECTS:
 */

int32_t open(const uint8_t* filename){

  if(filename == NULL || strlen((int8_t*)filename)==0){
    return INVALIDORFAIL;
  }
  /* Directory Entry struct */
  dentry_t dir_entry;
 /* Error handling if file not found */
  if(read_dentry_by_name((int8_t*)filename,&dir_entry)==-1){
    return INVALIDORFAIL;
  }
  int32_t fd_opened;

  /* Get address of relevant PCB */
  pcb* curr_pcb;
  curr_pcb = get_pcb_address();

  fd_opened = fd_array_handle_open(curr_pcb, dir_entry.file_type, dir_entry.inode_num);
  /* Error handling for bad fd value */
  if(fd_opened == INVALIDORFAIL)
    return INVALIDORFAIL;

 // call using the function pointer table
 int32_t (*fun_ptr)(const uint8_t*);
 fun_ptr = (curr_pcb->fd_array[fd_opened].operationstable)[FILE_OPS_OPEN];
 (*fun_ptr)(filename);

 /* Return index of the file descriptor opened */
  return fd_opened;

}


/*
 * close
 *   DESCRIPTION: close  system call. tearsdown stack and closes the file in the fd array
 *   INPUTS: fd -> FD index to be closed
 *   OUTPUTS:None
 *   RETURN VALUE: int32_t indicating success or failure
 *   SIDE EFFECTS:
 */


int32_t close(int32_t fd){

  if(fd<0){
    return INVALIDORFAIL;
  }
  /* Check for valid fd */
  if(fd < FDSTART || fd > FDEND)
    return INVALIDORFAIL;

  /* Get address of relevant PCB */
    pcb* curr_pcb;
    curr_pcb = get_pcb_address();

    /* Check if file descriptor index not in use */
    if(curr_pcb->fd_array[fd].flags[IN_USE_INDEX] == NOTINUSE)
      return -1;
    /* Check if operationstable* is valid */
    if(curr_pcb->fd_array[fd].operationstable == NULL)
      return -1;
      /* Else call the close function from the operations table */

    int32_t (*fun_ptr)(int32_t);
    fun_ptr = (curr_pcb->fd_array[fd].operationstable)[FILE_OPS_CLOSE];
    (*fun_ptr)(fd);

    return fd_array_handle_close(fd, curr_pcb);

}

/*
 * read
 *   DESCRIPTION: read system call. Calls the appropriate read function from file ops table
 *   INPUTS: fd -> File descriptor index
 *           buf -> buffer to read into
 *           nbytes -> number of bytes to read
 *   OUTPUTS:None
 *   RETURN VALUE: Returns the number of bytes read, or -1 on failure
 *   SIDE EFFECTS:
 */


int32_t read(int32_t fd,void*buf,int32_t nbytes){

  /* Initial Error handling */
  if(fd<0 || fd>FDEND){
    return INVALIDORFAIL;
  }

  if(buf == NULL)
    return INVALIDORFAIL;

  if(fd == FD_STDOUT)
    return -1;

  /* Get address of relevant PCB */
  pcb* curr_pcb;
  curr_pcb = get_pcb_address();

  /* File needs to be in use for read */
  if(curr_pcb->fd_array[fd].flags[IN_USE_INDEX] == NOTINUSE)
    return INVALIDORFAIL;

  /* Function pointer points to read of file operations table of fd*/
  int32_t (*fun_ptr)(int32_t, void*, int32_t);
  fun_ptr = (curr_pcb->fd_array[fd].operationstable)[FILE_OPS_READ];

	/* Handles file and directory reads */
	int retval;
	retval = (*fun_ptr)(fd, buf, nbytes);
  return retval;

}


/*
 * write
 *   DESCRIPTION: Write system call. Calls the appropriate write function from file ops table
 *   INPUTS: fd -> File descriptor index
 *           buf -> buffer to write from
 *           nbytes -> number of bytes to write
 *   OUTPUTS:None
 *   RETURN VALUE: Returns 0 on success, -1 on failure
 *   SIDE EFFECTS:
 */

int32_t write(int32_t fd, const void*buf,int32_t nbytes){
    if(fd<=0 || fd>FDEND){
      return INVALIDORFAIL;
    }
	 if(buf == NULL)
    return INVALIDORFAIL;

	  /* Get address of relevant PCB */
  pcb* curr_pcb;
  curr_pcb = get_pcb_address();

  /* File needs to be in use for write */
  if(curr_pcb->fd_array[fd].flags[IN_USE_INDEX] == NOTINUSE)
    return INVALIDORFAIL;

  int32_t (*fun_ptr)(int32_t, void*, int32_t);
  fun_ptr = (curr_pcb->fd_array[fd].operationstable)[FILE_OPS_WRITE];
   return (*fun_ptr)(fd, (void*) buf, nbytes);


}


/*
 * execute
 *   DESCRIPTION: execute system call. Starts a process and runs it
 *   INPUTS: command -> Executable to run
 *   OUTPUTS:None
 *   RETURN VALUE: int32_t indicating success or failure
 *   SIDE EFFECTS:
 */

int32_t execute(const uint8_t* command){
  int32_t i;
  int32_t j;
  int8_t buf[MAXBUFSIZEEXECUTE];
  dentry_t dir_entry;
  int32_t arg_size; // XXX tmp var to hold array size
  int32_t arg_idx;  // XXX tmp var to hold command starting idx
  int8_t arg_arr[MAXBUFSIZEEXECUTE]; // XXX tmp var to hold cmd

  /* Error handling for NULL */
  if(command==NULL){
    return -1;
  }
  //Make sure buffer isn't too thicc
  if (strlen((int8_t*)command) > MAXBUFSIZEEXECUTE){
    printf("Bad Length\n" );
    return INVALIDORFAIL;
  }

  for (i=0; i<MAXBUFSIZEEXECUTE; i++)
    arg_arr[i] = '\0';

  // XXX save the command buffer locally copies the entire command
  arg_size = 0;
  while(command[arg_size] != '\n' && command[arg_size] != '\0') {
    arg_arr[arg_size] = command[arg_size];
    arg_size++;
  }

  arg_arr[arg_size+1] = '\0';



  i=0;


  //leading spaces
  while(command[i]==' '){
    i=i+1;
  }

  // parse the filename from the command
  for(j=0; command[i]!=' ' && command[i]!='\n' && command[i]!='\0';i++, j++){
    buf[j]=command[i];
  }

  // Set the last character to sentinal
  if (j == MAXBUFSIZEEXECUTE)
    j = MAXBUFSIZEEXECUTE-1;
  buf[j]='\0';
  arg_idx = j+1;

  // check if program exists
  if(read_dentry_by_name(buf,&dir_entry)==-1){
    return INVALIDORFAIL ;
  }

  /* checks if you can read data */
  int8_t check_buf[METADATAEXECUTESIZE];
  uint8_t executablebytes[SIZEOFOPERATIONSTABLE];
  if( read_data(dir_entry.inode_num,0,(uint8_t*) check_buf,METADATAEXECUTESIZE) != METADATAEXECUTESIZE){
    return INVALIDORFAIL;
  }

  /* check if the file is an executable. these four magic numbers are used to check if a file is an executable  */
  if(check_buf[0]!=0x7f || check_buf[1]!=0x45||check_buf[2]!=0x4C||check_buf[3]!=0x46){  // we use these magic numbers only once and hence they are not macros
    return INVALIDORFAIL ;
  }

  /* set the bytes 24 to 27 */
  read_data(dir_entry.inode_num, STARTOFEIPINEXECUTABLE, (uint8_t*) executablebytes,SIZEOFOPERATIONSTABLE);

  /* create pcb and set parent pcb */
  i=1;
  uint32_t current_pid=-1;
  while(i<(NUMBEROFPROCESSESSUPPORTED+1)){
    if(processes_running[i-1]==0){
       current_pid=i;
       processes_running[i-1]=INUSE;
       break;
    }
    i=i+1;
  }

  if(current_pid==-1){
    printf("Maximum number of processes reached\n");
        return INVALIDORFAIL;
  }
  set_pid(current_pid);
  pcb* parent_pcb;
  int parent_pid;
  int*y;
  int x;
  y=&x;
  paging_change_process(current_pid);
  /*  read data into virtual address 128 MB */
  read_data(dir_entry.inode_num,0,(uint8_t*) VIRTUALADDRESSFOREXECUTABLEDATA,EXTENDEDPAGESIZE);
  /* Bit-Mask 0xFFE000 should give us an 8KB aligned address of the PCB */

  /* this part is to create the pcb */
  pcb* current_process=(pcb*)((int)END_KMEM-(current_pid+1)*PCB_SIZE); // start at 8MB-2*8kb

  if(current_pid>NUM_TERMINALS){
    parent_pcb = get_pcb_address();
    parent_pid = (END_KMEM - (unsigned int) parent_pcb)/(PCB_SIZE) - 1;
    current_process->parent =parent_pcb ;
    current_process->parent_process_id=parent_pid;
  }
  else{
    current_process->parent=NULL;
  }

  // SETS UP STD IN
 set_up_stdin(current_process);
  // SETS UP STD OUT
  set_up_stdout(current_process);

  current_process->process_id=current_pid;
set_up_fdsandargs(current_process,arg_size,arg_idx,arg_arr);

  /* this part alters the TSS */
  tss.ss0 = KERNEL_DS;
  tss.esp0 = END_KMEM - (current_pid)*KERNEL_MEM_SIZE; // This will be the processe's kernelmode stack

  /* lets do the context switch!!!!!! */
  contextswitchasm(*((uint32_t*) executablebytes),current_process);

return current_process->status;
}


/*
 * halt
 *   DESCRIPTION: halt system call. tearsdown stack and restores to parent process
 *   INPUTS: status -> character
 *   OUTPUTS:None
 *   RETURN VALUE: int32_t indicating success or failure
 *   SIDE EFFECTS:
 */

int32_t halt(uint8_t status){

  /* get the address of the process to halt */
  pcb* curr_pcb;
  curr_pcb = get_pcb_address();

  // Don't close the first 3 processes (shells)
  if (curr_pcb->process_id < (NUM_TERMINALS + 1)){
    printf("Cannot close base process -- ");
    cli();
    processes_running[(curr_pcb->process_id-1)] = 0;
    execute((uint8_t*)"shell");
  }

  processes_running[curr_pcb->process_id-1]=NOTINUSE;
  uint32_t parentid=curr_pcb->parent_process_id;
  int i=0;
  // close all the open files
  while(i<ENTRIESINFDARRAY){
    if(curr_pcb->fd_array[i].flags[0]==INUSE){
      close(i);
    }
    i=i+1;
  }
  if(curr_pcb->process_id>NUM_TERMINALS){
    curr_pcb->parent->status = (uint32_t) status ;
  }
  set_pid(parentid);
 // restore paging
  paging_change_process(parentid);
  tss.ss0=KERNEL_DS; // switch back to parent stack
  tss.esp0= END_KMEM - (parentid)*KERNEL_MEM_SIZE;

  return restoreparent(curr_pcb->parent_ebp,curr_pcb->parent_esp,(uint32_t) status); // assembly function to restor ebp and esp





}

//helper function
/*
 * save_parent_registers
 *   DESCRIPTION: Saves the parent registers %esp and %ebp before running a new process
 *   INPUTS: pcb* a -> Address of the pcb structure
 *          int32_t esp -> esp value
 *          int32_t ebp -> ebp value
 *   OUTPUTS:None
 *   RETURN VALUE: None
 *   SIDE EFFECTS: Saves the %ebp and %esp registers in the pcb struct
 */

void save_parent_registers(pcb* a,int32_t esp,int32_t ebp){
a->parent_esp=esp;
a->parent_ebp=ebp;
return ;
}

////////////////////////////////CP4/////////////////////////////////


/*
 * vidmap
 *   DESCRIPTION: Maps the text mode video memory into user space at a pre-set
 *                virtual address
 *   INPUTS: uint8_t screen_start Location where the virtual address must be stored
 *   OUTPUTS:None
 *   RETURN VALUE: None
 *   SIDE EFFECTS: None
 */

int32_t vidmap(uint8_t ** screen_start)
{
  //TODO

// checks if the pointer passed is a valid user level pointer
// does this by checking if its between 128 MB and 132 MB in virtual user memory
// this checking is very wrong. Need to check specific parameters. AAh will do.

if(screen_start==NULL){
  return -1;
}

uint32_t page_dir_idx = ((MASK_FOR_PAGEDIRIDX) & (uint32_t)screen_start) >> SHIFT_FOR_PAGEDIRIDX;
// uint32_t page_dir_idx = (uint32_t) screen_start >> 22;
// Basically if its not 32 then return
if(page_dir_idx !=CHECK_USER_VMEM ){
  return -1;
}

paging_vidmap();

uint32_t vmemlocation = VMEM_133MB;

memcpy(screen_start,&vmemlocation,NUMBER_BYTES_INT);

return 0;
}


/*
 * set_handler
 *   DESCRIPTION: Set_handler system call which currently only
 *                returns -1. Will be done for EC
 *   INPUTS: int32_t signum,void* handler_address
 *   OUTPUTS:-1
 *   RETURN VALUE: None
 *   SIDE EFFECTS: None
*/

int32_t set_handler(int32_t signum,void* handler_address){

  return -1;

}

/*
 * sigreturn
 *   DESCRIPTION: sigreturn system call which currently
 *                only returns -1. Will be done for EC
 *   INPUTS: void
 *   OUTPUTS:-1
 *   RETURN VALUE: None
 *   SIDE EFFECTS: None
*/

int32_t sigreturn(void){
  return -1;

}
/*
 *  getargs
 *  INPUT:  nbytes  - number of bytes/chars to read into the buffer
 *
 *  OUTPUT: buf     - buffer that args will be stored into
 *
 *  RETURN: 0   - success
 *          -1  - buffer is too small or noarg or NULL buffer
 *
 *  EFFECT: Reads the program's command line arguments into a buffer
 */
int32_t getargs(uint8_t * buf, int32_t nbytes)
{
    pcb * curr_pcb = get_pcb_address();
  // NOTE: need the last element in buf for sentinel
  if (buf == NULL || nbytes < 0 || (nbytes-1) < curr_pcb->arg_size)
    return INVALIDORFAIL;




  strncpy((int8_t*)buf, (int8_t*)curr_pcb->arg_arr, nbytes); // Copy args into buf
//  buf[nbytes] = '\0'; // Insert nullbyte sentinel

  return 0;
}
