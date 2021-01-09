/* Header file for the FileSystem */

#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#define FILE_NAME_SIZE          32
#define DENTRY_RESERVED         6

#define FS_BLOCK_SIZE           4096
#define BYTES_IN_INT            4
#define ABS_BLK_OFFSET        (FS_BLOCK_SIZE/BYTES_IN_INT)

#define BOOT_BLOCK_SIZE         64
#define BOOT_BLOCK_DIR_ENTRIES  (BOOT_BLOCK_SIZE-2)
#define NUM_INODES              64


#define INODE_MAX_D_BLOCKS      1023
#define MAX_FILE_SIZE           (INODE_MAX_D_BLOCKS*FS_BLOCK_SIZE)

#define BB_INODE_OFFSET         1
#define BB_DBLOCK_OFFSET        2

#define MAX_FILE_NUM            62

#define SKIP_TO_DIR_ENTRIES     16

#define VAL_ONE                  1

#define FTYPE_REGULAR            2

#define FILETYPE_INDEX           1




/* Change these includes to what is necessary */
#include "types.h"



/* Struct for a Directory Entry */
typedef struct dentry {
    char fname[FILE_NAME_SIZE];
    uint32_t file_type;
    uint32_t inode_num;
    uint32_t reserved[DENTRY_RESERVED];
} dentry_t;


/* Giving the multiboot_struct into fs_open for the boot block addr */
extern int32_t set_mod_start(unsigned int mod_start);

/* sets the global dentry based on filename */
extern int32_t fs_open(const int8_t* filename);

/* resets the global dentry struct */
extern int32_t fs_close();

/* populates the buffer based on the file descriptor */
extern int32_t fs_read(int32_t fd, void* buf, int32_t nbytes);

/* Read only file system. just does error checking */
extern int32_t fs_write(int32_t fd, void* buf, int32_t nbytes);

/* Searches for a directory entry by name, copies over elements and returns a pointer to the directory entry */
extern int32_t read_dentry_by_name(const int8_t* fname, dentry_t* dentry);

/* Searches for a directory entry by index of the directory entries in boot block(not inode#), copies over elements and returns a pointer to the directory entry */
extern int32_t read_dentry_by_index(uint32_t index, dentry_t* dentry);

/* Searches for a directory entry by inode#, copies over a number of elements from an offset into buf* */
extern int32_t read_data(uint32_t inode, uint32_t offset, uint8_t* buf, uint32_t length);

/* Reads and lists the contents of the directory on the screen */
extern int32_t dir_read(int fd, void* buf, int32_t nbytes);
/* Closes the directory */
extern int32_t dir_close();
/* opens the directory */
extern int32_t dir_open(const int8_t* filename);



/* return the size of a particular file */
extern int32_t get_filesize(uint32_t inode);

/* Returns 0, read-only filesystem */
extern int32_t dir_write(int32_t fd,const void* buf,int32_t nbytes);

#endif
