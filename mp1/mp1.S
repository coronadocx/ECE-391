
.data					# section declaration

        # Useful offset constants for accessing members of a 
        # struct mp1_blink_struct structure
        LOCATION   = 0    
        ON_CHAR    = 2
        OFF_CHAR   = 3 
        ON_LENGTH  = 4
        OFF_LENGTH = 6
        COUNTDOWN  = 8
        STATUS     = 10
        NEXT       = 12

        STRUCT_SIZE = 16
    
        MAX_LOCATION = 2000 #custom

# Pointer to head of list (initialized to NULL)
mp1_list_head:
        .long   0

.text					# section declaration

# Export the function symbol names

.global mp1_rtc_tasklet
.global mp1_ioctl

# void mp1_poke(void);
#
# Interface: Register-based arguments (not C-style)
#    Inputs: %cl  - The byte you wish to write
#            %eax - Offset from the start of video memory that you wish
#                   to write to
#   Outputs: Text-mode video screen is written to at location %eax with
#            the byte in %cl
# Registers: Clobbers EDX
mp1_poke:
        
	movl    vmem_base_addr(,1),%edx
	movb    %cl,(%edx,%eax,1)
	ret

/* Input - void*/
/* Output - void*/
/* Decrement counter field for each struct in list*/
mp1_rtc_tasklet:
    pushl   %ebp                        # make stackframe
    movl    %esp, %ebp
    pushl   %ebx
    pushl   %esi
    pushl   %edi

    movl    mp1_list_head, %eax         # eax <- list_head

mp1_rtc_tasklet_loop:
    cmpl    $0, %eax
    je      mp1_rtc_tasklet_end         # when list ptr is NULL, end
    
    movw    COUNTDOWN(%eax), %bx        # decrement countdown
    decw    %bx
    movw    %bx, COUNTDOWN(%eax)

    cmpw    $0, %bx                     # if countdown != 0, continue
    jne     mp1_rtc_tasklet_cont

    movw    STATUS(%eax), %cx           # cx <- blink.status
    cmpw    $0, %cx                     # if status = 1, switch from on to off
    jne     mp1_rtc_tasklet_on2off

    incw    %cx                         # else status = 0, switch from off to on
    movw    %cx, STATUS(%eax)           # status <- 1
   
    movw    ON_LENGTH(%eax), %cx
    movw    %cx, COUNTDOWN(%eax)        # countdown <- on_length

    movl    %eax, %ebx                  
    xorl    %eax, %eax
    xorl    %ecx, %ecx
    movw    LOCATION(%ebx), %ax
    movb    ON_CHAR(%ebx), %cl
    shlw    $1, %ax
    call    mp1_poke                    # Render new character
    movl    %ebx, %eax                  
    jmp     mp1_rtc_tasklet_cont

mp1_rtc_tasklet_on2off:
    movw    $0, STATUS(%eax)            # status <- 0
    
    movw    OFF_LENGTH(%eax), %cx
    movw    %cx, COUNTDOWN(%eax)        # countdown <- off_length

    movl    %eax, %ebx
    xorl    %eax, %eax
    xorl    %ecx, %ecx
    movw    LOCATION(%ebx), %ax
    shlw    $1, %ax
    movb    OFF_CHAR(%ebx), %cl
    call    mp1_poke                    # Render new character
    movl    %ebx, %eax
    
mp1_rtc_tasklet_cont:
    movl    NEXT(%eax), %eax            # eax <- ptr to next struct in list
    jmp     mp1_rtc_tasklet_loop        # repeat loop

mp1_rtc_tasklet_end:
    popl    %edi
    popl    %esi
    popl    %ebx
    leave
    ret

/* Input - cmd, arg*/
/* Output - void*/
/* Dispatcher for the various ioctl functions*/
mp1_ioctl:
    movl    8(%esp), %eax               # move cmd arg to eax
    cmpl    $0, %eax                    # if cmd < 0, ret -1
    jl      mp1_ioctl_OOB

    cmpl    $3, %eax                    # if cmd > 3, ret -1 
    jg      mp1_ioctl_OOB

    jmp     *mp1_ioctl_JT(,%eax,4)      # jmp to given command

mp1_ioctl_OOB:
    movl    $-1, %eax                   # if bad input, return -1


/* Input  - Userlevel pointer to blink struct*/
/* Output - Status int*/
mp1_ioctl_add:
    pushl   %ebp                        # Create stackframe
    movl    %esp, %ebp
    pushl   %ebx
    pushl   %esi
    pushl   %edi

    movl    8(%ebp), %esi               # esi <- blink struct ptr
    cmpl    $0, %esi                    # nullcheck provided pointer
    je      mp1_ioctl_add_fail

    movw    LOCATION(%esi), %ax         # make sure struct location in range
    cmpw    $MAX_LOCATION, %ax
    jge     mp1_ioctl_add_fail

    pushl   $STRUCT_SIZE                # No registers in use, just push args
    call    mp1_malloc                  # Make memory allocation with malloc
    addl    $4, %esp                    # pop args from stack
    cmpl    $0, %eax
    je      mp1_ioctl_add_fail          # if malloc fails, throw error
    movl    %eax, %ebx                  # ebx <- memory allocation
    
    pushl   $STRUCT_SIZE                # push number of bytes to copy
    pushl   %esi                        # push from buffer address
    pushl   %ebx                        # push to buffer address
    call    mp1_copy_from_user          # Copy blink struct with copy_from_user
    addl    $12, %esp                   # pop args from stack
    cmpl    $0, %eax
    jne     mp1_ioctl_add_copy_fail     # if copy_from_user fails, throw error and free memory

    movw    ON_LENGTH(%ebx), %ax        # blink.countdown <- blink.onlength
    movw    %ax, COUNTDOWN(%ebx)    
    movw    $1, STATUS(%ebx)            # blink.status    <- 1
    
    movl    mp1_list_head, %eax         # insert struct at head of list
    movl    %eax, NEXT(%ebx)
    movl    %ebx, mp1_list_head

    xorl    %eax, %eax
    xorl    %ecx, %ecx
    movw    LOCATION(%ebx), %ax        
    shlw     $1, %ax
    movb    ON_CHAR(%ebx), %cl          # call poke to display character
    call    mp1_poke    

    movl    $0,% eax                    # function success, return 0
    popl    %edi                        # teardown
    popl    %esi
    popl    %ebx
    leave
	ret
    
mp1_ioctl_add_copy_fail:
    pushl   %ebx                        # if copy fails, free mem then ret
    call    mp1_free

mp1_ioctl_add_fail:
    movl    $-1, %eax                   # something went wrong, return -1
    popl    %edi 
    popl    %esi
    popl    %ebx
    leave
    ret       

/* Input - location in lower 16 bits */
/* Output - 0 element removed, -1 something wrong */
mp1_ioctl_remove:
    pushl   %ebp
    movl    %esp, %ebp
    pushl   %ebx
    pushl   %esi
    pushl   %edi

    movl    8(%ebp), %ebx               # ebx <- arg
    pushl   %ebx
    call    mp1_ll_search               # find the requested element
    addl    $4, %esp                    
    cmpl    $0, %eax
    je      mp1_ioctl_remove_fail       # if nullptr, element doesn't exist 
    
    /*serach function places desired register in eax and its predecessor in edx */
    cmpl    mp1_list_head, %eax
    je      mp1_ioctl_remove_head
    
    movl    NEXT(%eax), %esi            #prev struct next is now cur struct next
    movl    %esi, NEXT(%edx) 
    pushl   %eax
    call    mp1_free
    movl    $0, %eax
    jmp     mp1_ioctl_remove_done

mp1_ioctl_remove_head:
    movl    NEXT(%eax), %esi
    movl    %esi, mp1_list_head
    pushl   %eax
    call    mp1_free
    movl    $0, %eax
    jmp     mp1_ioctl_remove_done

mp1_ioctl_remove_fail:
    movl    $-1, %eax

mp1_ioctl_remove_done:
    popl    %edi
    popl    %esi
    popl    %ebx
    leave
	ret
        
/* Input - blink struct ptr */
/* Output - 0/-1 for found element/ error */
mp1_ioctl_find:    
    pushl   %ebp                        # build stackframe
    movl    %esp, %ebp
    pushl   %ebx
    pushl   %esi 
    pushl   %edi

    movl    8(%ebp), %esi               # esi <- arg
    cmpl    $0, %esi
    je      mp1_ioctl_find_fail         # make sure ptr isn't null

    movw    LOCATION(%esi), %bx         # ebx <- arg.location
    cmpw    $MAX_LOCATION, %bx
    jge     mp1_ioctl_find_fail         # make sure location is valid

	pushl	%ebx
	call	mp1_ll_search				# look for element with given len
	addl 	$1, %esp
	cmpl 	$0, %eax
	je		mp1_ioctl_find_fail			# if element doesn't exist, fail
	movl 	%eax, %ebx
	
    pushl   $STRUCT_SIZE                # copy kernel-level struct to user-level
    pushl   %ebx
    pushl   %esi
    call    mp1_copy_to_user
    addl    $12, %esp                   # pop args from stack
    cmpl    $0, %eax
    je      mp1_ioctl_find_fail         # if copy_to_user fails, fail
    movl    %esi, 8(%ebp)               # update arg
    movl    $0, %eax                    # update ret val
    jmp     mp1_ioctl_find_end

mp1_ioctl_find_fail:
    movl    $-1, %eax

mp1_ioctl_find_end:
    popl    %edi                        # teardown stackframe
    popl    %esi
    popl    %ebx
    leave
	ret

/* Input - [location1: location2], sync 2nd to 1st*/
/* Output - 0 on success, -1 on failure*/
mp1_ioctl_sync:
    pushl   %ebp
    movl    %esp, %ebp
    pushl   %ebx
    pushl   %esi    
    pushl   %edi

    movl    8(%ebp), %eax               # extract args
    xorl    %ebx, %ebx                  
    movw    %ax, %bx                    # bx <- location2
    shrl    $16, %eax
    movl    %eax, %esi                  # si <- location1

    pushl   %esi
    call    mp1_ll_search               # search linked list for struct1
    addl    $4, %esp                    # pop args from stack
    movl    %eax, %esi                  # esi <- ptr to struct1
    cmpl    $0, %esi                    # if element doesn't exist, fail
    je      mp1_ioctl_sync_fail

    pushl   %ebx
    call    mp1_ll_search               # search linked list for struct2
    addl    $4, %esp                    # pop args from stack
    movl    %eax, %ebx                  # ebx <- ptr to struct2
    cmpl    $0, %ebx                    # if element doesn't exist, fail
    je      mp1_ioctl_sync_fail
    
    movw    ON_LENGTH(%esi), %ax        # copy on_length
    movw    %ax, ON_LENGTH(%ebx)

    movw    OFF_LENGTH(%esi), %ax       # copy off_length
    movw    %ax, OFF_LENGTH(%ebx)

    movw    COUNTDOWN(%esi), %ax        # copy countdown
    movw    %ax, COUNTDOWN(%ebx)

    movw    STATUS(%esi), %ax           # copy status
    movw    %ax, STATUS(%ebx)

    cmpw    $0, %ax
    je      mp1_ioctl_sync_status0
    
    xorl    %eax, %eax
    movw    LOCATION(%ebx), %ax
    shll    $1, %eax
    movb    ON_CHAR(%ebx), %cl
    call    mp1_poke                    # if status = 1, poke on
    movl    $0, %eax
    jmp     mp1_ioctl_sync_done

mp1_ioctl_sync_status0:
    xorl    %eax, %eax
    movw    LOCATION(%ebx), %ax
    shll    $1, %eax
    movb    OFF_CHAR(%ebx), %cl         # if status = 0, poke off
    call    mp1_poke
    movl    $0, %eax
    jmp     mp1_ioctl_sync_done

mp1_ioctl_sync_fail:
    movl    $-1, %eax

mp1_ioctl_sync_done:
    popl    %edi
    popl    %esi
    popl    %ebx
    leave
    ret

/* This is the helper function we're supposed to write */
/* Input - desired location*/
/* Output - pointer to related struct in %eax, NULL if fail*/
/*          pointer to prev struct in %edx as well*/
mp1_ll_search:
    pushl   %ebp
    movl    %esp, %ebp
    pushl   %ebx
    pushl   %esi
    pushl   %edi

    movw    8(%ebp), %bx                # ebx <- location
    cmpw    $MAX_LOCATION, %bx
    jge     mp1_ll_search_fail          # make sure location is valid

    movl    mp1_list_head, %eax         # eax <- list_head    
    movl    mp1_list_head, %edx         # edx <- list_head

mp1_ll_search_loop:
    cmpl    $0, %eax
    je      mp1_ll_search_fail          # if end of list, fail
    
    movw    LOCATION(%eax), %si         # si <- curr.location
    cmpw    %bx, %si                    
    je      mp1_ll_search_done          # if curr.location == arg.location, success
    
    movl    %eax, %edx                  # edx <- curr
    movl    NEXT(%eax), %eax            # eax <- curr.next
    jmp     mp1_ll_search_loop

mp1_ll_search_fail:
    movl    $0, %eax
    
mp1_ll_search_done:
    popl    %edi
    popl    %esi
    popl    %ebx
    leave
    ret

mp1_ioctl_JT:
    .long mp1_ioctl_add, mp1_ioctl_remove, mp1_ioctl_find, mp1_ioctl_sync

.end
