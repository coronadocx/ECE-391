/*
 * tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) //printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)

/************************ Protocol Implementation *************************/
static int need_ack;                   //Signals when the driver should wait for ack
static unsigned button_vector;              //Bit-vector for determining button presses [7:0] = (right, left, down, up, c, b, a, start)
static unsigned long led_state;             //Stores the state of the LEDs in case of reset

static spinlock_t ioctl_lock = SPIN_LOCK_UNLOCKED;  //Prevent race conditions on button vector

static unsigned char num_to_led[16] = {0xE7, 0x06, 0xCB, 0x8f, 0x2e, 0xAD, 0xED, 0x86, 0xEF, 0xAF, 0xEE, 0x6d, 0xE1, 0x4f, 0xE9, 0xE8}; //Map numbers to led states

//Local function declerations
static int _tux_init(struct tty_struct* tty);
static int _tux_buttons(unsigned long arg);
static int _tux_set_led(struct tty_struct* tty, unsigned long arg);

//Local function definitions

/*
 * _tux_init
 *  DESCRIPTION: Initializes the tux controller and sets all relevant vars
 *  INPUTS: none
 *  OUTPUTS: none
 *  RETURN VALUE: returns 0 always
 *  SIDE EFFECTS: Puts the tux LEDs into user mode.
 *                Also pust the controller into button interrupt mode
 */
int _tux_init(struct tty_struct* tty)
{
    char buf[1];            //Input buffer 
    unsigned long flags;

    //Initialize vars
    spin_lock_irqsave(&ioctl_lock, flags);
    led_state = 0;
    button_vector = 0;
    need_ack = 2;    
    spin_unlock_irqrestore(&ioctl_lock, flags);

    //Initialize Tux behavior
    buf[0] = MTCP_LED_USR;
    tuxctl_ldisc_put(tty, buf, 1);     //Put tux LEDs into user mode (no ACK required)

    buf[0] = MTCP_BIOC_ON;
    tuxctl_ldisc_put(tty, buf, 1);     //Tell the tux controller to use interrupts for buttons
    return 0;
}

/*
 * _tux_buttons
 *  INPUTS: arg - Pointer to an integer
 *  OUTPUTS: arg - Holds a bit-vector of the current-button presses
 *  RETURN VALUE: -1 on error, 0 on success
 *  SIDE EFFECTS: none
 */
int _tux_buttons(unsigned long arg)
{
    unsigned long flags;

    //Make sure pointer is valid
    if (arg == 0)
        return -1;

    //Lock to check ack and write button vector
    spin_lock_irqsave(&ioctl_lock, flags);
    if (need_ack != 0){
        spin_unlock_irqrestore(&ioctl_lock, flags);
        return -1;
    }

    __copy_to_user((unsigned long *)arg, &button_vector, 2);
    spin_unlock_irqrestore(&ioctl_lock, flags);
    return 0;
}

/*
 * _tux_set_led
 *  INPUT: tty - Pointer to tty_struct to manipulate device 
 *         arg - number containing LED information (description in mtcp.h)
 *  OUTPUT: none
 *  RETURN VALUE: 0 on success, -1 on need-ack
 *  SIDE EFFECTS: changes the state of the tux leds
 *                saves the current led state for resets
 */
int _tux_set_led(struct tty_struct* tty, unsigned long arg)
{
    char buf[NUM_LEDS+2];                               //At most, buffer contains the command, the on-vector, and 4 led bytes
    unsigned long flags;
    unsigned disp_num    =  (arg & 0x0000FFFF);         //Low 16 specify a number whose hex value is to be displayed on the LEDs
    unsigned char led_on = ((arg & 0x000F0000)>>16);    //Low 4 bits of the 3rd byte (bits [19:16]) specify which LEDs to turn on
    unsigned char dec_on = ((arg & 0x0F000000)>>24);    //Low 4 bits of the 4th byte (buts [27:24]) specify which decimal poiunts to be turned on

    unsigned i;
    unsigned char num_led_on = ((led_on&0x01)) + ((led_on&0x02)>>1) + ((led_on&0x04)>>2) + ((led_on&0x08)>>3);
    if (num_led_on == 0)
        return 0;

    // printk("Total LEDs : %x\n", num_led_on);
    
    //First byte is the command opcode
    buf[0] = MTCP_LED_SET;

    //Second byte determines which leds to set and how many bytes will follow (one per LED being set)
    buf[1] = led_on;

    /*
     * Create a bye for each led being used.
     * The proper segment display is precalculated and then combined
     * with the decimal point bit to create the output byte.
     * LEDs are sent in ascending order (CMD, LED_ON, 0,1,2,3)
     */
    for (i = 0; i < num_led_on; i++){
        // printk("%u\n", ((disp_num&(0x000F<<(i*4)))>>(i*4)));
        buf[(i+2)] = (num_to_led[((disp_num&(0x000F<<(i*4)))>>(i*4))] | (((dec_on >> i) & 0x01) << 4));
    }

    spin_lock_irqsave(&ioctl_lock, flags);
    if (need_ack != 0){
        // printk("SetLED need ACK : %d\n", need_ack);
        spin_unlock_irqrestore(&ioctl_lock, flags);
        return -1;
    }

    led_state = arg;    //Save led state
    need_ack++;       //Require ack
    spin_unlock_irqrestore(&ioctl_lock, flags);

    //Write buffer to tux controller
    tuxctl_ldisc_put(tty, buf, (num_led_on+2));
    // printk("Exited normally");
    return 0;
}

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in
 * tuxctl-ld.c. It calls this function, so all warnings there apply
 * here as well.
 *
 * IMPORTANT: This function is called from an interrupt context, so it
 *            cannot acquire any semaphores or otherwise sleep, or access
 *            the 'current' pointer. It also must not take up too much time.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet) {
    unsigned a, b, c;
    unsigned long flags;    
    char buf[1]; 

    a = packet[0]; /* Avoid //printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

    //printk("packet : %x %x %x\n", a, b, c); 


    //Discard ps2 packets (for now?)
    if (a&0x08)
        return;


    // Decide what to do based off of the op-code 
    switch (a) {
        case MTCP_ACK:          //Signal driver that device is ready for next command
            spin_lock_irqsave(&ioctl_lock, flags);
            need_ack--;
            if (need_ack < 0){
                //printk("Negative ACK\n");
                need_ack = 0;   //Prevent negative ack    
            }

            //printk("ACK : %d\n", need_ack);
            spin_unlock_irqrestore(&ioctl_lock, flags);
            return;

        case MTCP_BIOC_EVENT:   //Put button data into buffer (should act as an interrupt) 
            //printk("BIOC_EVENT\n");
            spin_lock_irqsave(&ioctl_lock, flags);
            //                      R            L               D             U               CBAS
            button_vector = ( (((c&0x08) | ((c&0x02)<<1) | ((c&0x04)>>1) | (c&0x01)) << 4) | (b & 0x0F));    //Byte 1 [3:0] is CBAS respectively, Byte 2 [3:0] is RDLU respectively, need RLDUCBAS 

            //printk("%u\n",button_vector);
            spin_unlock_irqrestore(&ioctl_lock, flags);
            return;

        case MTCP_RESET:    //FIXME
            //printk("RESET\n");
            need_ack = 0;

            //Reset LEDs
            _tux_set_led(tty, led_state);                   //Restore LEDs
            need_ack += 1;                                  //Increment ack for setting user and bioc
			
			//NOTE - For some reason QEMU does not like this command. 
			//Everything seems to work fine without it, so I guess it stays commented.
            //buf[0] = MTCP_LED_USR;
            //tuxctl_ldisc_put(tty, buf, 1);                  //Put tux LEDs into user mode (no ACK required)

            spin_lock_irqsave(&ioctl_lock, flags);
            button_vector = 0;                              //Clear reset button vector
            spin_unlock_irqrestore(&ioctl_lock, flags);

            //Reset buttons
            buf[0] = MTCP_BIOC_ON;
            tuxctl_ldisc_put(tty, buf, 1);                  //Tell the tux controller to use interrupts for buttons

            return;

        default:
            return;
    }

}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this   rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/

int tuxctl_ioctl(   struct tty_struct* tty, 
                    struct file* file,  //Currently unused
                    unsigned cmd, 
                    unsigned long arg)
{
    //printk("IOCTL\n");

    //If tty is broken, do not continue
    if (tty == 0){                   
        //printk("tty bad\n");
        return -1;
    }

    switch (cmd) {
        case TUX_INIT:
            //printk("INIT\n");
            return _tux_init(tty);

        case TUX_BUTTONS:
            //printk("BUTTONS\n");
            return _tux_buttons(arg);

        case TUX_SET_LED:
            //printk("LED\n");
            return _tux_set_led(tty, arg);
        default:
            return -EINVAL;
    }
}

