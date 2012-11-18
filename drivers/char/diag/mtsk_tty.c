/*
 * DIAG MTS for LGE MTS Kernel Driver
 *
 *  lg-msp TEAM <lg-msp@lge.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
*/
#include "mtsk_tty.h"
#ifdef CONFIG_LGE_MTS
#define MTSK_TTY_IOCTL_MAGIC        'S'
#define MTSK_TTY_OPEN               _IOWR(MTSK_TTY_IOCTL_MAGIC, 0x01, short)
#define MTSK_TTY_CLOSE              _IOWR(MTSK_TTY_IOCTL_MAGIC, 0x02, short)
#define MTSK_TTY_START              _IOWR(MTSK_TTY_IOCTL_MAGIC, 0x03, short)
#define MTSK_TTY_STOP               _IOWR(MTSK_TTY_IOCTL_MAGIC, 0x04, short)
#define MTSK_TTY_SETMASK            _IOWR(MTSK_TTY_IOCTL_MAGIC, 0x05, short)
#define MTSK_TTY_CHK_REG            _IOWR(MTSK_TTY_IOCTL_MAGIC, 0x06, unsigned int)

typedef enum {
    MTSK_TTY_SET_MASK_NONE        = 0,
    MTSK_TTY_SET_MASK_BT_ON       = 1,
    MTSK_TTY_SET_MASK_WIFI_ON     = 2,
    MTSK_TTY_SET_MASK_RF_CDMA_ON  = 3,
    MTSK_TTY_SET_MASK_RF_WCDMA_ON = 4,
    MTSK_TTY_SET_MASK_RF_LTE_ON   = 5,
    MTSK_TTY_SET_MASK_MAX         = 6
} _type_mtsk_tty_mask_table;

struct mts_tty *mtsk_tty = NULL;

int init_mtsk_tty = FALSE;

void mtsk_tty_push (char *buf , int  left) {
    int num_push = 0;
    int total_push = 0;
    static int count = 0;
    struct mts_tty *mtsk_tty_drv = mtsk_tty;

    if(mtsk_tty_drv == NULL)  {
        return;
    }

    do {
        num_push = tty_insert_flip_string(mtsk_tty_drv->tty_struct, buf + total_push, left);
        total_push += num_push;
        left -= num_push;
        tty_flip_buffer_push(mtsk_tty_drv->tty_struct);
        count ++;
    } while (left != 0);

    return;
}

int mtsk_tty_modem_request(const unsigned char *buf, int count) {
    diag_process_hdlc((void *)buf + 6, count - 6); 
    return count;
}

static int mtsk_tty_open(struct tty_struct *tty, struct file *file) {
    struct mts_tty *mtsk_tty_drv = NULL;

    if (!tty)
        return -ENODEV;

    mtsk_tty_drv = mtsk_tty;

    if (!mtsk_tty_drv)
        return -ENODEV;

    tty->driver_data = mtsk_tty_drv;
    mtsk_tty_drv->tty_struct = tty;

    if (mtsk_tty_drv->tty_state == MTS_TTY_OPEN)
        return -EBUSY;

    set_bit(TTY_NO_WRITE_SPLIT, &mtsk_tty_drv->tty_struct->flags);
 
    mtsk_tty_drv->tty_state = MTS_TTY_OPEN;

    printk(KERN_INFO "mtsk_tty_open TTY device open %d,%d\n", 0, 0);

    return 0;
}

static void mtsk_tty_close(struct tty_struct *tty, struct file *file) {
    struct mts_tty *mtsk_tty_drv = NULL;

    if (!tty) {
        printk(KERN_INFO "mtsk_tty_close FAIL. tty is Null %d,%d\n", 0, 0);
        return;
    }

    mtsk_tty_drv = tty->driver_data;

    if (mtsk_tty_drv->tty_state != MTS_TTY_OPEN) {
        printk(KERN_INFO "mtsk_tty_close FAIL. why MTS_TTY_OPEN is not opend  %d,%d \n", 0, 0);
        return;
    } 

    mtsk_tty->tty_state = MTS_TTY_CLOSED;
    //mtsk_tty_drv->tty_struct = NULL;
    printk(KERN_INFO "mtsk_tty_close SUCCESS. %d,%d \n", 0, 0);

    return;
}

static int mtsk_tty_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg) {
    struct mts_tty *mtsk_tty_drv = NULL;
	int ret = 0;
	unsigned int * cable_info;
	unsigned int smem_size = 0;

    mtsk_tty_drv = mtsk_tty;
    tty->driver_data = mtsk_tty_drv;
    mtsk_tty_drv->tty_struct = tty;

    if (_IOC_TYPE(cmd) != MTSK_TTY_IOCTL_MAGIC) {
        printk(KERN_INFO "mtsk_tty_ioctl %d - %d\n", cmd, 0);        
        return -EINVAL;
    }

	switch (cmd) {
		case MTSK_TTY_START:
			break;
			
		case MTSK_TTY_STOP:
			break;

		case MTSK_TTY_CHK_REG:
			//cable_info = *(unsigned int *)(smem_get_entry(SMEM_ID_VENDOR1, &smem_size));
			cable_info = smem_get_entry(SMEM_ID_VENDOR1, &smem_size);
			if (smem_size == 0 || !cable_info) {
				printk("%s : smem_get_entry EFAULT\n", __func__);
				return -EFAULT;
			}

			if (copy_to_user((void *)arg, (const void *)cable_info, sizeof(unsigned int)) == 0) {
				printk(KERN_INFO "mtsk: complete sending pif cable info %d\n", *cable_info);
			} else {
				printk(KERN_INFO "mtsk: fail to sending pif cable info\n");
			}
			break;

		default: 
			ret = -EINVAL;
			break;
	}
 
    return ret;
}

static void mtsk_tty_unthrottle(struct tty_struct *tty) {
    return;
}

static int mtsk_tty_write_room(struct tty_struct *tty) {
    return DIAG_MTS_TX_SIZE;
}

static int mtsk_tty_write(struct tty_struct *tty, const unsigned char *buf, int count) {
    int result;
    struct mts_tty *mtsk_tty_drv = NULL;

    mtsk_tty_drv = mtsk_tty;
    tty->driver_data = mtsk_tty_drv;
    mtsk_tty_drv->tty_struct = tty;

    /* check the packet size */
    if (count > DIAG_MTS_RX_MAX_PACKET_SIZE) {
        printk(KERN_INFO "mtsk_tty_write packet size  %d,%d\n", count, DIAG_MTS_RX_MAX_PACKET_SIZE);
        return -EPERM;
    }
    
    result = mtsk_tty_modem_request(buf, count);
    return result;
}

static const struct tty_operations mtsk_tty_ops = {
    .open = mtsk_tty_open,
    .close = mtsk_tty_close,
    .write = mtsk_tty_write,
    .write_room = mtsk_tty_write_room,
    .unthrottle = mtsk_tty_unthrottle,
    .ioctl = mtsk_tty_ioctl,
};

static int __init mtsk_tty_init(void) {
    int ret = 0;
	struct device *tty_dev =  NULL; 
	struct mts_tty *mtsk_tty_drv = NULL;
    
    pr_info(MTS_TTY_MODULE_NAME ": %s\n", __func__);
    mtsk_tty_drv = kzalloc(sizeof(struct mts_tty), GFP_KERNEL);
    
    if (mtsk_tty_drv == NULL) {
        printk(KERN_INFO "lge_diag_mts_init FAIL %d - %d\n", 0, 0);
        return 0;
    }

    mtsk_tty = mtsk_tty_drv;
    mtsk_tty_drv->tty_drv = alloc_tty_driver(MAX_DIAG_MTS_DRV);

    if (!mtsk_tty_drv->tty_drv) {
        printk(KERN_INFO "lge_diag_mts_init init FAIL %d - %d\n", 1, 0);
        kfree(mtsk_tty_drv);
        return 0;
    }

    mtsk_tty_drv->tty_drv->name = "mtsk_tty"; 
    mtsk_tty_drv->tty_drv->owner = THIS_MODULE;
    mtsk_tty_drv->tty_drv->driver_name = "mtsk_tty";
    
    /* uses dynamically assigned dev_t values */
    mtsk_tty_drv->tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
    mtsk_tty_drv->tty_drv->subtype = SERIAL_TYPE_NORMAL;
    mtsk_tty_drv->tty_drv->flags = TTY_DRIVER_REAL_RAW     | TTY_DRIVER_DYNAMIC_DEV | TTY_DRIVER_RESET_TERMIOS;

    /* initializing the mts driver */
    mtsk_tty_drv->tty_drv->init_termios = tty_std_termios;
    mtsk_tty_drv->tty_drv->init_termios.c_iflag = IGNBRK | IGNPAR;
    mtsk_tty_drv->tty_drv->init_termios.c_oflag = 0;
    mtsk_tty_drv->tty_drv->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
    mtsk_tty_drv->tty_drv->init_termios.c_lflag = 0;
    tty_set_operations(mtsk_tty_drv->tty_drv, &mtsk_tty_ops);
    ret = tty_register_driver(mtsk_tty_drv->tty_drv);

    if (ret) {
        put_tty_driver(mtsk_tty_drv->tty_drv);
        mtsk_tty_drv->tty_drv = NULL;
        kfree(mtsk_tty_drv);
        return 0;
    }

    tty_dev = tty_register_device(mtsk_tty_drv->tty_drv, 0, NULL);

    if (IS_ERR(tty_dev)) {
        tty_unregister_driver(mtsk_tty_drv->tty_drv);
        put_tty_driver(mtsk_tty_drv->tty_drv);
        kfree(mtsk_tty_drv);
        return 0;
    }
     mtsk_tty_drv->tty_state = MTS_TTY_REGISTERED;


    printk(KERN_INFO "mtsk_tty_init  SUCESS MTS_TTY_REGISTERED \n");
    init_mtsk_tty = TRUE;

    return 0;
}

static void __exit mtsk_tty_exit(void) {
    int ret = 0;
    struct mts_tty *mtsk_tty_drv = NULL;

    init_mtsk_tty = FALSE;

    mtsk_tty_drv = mtsk_tty;

    if (!mtsk_tty_drv) {
        pr_err(MTS_TTY_MODULE_NAME ": %s:"
        "NULL mtsk_tty_drv", __func__);
        return;
    }

    mdelay(20); 

    tty_unregister_device(mtsk_tty_drv->tty_drv, 0);
    ret = tty_unregister_driver(mtsk_tty_drv->tty_drv);
    put_tty_driver(mtsk_tty_drv->tty_drv);
    mtsk_tty_drv->tty_state = MTS_TTY_NONE;
    mtsk_tty_drv->tty_drv = NULL;

    kfree(mtsk_tty_drv); 
    mtsk_tty = NULL;
    printk(KERN_INFO "mtsk_tty_exit  SUCESS %d - %d\n", 0, 0);
    return;
}
 
module_init(mtsk_tty_init);
module_exit(mtsk_tty_exit);

MODULE_DESCRIPTION("LGE MTS TTY");
MODULE_LICENSE("GPL");
MODULE_AUTHOR(" lg-msp TEAM <lg-msp@lge.com>");
#endif
