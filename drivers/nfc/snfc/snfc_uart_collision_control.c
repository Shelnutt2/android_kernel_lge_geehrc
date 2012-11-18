/*
 *  snfc_uart_collsion_control.c
 *  
 */
 /*
  *    Include header files
  */
#include "snfc_uart_collision_control.h"

/*
*	Define
*/

/*
*	Internal definitions
*/
/*
*	Internal variables
*/
static int isopen_snfcuartcontrol = 0; // 0 : No open 1 : Opend
_e_snfc_uart_status g_uartcollisoncontrol = UART_STATUS_KOTO_OFF;
static int gpio_init = 0;
static int forced_hsel_up_flag=0;
static int forced_pon_up_flag=0;	
/*
 *	Function definitions
 */
 
/*
* Description : open uart collision control
* Input : 
* Output : 
*/
void __snfc_uart_control_set_uart_status(_e_snfc_uart_status uart_status)
{
	_e_snfc_uart_status current_status = g_uartcollisoncontrol;
	
	g_uartcollisoncontrol = uart_status;
	SNFC_DEBUG_MSG("[snfc_uart_control] uart status %d -> %d\n", current_status, g_uartcollisoncontrol );

	return;
}

EXPORT_SYMBOL(__snfc_uart_control_set_uart_status);
/*
* Description : open uart collision control
* Input : 
* Output : 
*/
_e_snfc_uart_status __snfc_uart_control_get_uart_status(void)
{
	return g_uartcollisoncontrol;
}
EXPORT_SYMBOL(__snfc_uart_control_get_uart_status);
/*
* Description : open uart collision control
* Input : 
* Output : 
*/
static int snfc_uart_control_open(struct inode *inode, struct file *fp)
{
	int rc = 0;

	if(isopen_snfcuartcontrol == 1)
	{
		#ifdef FEATURE_DEBUG_LOW 
			SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_open - already open \n");
		#endif
		return 0;
	}
	#ifdef FEATURE_DEBUG_LOW 
    	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_open - start \n");
	#endif

	isopen_snfcuartcontrol = 1;

	if(gpio_init ==0)
	{
		snfc_gpio_open(GPIO_SNFC_HSEL,GPIO_DIRECTION_OUT,GPIO_LOW_VALUE);
		snfc_gpio_open(GPIO_SNFC_PON,GPIO_DIRECTION_OUT,GPIO_LOW_VALUE);
		SNFC_DEBUG_MSG("[snfc_uart_control] GPIO_SNFC_PON = %d, GPIO_SNFC_HSEL = %d\n",
			snfc_gpio_read(GPIO_SNFC_PON),snfc_gpio_read(GPIO_SNFC_HSEL) );	
		gpio_init = 1;
	}

	#ifdef FEATURE_DEBUG_LOW 
		SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_open - end \n");
	#endif

	return rc;
}

/*
* Description : 
* Input : 
* Output : 
*/
static int snfc_uart_control_release (struct inode *inode, struct file *fp)
{
	if(isopen_snfcuartcontrol == 0)
	{
		#ifdef FEATURE_DEBUG_LOW 
		SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_release - not open \n");
		#endif
		return -1;
	}

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_release - start \n");
	#endif

	isopen_snfcuartcontrol = 0;

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_release - end \n");
	#endif

	return 0;
}
/*
* Description : 
* Input : 
* Output : 
*/
static long snfc_uart_control_ioctl(struct file *flip, unsigned int cmd, unsigned long arg)
{
	//ioctl_buf *k_buf;
	//int i,err;
	int size;
	_e_snfc_uart_status current_status;
	size = _IOC_SIZE(cmd);
	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_ioctl - start,cmd =%d\n", cmd);

	current_status = __snfc_uart_control_get_uart_status();
	if( current_status == UART_STATUS_FOR_FELICA )
	{
		SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_ioctl, UART is used to FeliCa\n");
		return -1;
	}

	switch(cmd)
	{
		case IOCTL_SNFC_START_SETTING :
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_SETTING - start\n");
			if(forced_pon_up_flag == 1 || forced_hsel_up_flag == 1)
				break;
			if(current_status != UART_STATUS_READY && current_status != UART_STATUS_FOR_NFC)
			{
				SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_SETTING, UART is not ready\n");
				return -1;
			}
			__snfc_uart_control_set_uart_status(UART_STATUS_FOR_NFC);			
			snfc_gpio_write(GPIO_SNFC_PON, GPIO_HIGH_VALUE);
			mdelay(10); 
			//usleep(10000);
			snfc_gpio_write(GPIO_SNFC_HSEL, GPIO_HIGH_VALUE);
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_SETTING - end\n");
			break;
			
		case IOCTL_SNFC_START_AUTOPOLL :
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_AUTOPOLL - start\n");
			if(forced_pon_up_flag == 1 || forced_hsel_up_flag == 1)
				break;
			if(current_status != UART_STATUS_READY && current_status != UART_STATUS_FOR_NFC)
			{
				SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_AUTOPOLL, UART is not ready\n");
				return -1;
			}	
			if(GPIO_LOW_VALUE == snfc_gpio_read(GPIO_SNFC_RFS))
				return -1;
			
			__snfc_uart_control_set_uart_status(UART_STATUS_FOR_NFC);
			snfc_gpio_write(GPIO_SNFC_PON, GPIO_HIGH_VALUE);
			mdelay(10);
			//usleep(10000);
			snfc_gpio_write(GPIO_SNFC_HSEL, GPIO_HIGH_VALUE);	
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_AUTOPOLL - end\n");
			break;
			
		case IOCTL_SNFC_START_RW :
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_RW - start\n");
			if(forced_pon_up_flag == 1 || forced_hsel_up_flag == 1)
				break;
			if(current_status != UART_STATUS_READY && current_status != UART_STATUS_FOR_NFC)
			{
				SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_RW, UART is not ready\n");
				return -1;
			}	
			__snfc_uart_control_set_uart_status(UART_STATUS_FOR_NFC);
			snfc_gpio_write(GPIO_SNFC_PON, GPIO_HIGH_VALUE);
			mdelay(10);
			//usleep(10000);
			snfc_gpio_write(GPIO_SNFC_HSEL, GPIO_HIGH_VALUE);	
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_RW - end\n");
			break;
			
		case IOCTL_SNFC_START_TARGET :
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_TARGET - start\n");
			if(forced_pon_up_flag == 1 || forced_hsel_up_flag == 1)
				break;
			if(current_status != UART_STATUS_READY && current_status != UART_STATUS_FOR_NFC)
			{
				SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_TARGET, UART is not ready\n");
				return -1;
			}			

			__snfc_uart_control_set_uart_status(UART_STATUS_FOR_NFC);
			snfc_gpio_write(GPIO_SNFC_PON, GPIO_HIGH_VALUE);
			mdelay(10);
			//usleep(10000);
			//snfc_gpio_open(GPIO_SNFC_HSEL,GPIO_DIRECTION_OUT,GPIO_HIGH_VALUE);
			snfc_gpio_write(GPIO_SNFC_HSEL, GPIO_HIGH_VALUE);	
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_TARGET - end\n");
			break;
			
		case IOCTL_SNFC_START_INTU :
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_INTU - start\n");
			if(forced_pon_up_flag == 1 || forced_hsel_up_flag == 1)
				break;
			if(current_status != UART_STATUS_READY && current_status != UART_STATUS_FOR_NFC)
			{
				SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_INTU, UART is not ready or not for nfc\n");
				return -1;
			}		
			__snfc_uart_control_set_uart_status(UART_STATUS_FOR_NFC);
			snfc_gpio_write(GPIO_SNFC_PON, GPIO_HIGH_VALUE);
			mdelay(10);
			//usleep(10000);
			snfc_gpio_write(GPIO_SNFC_HSEL, GPIO_HIGH_VALUE);	
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_INTU - end\n");
			break;
			
		case IOCTL_SNFC_START_WAITSIMBOOT:
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_WAITSIMBOOT - start\n");
			if(forced_pon_up_flag == 1 || forced_hsel_up_flag == 1)
				break;

			if(current_status != UART_STATUS_READY && current_status != UART_STATUS_FOR_NFC)
			{
				SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_INTU, UART is not ready or not for nfc\n");
				return -1;
			}	

			__snfc_uart_control_set_uart_status(UART_STATUS_FOR_NFC);
			snfc_gpio_write(GPIO_SNFC_PON, GPIO_HIGH_VALUE);
			mdelay(10);
			//usleep(10000);
			snfc_gpio_write(GPIO_SNFC_HSEL, GPIO_HIGH_VALUE);	
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_START_WAITSIMBOOT - end\n");		
			break;
			
		case IOCTL_SNFC_HSEL_UP:
			forced_hsel_up_flag = 1;
			__snfc_uart_control_set_uart_status(UART_STATUS_FOR_NFC);	
			SNFC_DEBUG_MSG("[snfc_uart_control] ioctl_snfc_hsel_up\n");	
			snfc_gpio_write(GPIO_SNFC_HSEL, GPIO_HIGH_VALUE);	
			break;
			
		case IOCTL_SNFC_HSEL_DOWN:	
			SNFC_DEBUG_MSG("[snfc_uart_control] ioctl_snfc_hsel_down\n");	
			snfc_gpio_write(GPIO_SNFC_HSEL, GPIO_LOW_VALUE);
			forced_hsel_up_flag = 0;
			__snfc_uart_control_set_uart_status(UART_STATUS_READY);
			break;
			
		case IOCTL_SNFC_PON_UP:
			forced_pon_up_flag = 1;
			SNFC_DEBUG_MSG("[snfc_uart_control] ioctl_snfc_pon_up\n");	
			snfc_gpio_write(GPIO_SNFC_PON, GPIO_HIGH_VALUE);
			mdelay(10);
			break;
			
		case IOCTL_SNFC_PON_DOWN:	
			SNFC_DEBUG_MSG("[snfc_uart_control] ioctl_snfc_pon_down\n");	
			forced_pon_up_flag = 0;
			snfc_gpio_write(GPIO_SNFC_PON, GPIO_LOW_VALUE);	
			break;
			
		case IOCTL_SNFC_END :
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_END - start\n");
			if(forced_pon_up_flag == 1 || forced_hsel_up_flag == 1)
			{
				SNFC_DEBUG_MSG("[snfc_uart_control] pon & hsel forced up!! pon and/or sel will keep high\n");
				break;
			}
			if(current_status != UART_STATUS_FOR_NFC)
			{
				SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_END, UART is not used to NFC\n");
				//return -2;
			}				
			snfc_gpio_write(GPIO_SNFC_HSEL, GPIO_LOW_VALUE);
			snfc_gpio_write(GPIO_SNFC_PON, GPIO_LOW_VALUE);	
			__snfc_uart_control_set_uart_status(UART_STATUS_READY);
			SNFC_DEBUG_MSG("[snfc_uart_control] IOCTL_SNFC_END - end (hsel low)(pon low)\n");
			break;			
			
	}
	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_ioctl - end\n");

	return 0;
}
/*
* Description : 
* Input : 
* Output : 
*/
static int snfc_uart_control_read(struct file *pf, char *pbuf, size_t size, loff_t *pos)
{
	_e_snfc_uart_status current_status;
	int rc;
	
	#ifdef FEATURE_DEBUG_LOW 	
	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_read - start \n");
	#endif
	
	current_status = __snfc_uart_control_get_uart_status();	

	rc = copy_to_user((void*)pbuf, (void*)&current_status, size);
	if(rc)
	{
		SNFC_DEBUG_MSG("[snfc_uart_control] ERROR -  copy_to_user \n");
		return rc;
	}

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_read - end \n");
	#endif	

	return size;
}
/*
* Description : 
* Input : 
* Output : 
*/
static int snfc_uart_control_write(struct file *pf, const char *pbuf, size_t size, loff_t *pos)
{
	_e_snfc_uart_status new_status;
	int rc;
	
	#ifdef FEATURE_DEBUG_LOW 	
	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_write - start \n");
	#endif
	
	rc = copy_from_user(&new_status, (void*)pbuf, size);
	if(rc)
	{
		SNFC_DEBUG_MSG("[snfc_uart_control] ERROR -  copy_to_user \n");
		return rc;
	}

	__snfc_uart_control_set_uart_status(new_status);	

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_write - end \n");
	#endif	

	return size;
}

static struct file_operations snfc_uart_control_fops = 
{
	.owner    = THIS_MODULE,
	.open     = snfc_uart_control_open,
	.read		= snfc_uart_control_read,
	.write    = snfc_uart_control_write,
	.unlocked_ioctl = snfc_uart_control_ioctl,
	.release  = snfc_uart_control_release,
};

static struct miscdevice snfc_uart_control_device = 
{
	.minor = 126,
	.name = "snfc_uart_control",
	.fops = &snfc_uart_control_fops,
};

/*
* Description : 
* Input : 
* Output : 
*/
static int snfc_uart_control_init(void)
{
	int rc=0;
	
	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_init - start \n");
	#endif

	/* register the device file */
	rc = misc_register(&snfc_uart_control_device);
	if (rc)
	{
		SNFC_DEBUG_MSG("[snfc_uart_control] FAIL!! can not register snfc_uart_control \n");
		return rc;
	}
	__snfc_uart_control_set_uart_status(UART_STATUS_READY);

	snfc_gpio_open(GPIO_SNFC_HSEL,GPIO_DIRECTION_OUT,GPIO_LOW_VALUE);
	snfc_gpio_open(GPIO_SNFC_PON,GPIO_DIRECTION_OUT,GPIO_LOW_VALUE);
	SNFC_DEBUG_MSG("[snfc_uart_control] GPIO_SNFC_PON = %d, GPIO_SNFC_HSEL = %d\n",
		snfc_gpio_read(GPIO_SNFC_PON),snfc_gpio_read(GPIO_SNFC_HSEL) );	
	
	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_init - end \n");
	#endif

	return 0;  	
}

/*
* Description : 
* Input : 
* Output : 
*/
static void snfc_uart_control_exit(void)
{
  #ifdef FEATURE_DEBUG_LOW 
  SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_exit - start \n");
  #endif

  /* deregister the device file */
  misc_deregister(&snfc_uart_control_device);
  
  #ifdef FEATURE_DEBUG_LOW 
  SNFC_DEBUG_MSG("[snfc_uart_control] snfc_uart_control_exit - end \n");
  #endif
}

module_init(snfc_uart_control_init);
module_exit(snfc_uart_control_exit);

MODULE_LICENSE("Dual BSD/GPL");

