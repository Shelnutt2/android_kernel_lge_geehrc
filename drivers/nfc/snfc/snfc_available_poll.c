/*
 *  snfc_available_poll.c
 *  
 */

/*
 *  Inclued header files
 */

#include "snfc_available_poll.h"
#include <linux/delay.h>

/*
 *  Define
 */

/*
 *   Function prototype
 */

/*
 *   Internal definition
 */

 /*
  *   Internal variable
  */
static int isopen_snfcavailpoll = 0; // 0 : No open 1 : Open

/*
*	Function definition
*/
/*
 * Description: 
 * Input: 
 * Output: 
 */
static int __snfc_avail_poll_get_rfs_status(void)
{
	return snfc_gpio_read(GPIO_SNFC_RFS);
}
/*
 * Description: 
 * Input: 
 * Output: 
 */
static int __snfc_avail_poll_get_cen_status(void)
{
	int rc = 0;
	unsigned char read_buf = 0x00;
	int cen_status;
	
	rc = snfc_i2c_read(0x02, &read_buf, 1);
	if(rc)
	{
		SNFC_DEBUG_MSG("[__snfc_avail_poll_get_cen_status] snfc_i2c_read : %d \n",rc);
		return -1;
	}
	// check bit 7(locken)
	if(read_buf&0x01) 
	{
		#ifdef FEATURE_DEBUG_LOW
		SNFC_DEBUG_MSG("[__snfc_avail_poll_get_cen_status] CEN = High (UNLOCK) \n");
		#endif
		cen_status = GPIO_HIGH_VALUE;
	}
	else  
	{
		#ifdef FEATURE_DEBUG_LOW
		SNFC_DEBUG_MSG("[__snfc_avail_poll_get_cen_status] CEN = Low (LOCK) \n");
		#endif
		cen_status = GPIO_LOW_VALUE;
	}	

	return cen_status;
}

/*
 * Description: it may need to check rfs, cen gpio is initialized
 * Input: 
 * Output: 
 */
static int snfc_avail_poll_open (struct inode *inode, struct file *fp)
{
	int rc = 0;

	if(isopen_snfcavailpoll == 1)
	{
		#ifdef FEATURE_DEBUG_LOW 
		SNFC_DEBUG_MSG("[snfc_avail_poll] snfc_avail_poll_open - already open \n");
		#endif
		return 0;
	}

	isopen_snfcavailpoll = 1;

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_avail_poll] snfc_avail_poll_open - end \n");
	#endif

	return rc;
}

/*
 * Description: 
 * Input: 
 * Output: 
 */
static int snfc_avail_poll_release (struct inode *inode, struct file *fp)
{
	int rc = 0;

	if(isopen_snfcavailpoll == 0)
	{
		#ifdef FEATURE_DEBUG_LOW 
		SNFC_DEBUG_MSG("[snfc_avail_poll] snfc_avail_poll_release - not opened \n");
		#endif
		return -1;
	}

	isopen_snfcavailpoll = 0;

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_avail_poll] snfc_avail_poll_release - end \n");
	#endif

	return rc;
}

/*
 * Description:
 * Input:
 * Output:
 */
static ssize_t snfc_avail_poll_read(struct file *pf, char *pbuf, size_t size, loff_t *pos)
{
	//unsigned char read_buf = 0x00;
	int loop = 1;
	int available_poll = -1;
	int rc = -1;
	int rfs_status = -1, cen_status = -1, uart_status = -1;
	//unsigned char restart_value=0;
	
	#ifdef FEATURE_DEBUG_LOW
	SNFC_DEBUG_MSG("[snfc_avail_poll] snfc_avail_poll_read - start \n");
	#endif

	/* Check parameters */
	if( NULL == pf || NULL == pbuf /*|| size == NULL*/ /*|| pos == NULL*/)
	{
		SNFC_DEBUG_MSG("[snfc_avail_poll] file error pf = %p, pbuf = %p, size = %d, pos = %d\n", pf, pbuf, (int)size,(int)pos);
		return -1;    
	}

	do{
		rfs_status = __snfc_avail_poll_get_rfs_status();
		cen_status = __snfc_avail_poll_get_cen_status();
		uart_status = __snfc_uart_control_get_uart_status();
		SNFC_DEBUG_MSG("[snfc_avail_poll] current rfs_status : %d, cen_status : %d \n",rfs_status, cen_status);
		if(rfs_status == GPIO_HIGH_VALUE && cen_status == GPIO_HIGH_VALUE && uart_status != UART_STATUS_FOR_FELICA)
			break;
		msleep(10);
	}while(loop);

	available_poll = 1;

	rc = copy_to_user(pbuf, &available_poll, size);
	
	#ifdef FEATURE_DEBUG_LOW
	SNFC_DEBUG_MSG("[snfc_avail_poll] snfc_avail_poll_read - end \n");
	#endif

	return 1;
}

static struct file_operations snfc_avail_poll_fops = {
	.owner    = THIS_MODULE,
	.open     = snfc_avail_poll_open,
	.read     = snfc_avail_poll_read,
	.release  = snfc_avail_poll_release,
};

static struct miscdevice snfc_avail_poll_device = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "snfc_available_poll",
	.fops 	= &snfc_avail_poll_fops,
};

static int snfc_avail_poll_init(void)
{
	int rc;

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_avail_poll] snfc_avail_poll_init - start \n");
	#endif

	/* Register the device file */
	rc = misc_register(&snfc_avail_poll_device);
	if (rc < 0)
	{
		SNFC_DEBUG_MSG("[snfc_avail_poll] FAIL!! can not register snfc_avail_poll \n");
		return rc;
	}

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_avail_poll] snfc_avail_poll_init - end \n");
	#endif

	return 0;
}

static void snfc_avail_poll_exit(void)
{
	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_avail_poll] snfc_avail_poll_exit - start \n");
	#endif

	/* deregister the device file */
	misc_deregister(&snfc_avail_poll_device);

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_avail_poll] snfc_avail_poll_exit - end \n");
	#endif
}

module_init(snfc_avail_poll_init);
module_exit(snfc_avail_poll_exit);

MODULE_LICENSE("Dual BSD/GPL");

