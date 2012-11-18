/*
 *  snfc_rfs.c
 *  
 */

 /*
  *	Include header files
  */

#include "snfc_rfs.h"
#include "snfc_gpio.h"

/*
*	Defines
*/

/*
 *   Function prototype
 */

/*
 *   Internal definition
 */

/*
*	Internal variables
*/
static int isopen_snfcrfs = 0; // 0 : No open 1 : Open

/*
*	Function definition
*/

/*
 * Description:
 * Input: 
 * Output: 
 */
static int snfc_rfs_open (struct inode *inode, struct file *fp)
{
	int rc = 0;

	if(isopen_snfcrfs == 1)
	{
		#ifdef FEATURE_DEBUG_LOW 
		SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_open - already open \n");
		#endif
		return 0;
	}

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_open - start \n");
	#endif

	//rc = snfc_gpio_open(GPIO_SNFC_RFS, GPIO_DIRECTION_IN, GPIO_LOW_VALUE);

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_open - end \n");
	#endif
	isopen_snfcrfs = 1;

	return rc;
}


/*
 * Description:
 * Input: 
 * Output: RFS low : 1 RFS high : 0
 */
static ssize_t snfc_rfs_read(struct file *pf, char *pbuf, size_t size, loff_t *pos)
{
	int rc = 0;
	int getvalue = GPIO_LOW_VALUE;
	int rfonoff;

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_read - start \n");
	#endif

	/* Check Parameters */
	if(pf == NULL || pbuf == NULL /*|| size == NULL*/ /*||pos == NULL*/)
	{
		SNFC_DEBUG_MSG("[snfc_rfs] parameters ERROR pf = %p, pbuf = %p, size = %d, pos = %p\n",pf,pbuf,(int)size,pos);
		return -1;    
	}

	/* Get GPIO value */
	getvalue = snfc_gpio_read(GPIO_SNFC_RFS);

	if((getvalue != GPIO_LOW_VALUE)&&(getvalue != GPIO_HIGH_VALUE))
	{
		SNFC_DEBUG_MSG("[snfc_rfs] ERROR - getvalue is out of range \n");
		return -2;    
	}

	/* Copy value to user memory */
	//getvalue = getvalue ? GPIO_LOW_VALUE: GPIO_HIGH_VALUE;
	SNFC_DEBUG_MSG("[snfc_rfs] RFS pin status : %d \n", getvalue);

	if(getvalue)
		rfonoff = 0;
	else
		rfonoff = 1;
	
	SNFC_DEBUG_MSG("[snfc_rfs] rf status : %d \n", rfonoff);

	rc = copy_to_user((void*)pbuf, (void*)&rfonoff, size);
	if(rc)
	{
		SNFC_DEBUG_MSG("[snfc_rfs] ERROR -  copy_to_user \n");
		return rc;
	}

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_read - end \n");
	#endif

	return size;
}

/*
 * Description: 
 * Input: 
 * Output: 
 */
static int snfc_rfs_release (struct inode *inode, struct file *fp)
{
	if(isopen_snfcrfs == 0)
	{
		#ifdef FEATURE_DEBUG_LOW 
		SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_release - not open \n");
		#endif
		return -1;
	}

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_release - start \n");
	#endif
	isopen_snfcrfs = 0;

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_release - end \n");
	#endif

	return 0;
}

static struct file_operations snfc_rfs_fops = 
{
	.owner    = THIS_MODULE,
	.open      = snfc_rfs_open,
	.read      = snfc_rfs_read,
	.release  = snfc_rfs_release,
};

static struct miscdevice snfc_rfs_device = {
	.minor = 125,
	.name = "snfc_rfs",
	.fops = &snfc_rfs_fops,
};

static int snfc_rfs_init(void)
{
	int rc;

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_init - start \n");
	#endif

	/* register the device file */
	rc = misc_register(&snfc_rfs_device);
	if (rc < 0)
	{
		SNFC_DEBUG_MSG("[snfc_rfs] FAIL!! can not register snfc_rfs \n");
		return rc;
	}

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_init - end \n");
	#endif

	return 0;
}

static void snfc_rfs_exit(void)
{
	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_exit - start \n");
	#endif

	/* deregister the device file */
	misc_deregister(&snfc_rfs_device);

	#ifdef FEATURE_DEBUG_LOW 
	SNFC_DEBUG_MSG("[snfc_rfs] snfc_rfs_exit - end \n");
	#endif
}

module_init(snfc_rfs_init);
module_exit(snfc_rfs_exit);

MODULE_LICENSE("Dual BSD/GPL");

