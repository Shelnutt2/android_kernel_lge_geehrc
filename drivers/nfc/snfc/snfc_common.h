/*
*	snfc_common.h
*
*/

#ifndef __SNFC_COMMON_H__
#define __SNFC_COMMON_H__

/*
*	Include header files
*/
#include <linux/kernel.h>/* printk() */
#include <linux/fs.h>/*file_operations*/
#include <asm/uaccess.h>/*copy_from_user*/
#include <linux/delay.h>/*mdelay*/
#include <linux/types.h>/* size_t */
#include <linux/miscdevice.h>/*misc_register, misc_deregister*/

/*
 *  Define
 */

/* debug message */
#define FEATURE_DEBUG_LOW
#define SNFC_DEBUG_MSG printk
//#define SNFC_DEBUG_MSG(ARGS,...)

#endif

