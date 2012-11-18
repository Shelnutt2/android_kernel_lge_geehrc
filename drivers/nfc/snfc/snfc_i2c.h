/*
* snfc_i2c.h
*
*/

#ifndef __SNFC_I2C_H__
#define __SNFC_I2C_H__

/*
 *  INCLUDE FILES FOR MODULE
 */
#include <linux/list.h>

#include "snfc_common.h" 
/*
 *	DEFINE
 */

/*
 *	FUNCTION PROTOTYPE
 */
int snfc_i2c_open (void);
int snfc_i2c_release (void);
int snfc_i2c_read(unsigned char reg, unsigned char *buf, size_t count);
int snfc_i2c_write(unsigned char reg, unsigned char *buf, size_t count);
int snfc_i2c_set_slave_address (unsigned char slave_address);


#endif
