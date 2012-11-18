/*
 *  felicagpio.h
 *  
 */

#ifndef __FELICA_GPIO_H__
#define __FELICA_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  INCLUDE FILES FOR MODULE
 */

#include <linux/list.h>
#include <linux/gpio.h>

#include "felica_common.h"
/*
 *  DEFINE
 */

/* common */ 
enum{
  GPIO_DIRECTION_IN = 0,
  GPIO_DIRECTION_OUT,
};

enum{
  GPIO_LOW_VALUE = 0,
  GPIO_HIGH_VALUE,
};

enum{
  GPIO_CONFIG_ENABLE = 0,
  GPIO_CONFIG_DISABLE,
};

#if defined(CONFIG_LGE_FELICA_KDDI)
/* felica_pon */
#define GPIO_FELICA_PON   37

/* felica_rfs */
#define GPIO_FELICA_RFS   55

/* felica_int */
#define GPIO_FELICA_INT   22

/* felica_lockcont */
#define GPIO_FELICA_LOCKCONT   29

#define GPIO_NFC_HSEL   57

#elif defined(CONFIG_LGE_FELICA_DCM)
/* felica_pon */
#define GPIO_FELICA_PON 37
/*107 */

/* felica_rfs */
#define GPIO_FELICA_RFS 55
/* 128 */

/* felica_int */
#define GPIO_FELICA_INT 22
/* 125 */

/* felica_lockcont */
#define GPIO_FELICA_LOCKCONT 29
/* 123  */
#else

#endif
/*
 *  FUNCTION PROTOTYPE
 */
int felica_gpio_open(int gpionum, int direction, int value);
void felica_gpio_write(int gpionum, int value);
int felica_gpio_read(int gpionum);

#ifdef __cplusplus
}
#endif

#endif // __FELICA_RFS_H__
