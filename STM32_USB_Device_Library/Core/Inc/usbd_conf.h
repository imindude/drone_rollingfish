/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#ifndef USBD_CONF_H
#define USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ****************************************************************************************************************** */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"

/* ****************************************************************************************************************** */

#define USBD_MAX_NUM_INTERFACES         1
#define USBD_MAX_NUM_CONFIGURATION      1
#define USBD_MAX_STR_DESC_SIZ           0x100
#define USBD_SUPPORT_USER_STRING        0
#define USBD_SELF_POWERED               1
#define USBD_DEBUG_LEVEL                0
#define USBD_CDC_INTERVAL               2000
#define DEVICE_FS                       0

/* Memory management macros */
#define USBD_malloc               malloc
#define USBD_free                 free
#define USBD_memset               memset
#define USBD_memcpy               memcpy

#define USBD_ErrLog(...)

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

#endif  // USBD_CONF_H

/* end of file ****************************************************************************************************** */
