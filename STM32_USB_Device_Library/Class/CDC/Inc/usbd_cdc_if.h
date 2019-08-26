/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ****************************************************************************************************************** */

#include "usbd_cdc.h"

/* ****************************************************************************************************************** */

extern USBD_CDC_ItfTypeDef  USBD_Interface_fops_FS;
extern USBD_HandleTypeDef   husbd;

/* ****************************************************************************************************************** */

extern uint8_t  CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
extern int16_t  CDC_ReadRx(uint8_t *buf, int16_t len);

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

#endif  // USBD_CDC_IF_H

/* end of file ****************************************************************************************************** */