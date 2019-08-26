/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* ****************************************************************************************************************** */

#include <stdint.h>
#include <stdbool.h>

/* ****************************************************************************************************************** */

typedef void    (*cbDrvSbus)(int8_t n_ch, bool failsafe, void *param);

/* ****************************************************************************************************************** */

bool    drv_sbus_init(int16_t *rxch_buf, cbDrvSbus cb, void *param);

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

/* end of file ****************************************************************************************************** */