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

#include <stdbool.h>
#include "typedef.h"

/* ****************************************************************************************************************** */

#define MAX_RCIN_CHANNELS       18      // CPPM(8), SBUS(18)

enum eRcinType
{
    eRcinType_CPPM,
    eRcinType_SBUS,
    eRcinType_N
};
typedef enum eRcinType      eRcinType;

struct sRcinMap
{
    uint8_t     _thr, _rol, _pit, _yaw;
};
typedef struct sRcinMap     sRcinMap;

typedef void    (*cbRcin)(bool failsafe, void *param);

/* ****************************************************************************************************************** */

bool    rcin_init(eRcinType type, sRcinMap map, cbRcin cb, void *param);
float   rcin_get_sticks(uVectorF32 *rpy);
float   rcin_get_key(uint8_t ch);

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

/* end of file ****************************************************************************************************** */
