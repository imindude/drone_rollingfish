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

#define DEFAULT_MAHONY_TWOKP        0.25f
#define DEFAULT_MAHONY_TWOKI        0.0f
#define DEFAULT_MADGWICK_BETA       0.2f

enum eAhrsType
{
    eAhrsType_Mahony,
    eAhrsType_Madgwick,
    eAhrsType_N
};
typedef enum eAhrsType      eAhrsType;

union uAhrsGain
{
    struct
    {
        float   _two_kp;
        float   _two_ki;
    };
    float   _beta;
};
typedef union uAhrsGain     uAhrsGain;

/* ****************************************************************************************************************** */

bool        ahrs_init(eAhrsType type, uAhrsGain *gain);
void        ahrs_reset(void);
sEulerAngle ahrs_update(uVectorF32 *gyro, uVectorF32 *accl, float dt);

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

/* end of file ****************************************************************************************************** */
