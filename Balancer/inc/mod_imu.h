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

enum eImuRate
{
    eImuRate_500,
    eImuRate_1000,
    eImuRate_8000,
    eImuRate_N
};
typedef enum eImuRate           eImuRate;

enum eImuAlign
{
    eImuAlign_CW0,
    eImuAlign_CW90,
    eImuAlign_CW180,
    eImuAlign_CW270,
    eImuAlign_N
};
typedef enum eImuAlign          eImuAlign;

/* ****************************************************************************************************************** */

bool    imu_init(eImuRate rate, eImuAlign align, uVectorI16 gyro_fc, uVectorI16 accl_fc);
void    imu_calibration(void);
bool    imu_is_new_motion(void);
bool    imu_get_motion(uVectorF32 *gyro, uVectorF32 *accl);

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

/* end of file ****************************************************************************************************** */
