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

#include <math.h>
#include "typedef.h"

/* ****************************************************************************************************************** */

struct sQuaternion
{
    float   _q0, _q1, _q2, _q3;
};
typedef struct sQuaternion  sQuaternion;

#define SCALE(in, in_min, in_max, out_min, out_max)\
    (((((out_max) - (out_min)) * ((in) - (in_min))) / ((in_max) - (in_min))) + (out_min))
#define TRIM(in, min, max)              (((in) < (min)) ? (min) : ((in) > (max)) ? (max) : (in))
#define IN_RANGE(in, min, max)          (((in) >= (min)) && ((in) <= (max)))
#define IN_TOLERANCE(in, val, tol)      (((in) >= ((val) - (tol))) && ((in) <= ((val) + (tol))))

#define SQUARE(x)           ((x) * (x))
#define QUATERNION_INIT(q)\
{\
    (q)._q0 = 1.0f;\
    (q)._q1 = 0.0f;\
    (q)._q2 = 0.0f;\
    (q)._q3 = 0.0f;\
}

/* ****************************************************************************************************************** */

float       invert_sqrtf(float x);
sEulerAngle quaternion_to_euler(sQuaternion q);
float       radian_to_degree(float radian);
float       degree_to_radian(float degree);
float       linear_to_curve(float x, int8_t exp);

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

/* end of file ****************************************************************************************************** */
