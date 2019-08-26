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

enum eAxis
{
    eAxis_X,
    eAxis_Y,
    eAxis_Z,

    eAxis_Rol = eAxis_X,
    eAxis_Pit = eAxis_Y,
    eAxis_Yaw = eAxis_Z,

    eAxis_N
};
typedef enum eAxis          eAxis;

union uVectorI16
{
    struct
    {
        int16_t     _x, _y,_z;
    };
    struct
    {
        int16_t     _rol, _pit, _yaw;
    };
    int16_t     _v[3];
};
typedef union uVectorI16    uVectorI16;

union uVectorF32
{
    struct
    {
        float   _x, _y, _z;
    };
    struct
    {
        float   _rol, _pit, _yaw;
    };
    float   _v[3];
};
typedef union uVectorF32    uVectorF32;

struct sEulerAngle
{
    float   _rol, _pit, _yaw;
};
typedef struct sEulerAngle  sEulerAngle;

union uActuator
{
    struct
    {
        float   _rol, _pit, _yaw;
    };
    float   _v[3];
};
typedef union uActuator     uActuator;

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

/* end of file ****************************************************************************************************** */
