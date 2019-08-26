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

enum eDrvMpuMode
{
    eDrvMpuMode_500,
    eDrvMpuMode_1K,
    eDrvMpuMode_8K,
    eDrvMpuMode_N
};
typedef enum eDrvMpuMode    eDrvMpuMode;

/* ****************************************************************************************************************** */

bool    drv_mpu_init(eDrvMpuMode mode);
float   drv_mpu_get_gyro_factor(void);
float   drv_mpu_get_accl_factor(void);
float   drv_mpu_get_gyro_tolernace(void);
bool    drv_mpu_is_new(void);
void    drv_mpu_get_motion(int16_t *gx, int16_t *gy, int16_t *gz, int16_t *ax, int16_t *ay, int16_t *az);

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

/* end of file ****************************************************************************************************** */
