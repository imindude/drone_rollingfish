/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include <lib_lpf.h>
#include <lib_math.h>
#include "mod_imu.h"
#include "drv_mpu.h"

/* ****************************************************************************************************************** */

struct sImuData
{
    sLpf            _gyro_lpf[eAxis_N];
    sLpf            _accl_lpf[eAxis_N];
    eImuAlign       _align;
    bool            _cali_done;
    uint16_t        _bias_count;
    uint16_t        _bias_samples;
    uVectorI16      _gyro_bias_min;
    uVectorI16      _gyro_bias_max;
    uVectorF32      _gyro_bias_mean;
    float           _accl_scale_mean;
};
typedef struct sImuData     sImuData;

/* ****************************************************************************************************************** */

static sImuData     imu_data;

/* ****************************************************************************************************************** */

static bool calibration_update(uVectorI16 *gyro, uVectorI16 *accl)
{
    float   g = 1.0f / (float)(++imu_data._bias_count);
    for (int8_t axis = 0; axis < eAxis_N; axis++)
    {
        if (gyro->_v[axis] < imu_data._gyro_bias_min._v[axis])
            imu_data._gyro_bias_min._v[axis] = gyro->_v[axis];
        else if (gyro->_v[axis] > imu_data._gyro_bias_max._v[axis])
            imu_data._gyro_bias_max._v[axis] = gyro->_v[axis];
        imu_data._gyro_bias_mean._v[axis] = imu_data._gyro_bias_mean._v[axis] * (1.0f - g) + (float)gyro->_v[axis] * g;
    }
    imu_data._accl_scale_mean =
            imu_data._accl_scale_mean * (1.0f - g) + sqrtf(SQUARE(accl->_x) + SQUARE(accl->_y) + SQUARE(accl->_z)) * g;

    if (imu_data._bias_count >= imu_data._bias_samples)
    {
        float       gyro_tolerance = drv_mpu_get_gyro_tolernace();
        uVectorF32  bias_tolerance;
        for (int8_t axis = 0; axis < eAxis_N; axis++)
            bias_tolerance._v[axis] = (float)(imu_data._gyro_bias_max._v[axis] - imu_data._gyro_bias_min._v[axis]);
        if ((bias_tolerance._x < gyro_tolerance) &&
                (bias_tolerance._y < gyro_tolerance) && (bias_tolerance._z < gyro_tolerance))
            return true;
        else
            imu_calibration();
    }
    return false;
}

static void sensor_alignment(uVectorI16 *sensor)
{
    int16_t x = sensor->_x;
    int16_t y = sensor->_y;

    switch (imu_data._align)
    {
    default:
    case eImuAlign_CW0:
        // do nothing
        break;
    case eImuAlign_CW90:
        sensor->_x = -y;
        sensor->_y =  x;
        break;
    case eImuAlign_CW180:
        sensor->_x = -x;
        sensor->_y = -y;
        break;
    case eImuAlign_CW270:
        sensor->_x =  y;
        sensor->_y = -x;
        break;
    }
}

bool imu_init(eImuRate rate, eImuAlign align, uVectorI16 gyro_fc, uVectorI16 accl_fc)
{
    eDrvMpuMode     mpu_mode;
    uint32_t        fs_hz;

    switch (rate)
    {
    default:
    case eImuRate_500:
        mpu_mode = eDrvMpuMode_500;
        fs_hz    = 500;
        break;
    case eImuRate_1000:
        mpu_mode = eDrvMpuMode_1K;
        fs_hz    = 1000;
        break;
    case eImuRate_8000:
        mpu_mode = eDrvMpuMode_8K;
        fs_hz    = 8000;
        break;
    }

    drv_mpu_init(mpu_mode);
    for (int8_t axis = 0; axis < eAxis_N; axis++)
    {
        lpf_init(&imu_data._gyro_lpf[axis], gyro_fc._v[axis], fs_hz);
        lpf_init(&imu_data._accl_lpf[axis], accl_fc._v[axis], fs_hz);
    }
    imu_data._align = align;
    imu_data._bias_samples = (uint16_t)((float)fs_hz * 1.5f);
    imu_calibration();

    return true;
}

void imu_calibration(void)
{
    imu_data._cali_done  = false;
    imu_data._bias_count = 0;
    for (int8_t axis = 0; axis < eAxis_N; axis++)
    {
        imu_data._gyro_bias_min._v[axis]  = 0.0f;
        imu_data._gyro_bias_max._v[axis]  = 0.0f;
        imu_data._gyro_bias_mean._v[axis] = 0.0f;
    }
    imu_data._accl_scale_mean = 0.0f;
}

bool imu_is_new_motion(void)
{
    return drv_mpu_is_new();
}

//union uMessage
//{
//    int16_t     _words[6 * 500 * 3];
//    uint8_t     _bytes[12 * 500 * 3];
//} msg_buf;
//int msg_cnt = -1000;

bool imu_get_motion(uVectorF32 *gyro, uVectorF32 *accl)
{
    uVectorI16      sens_gyro, sens_accl;

    /**
     * MPU gyroscope/accelerometer sensor direction
     *      +y      z_up +
     *   -x    +x
     *      -y      z_dn -
     *
     * but I want to
     *      +x      z_up +
     *   +y    -y
     *      -x      z_dn -
     */
    drv_mpu_get_motion(&sens_gyro._y, &sens_gyro._x, &sens_gyro._z, &sens_accl._y, &sens_accl._x, &sens_accl._z);
    sens_accl._x = -sens_accl._x;
    sens_accl._y = -sens_accl._y;
    sensor_alignment(&sens_gyro);
    sensor_alignment(&sens_accl);

//    if (msg_cnt >= 0)
//    {
//        msg_buf._words[6 * msg_cnt + 0] = sens_gyro._x;
//        msg_buf._words[6 * msg_cnt + 1] = sens_gyro._y;
//        msg_buf._words[6 * msg_cnt + 2] = sens_gyro._z;
//        msg_buf._words[6 * msg_cnt + 3] = sens_accl._x;
//        msg_buf._words[6 * msg_cnt + 4] = sens_accl._y;
//        msg_buf._words[6 * msg_cnt + 5] = sens_accl._z;
//    }
//    msg_cnt++;
//    if (msg_cnt == (500 * 3))
//    {
//        msg_cnt = -1000;
//    }

    if (imu_data._cali_done == false)
    {
        imu_data._cali_done = calibration_update(&sens_gyro, &sens_accl);
        return false;
    }

    for (int8_t axis = 0; axis < eAxis_N; axis++)
    {
        gyro->_v[axis] = (float)sens_gyro._v[axis] - imu_data._gyro_bias_mean._v[axis];
        accl->_v[axis] = (float)sens_accl._v[axis] / imu_data._accl_scale_mean;
        gyro->_v[axis] = (float)sens_gyro._v[axis] * drv_mpu_get_gyro_factor();
        accl->_v[axis] = (float)sens_accl._v[axis] * drv_mpu_get_accl_factor();
        gyro->_v[axis] = lpf_update(&imu_data._gyro_lpf[axis], gyro->_v[axis]);
        accl->_v[axis] = lpf_update(&imu_data._accl_lpf[axis], accl->_v[axis]);
    }
    return true;
}

/* end of file ****************************************************************************************************** */
