/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "driving.h"
#include "tasker.h"
#include "logger.h"
#include "lib_math.h"
#include "mod_status.h"
#include "mod_rcin.h"
#include "mod_imu.h"
#include "mod_ahrs.h"
#include "mod_pid.h"
#include "mod_motor.h"

/* ****************************************************************************************************************** */

struct sDrivingData
{
    bool        _armed;
    bool        _rcin_ready;
    bool        _imu_update;
    int8_t      _rc_expo;
    uint8_t     _max_angle;
    uVectorF32  _desired_rpy;
    sPid        _pid_pit;
    sPid        _pid_yaw;

    uint32_t    _prev_ahrs_us;
};
typedef struct sDrivingData     sDrivingData;

/* ****************************************************************************************************************** */

static sDrivingData driving_data =
{
        ._armed      = false,
        ._rcin_ready = false,
        ._imu_update = false,
        ._rc_expo    = 75,
        ._max_angle  = 5,

        ._pid_pit._kp = 0.1f,
        ._pid_pit._ki = 0.01f,
        ._pid_pit._kd = 0.0f,
        ._pid_pit._max_iterm = 0.4f,
        ._pid_yaw._kp = 0.1f,
        ._pid_yaw._ki = 0.01f,
        ._pid_yaw._kd = 0.0f,
        ._pid_yaw._max_iterm = 0.4f,

        ._prev_ahrs_us = 0
};

/* ****************************************************************************************************************** */

static void rcin_callback(bool failsafe, void *param)
{
    ((sDrivingData*)param)->_rcin_ready = true;
}

static bool wakeup(uint32_t now_us, void *param)
{
    sDrivingData    *this = (sDrivingData*)param;

    this->_imu_update = imu_is_new_motion();
    if (this->_imu_update || this->_rcin_ready)
        return true;
    return false;
}

extern void motor_test(int wheel, float power);
static void worker(uint32_t now_us, void *param)
{
    sDrivingData    *this = (sDrivingData*)param;

    if (this->_rcin_ready)
    {
        this->_rcin_ready = false;

        uVectorF32  rpy;
        float       aux_key = rcin_get_key(6);

        rcin_get_sticks(&rpy);

        for (int8_t axis = 0; axis < eAxis_N; axis++)
        {
            rpy._v[axis] = linear_to_curve(rpy._v[axis], this->_rc_expo);
            this->_desired_rpy._v[axis] = SCALE(rpy._v[axis], -1.0f, 1.0f, -this->_max_angle, this->_max_angle);
        }

        this->_armed = IN_RANGE(aux_key, 0.5f, 1.0f) ? true : false;

        if (rpy._pit < 0.0f)
            motor_test(0, -0.54f + rpy._pit / 2.0f);
        else
            motor_test(0, 0.54f + rpy._pit / 2.0f);

        if (rpy._yaw < 0.0f)
            motor_test(1, -0.54f + rpy._yaw / 2.0f);
        else
            motor_test(1, 0.54f + rpy._yaw / 2.0f);
    }

    if (this->_imu_update)
    {
        this->_imu_update = false;

        float       dt = (now_us - this->_prev_ahrs_us) / 1000000.0f;
        uVectorF32  gyro, accl;
        sEulerAngle actual_rpy;
        uActuator   actuator = { { 0.0f, 0.0f, 0.0f } };

        this->_prev_ahrs_us = now_us;

        if (imu_get_motion(&gyro, &accl))
        {
            actual_rpy = ahrs_update(&gyro, &accl, dt);

//            {
//                static int count = 0;
//                if (count++ >= 250)
//                {
//                    TRACE("%d / %d / %d \r\n", (int)(actual_rpy._rol*100), (int)(actual_rpy._pit*100), (int)(actual_rpy._yaw * 100));
//                    count = 0;
//                }
//            }

            if (this->_armed)
            {
                actuator._pit =
                        pid_update(&this->_pid_pit, false, this->_desired_rpy._pit, actual_rpy._pit, gyro._pit, dt);
                actuator._yaw =
                        pid_update(&this->_pid_yaw, true, this->_desired_rpy._yaw, actual_rpy._yaw, gyro._yaw, dt);

                actuator._pit += (actuator._pit < 0) ? -0.54f : 0.54f;
                actuator._yaw += (actuator._yaw < 0) ? -0.54f : 0.54f;
            }
            else
            {
                pid_reset(&this->_pid_pit);
                pid_reset(&this->_pid_yaw);
            }
        }
        else
        {
            ahrs_reset();
        }

        motor_output(&actuator);
    }
}

bool driving_init(void)
{
    sRcinMap    rcin_map = { 3, 2, 1, 0 };
    uVectorI16  gyro_fc = {{ 50, 50, 50 }};
    uVectorI16  accl_fc = {{ 50, 50, 50 }};
    uAhrsGain   ahrs_gain =
    {
            ._two_kp = 0.72f,
            ._two_ki = 0.001f,
//            ._beta = 0.2f
    };

    status_init();
    rcin_init(eRcinType_SBUS, rcin_map, rcin_callback, &driving_data);
    imu_init(eImuRate_500, eImuAlign_CW270, gyro_fc, accl_fc);
    ahrs_init(eAhrsType_Mahony, &ahrs_gain);
    pid_init(&driving_data._pid_pit);
    pid_init(&driving_data._pid_yaw);
    motor_init();

    tasker_join(wakeup, worker, eTaskPrio_High, &driving_data);

    return true;
}

/* end of file ****************************************************************************************************** */
