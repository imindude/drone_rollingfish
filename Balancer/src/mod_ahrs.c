/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "mod_ahrs.h"
#include "lib_math.h"

/* ****************************************************************************************************************** */

struct sAhrsData
{
    uAhrsGain   _gain;
    sQuaternion _q;
    void        (*_update)(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    // mahony only
    float       _integral_feedback_x;
    float       _integral_feedback_y;
    float       _integral_feedback_z;
};
typedef struct sAhrsData    sAhrsData;

/* ****************************************************************************************************************** */

static sAhrsData    ahrs_data;

/* ****************************************************************************************************************** */

static void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float   recip_norm;
    float   halfvx, halfvy, halfvz;
    float   halfex, halfey, halfez;
    float   qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recip_norm = invert_sqrtf(SQUARE(ax) + SQUARE(ay) + SQUARE(az));
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = ahrs_data._q._q1 * ahrs_data._q._q3 - ahrs_data._q._q0 * ahrs_data._q._q2;
        halfvy = ahrs_data._q._q0 * ahrs_data._q._q1 + ahrs_data._q._q2 * ahrs_data._q._q3;
        halfvz = SQUARE(ahrs_data._q._q0) - 0.5f + SQUARE(ahrs_data._q._q3);

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (ahrs_data._gain._two_ki > 0.0f)
        {
            ahrs_data._integral_feedback_x += ahrs_data._gain._two_ki * halfex * dt;    // integral error scaled by Ki
            ahrs_data._integral_feedback_y += ahrs_data._gain._two_ki * halfey * dt;
            ahrs_data._integral_feedback_z += ahrs_data._gain._two_ki * halfez * dt;
            gx += ahrs_data._integral_feedback_x;   // apply integral feedback
            gy += ahrs_data._integral_feedback_y;
            gz += ahrs_data._integral_feedback_z;
        }
        else
        {
            ahrs_data._integral_feedback_x = 0.0f;  // prevent integral windup
            ahrs_data._integral_feedback_y = 0.0f;
            ahrs_data._integral_feedback_z = 0.0f;
        }

        // Apply proportional feedback
        gx += ahrs_data._gain._two_kp * halfex;
        gy += ahrs_data._gain._two_kp * halfey;
        gz += ahrs_data._gain._two_kp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);      // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = ahrs_data._q._q0;
    qb = ahrs_data._q._q1;
    qc = ahrs_data._q._q2;
    ahrs_data._q._q0 += (-qb * gx - qc * gy - ahrs_data._q._q3 * gz);
    ahrs_data._q._q1 += (qa * gx + qc * gz - ahrs_data._q._q3 * gy);
    ahrs_data._q._q2 += (qa * gy - qb * gz + ahrs_data._q._q3 * gx);
    ahrs_data._q._q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recip_norm = invert_sqrtf(SQUARE(ahrs_data._q._q0) + SQUARE(ahrs_data._q._q1) + SQUARE(ahrs_data._q._q2) +
            SQUARE(ahrs_data._q._q3));
    ahrs_data._q._q0 *= recip_norm;
    ahrs_data._q._q1 *= recip_norm;
    ahrs_data._q._q2 *= recip_norm;
    ahrs_data._q._q3 *= recip_norm;
}

static void madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float   recip_norm;
    float   s0, s1, s2, s3;
    float   qdot1, qdot2, qdot3, qdot4;
    float   _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qdot1 = 0.5f * (-ahrs_data._q._q1 * gx - ahrs_data._q._q2 * gy - ahrs_data._q._q3 * gz);
    qdot2 = 0.5f * (ahrs_data._q._q0 * gx + ahrs_data._q._q2 * gz - ahrs_data._q._q3 * gy);
    qdot3 = 0.5f * (ahrs_data._q._q0 * gy - ahrs_data._q._q1 * gz + ahrs_data._q._q3 * gx);
    qdot4 = 0.5f * (ahrs_data._q._q0 * gz + ahrs_data._q._q1 * gy - ahrs_data._q._q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recip_norm = invert_sqrtf(SQUARE(ax) + SQUARE(ay) + SQUARE(az));
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * ahrs_data._q._q0;
        _2q1 = 2.0f * ahrs_data._q._q1;
        _2q2 = 2.0f * ahrs_data._q._q2;
        _2q3 = 2.0f * ahrs_data._q._q3;
        _4q0 = 4.0f * ahrs_data._q._q0;
        _4q1 = 4.0f * ahrs_data._q._q1;
        _4q2 = 4.0f * ahrs_data._q._q2;
        _8q1 = 8.0f * ahrs_data._q._q1;
        _8q2 = 8.0f * ahrs_data._q._q2;
        q0q0 = ahrs_data._q._q0 * ahrs_data._q._q0;
        q1q1 = ahrs_data._q._q1 * ahrs_data._q._q1;
        q2q2 = ahrs_data._q._q2 * ahrs_data._q._q2;
        q3q3 = ahrs_data._q._q3 * ahrs_data._q._q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * ahrs_data._q._q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 +
                _4q1 * az;
        s2 = 4.0f * q0q0 * ahrs_data._q._q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 +
                _4q2 * az;
        s3 = 4.0f * q1q1 * ahrs_data._q._q3 - _2q1 * ax + 4.0f * q2q2 * ahrs_data._q._q3 - _2q2 * ay;
        recip_norm = invert_sqrtf(SQUARE(s0) + SQUARE(s1) + SQUARE(s2) + SQUARE(s3));   // normalise step magnitude
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        // Apply feedback step
        qdot1 -= ahrs_data._gain._beta * s0;
        qdot2 -= ahrs_data._gain._beta * s1;
        qdot3 -= ahrs_data._gain._beta * s2;
        qdot4 -= ahrs_data._gain._beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    ahrs_data._q._q0 += qdot1 * dt;
    ahrs_data._q._q1 += qdot2 * dt;
    ahrs_data._q._q2 += qdot3 * dt;
    ahrs_data._q._q3 += qdot4 * dt;

    // Normalise quaternion
    recip_norm = invert_sqrtf(SQUARE(ahrs_data._q._q0) + SQUARE(ahrs_data._q._q1) + SQUARE(ahrs_data._q._q2) +
            SQUARE(ahrs_data._q._q3));
    ahrs_data._q._q0 *= recip_norm;
    ahrs_data._q._q1 *= recip_norm;
    ahrs_data._q._q2 *= recip_norm;
    ahrs_data._q._q3 *= recip_norm;
}

bool ahrs_init(eAhrsType type, uAhrsGain *gain)
{
    ahrs_data._gain = *gain;
    ahrs_data._update = (type == eAhrsType_Mahony) ? mahony_update : madgwick_update;
    ahrs_reset();
    return true;
}

void ahrs_reset(void)
{
    QUATERNION_INIT(ahrs_data._q);
    ahrs_data._integral_feedback_x = 0.0f;
    ahrs_data._integral_feedback_y = 0.0f;
    ahrs_data._integral_feedback_z = 0.0f;
}

sEulerAngle ahrs_update(uVectorF32 *gyro, uVectorF32 *accl, float dt)
{
    ahrs_data._update(degree_to_radian(gyro->_x), degree_to_radian(gyro->_y), degree_to_radian(gyro->_z),
            accl->_x, accl->_y, accl->_z, dt);

    sEulerAngle euler_angle = quaternion_to_euler(ahrs_data._q);
    euler_angle._rol = radian_to_degree(euler_angle._rol);
    euler_angle._pit = radian_to_degree(euler_angle._pit);
    euler_angle._yaw = radian_to_degree(euler_angle._yaw);
    return euler_angle;
}

/* end of file ****************************************************************************************************** */
