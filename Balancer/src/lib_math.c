/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "lib_math.h"

/* ****************************************************************************************************************** */

__attribute__((always_inline))
inline float invert_sqrtf(float x)
{
    float   halfx = 0.5f * x;
    float   y = x;
    long    n = *(int32_t*)&y;
    n = 0x5f3759df - (n >> 1);
    y = *(float*)&n;
    return y * (1.5f - (halfx * y * y));
}

sEulerAngle quaternion_to_euler(sQuaternion q)
{
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    sEulerAngle euler;
    float       sinrol_cospit = 2.0f * (q._q0 * q._q1 + q._q2 * q._q3);
    float       cosrol_cospit = 1.0f - 2.0f * (q._q1 * q._q1 + q._q2 * q._q2);
    float       sinpit = 2.0f * (q._q0 * q._q2 - q._q1 * q._q3);
    float       sinyaw_cospit = 2.0f * (q._q0 * q._q3 + q._q1 * q._q2);
    float       cosyaw_cospit = 1.0f - 2.0f * (q._q2 * q._q2 + q._q3 * q._q3);
    // phi : rotation about X-axis
    euler._rol = atan2f(sinrol_cospit, cosrol_cospit);
    // theta : rotation about Y-axis
    if (fabsf(sinpit) >= 1.0f)
        euler._pit = copysignf(M_PI / 2.0f, sinpit);
    else
        euler._pit = asinf(sinpit);
    // psi : rotation about Z-axis
    euler._yaw = atan2f(sinyaw_cospit, cosyaw_cospit);

    return euler;
}

inline float radian_to_degree(float radian)
{
    return radian * (180.0f / M_PI);
}

inline float degree_to_radian(float degree)
{
    return degree * (M_PI / 180.0f);
}

/* *********************************************************************************************************************
 * The math here is done with
 *   https://www.wolframalpha.com/input/?i=Solve%5By%3D((g%2F100)*x%5E3%2B((100-g)%2F100)*x),x%5D
 * e.g. y = 0.85 expo = 50%
 *   https://www.wolframalpha.com/input/?i=Solve%5B0.85%3D((50%2F100)*x%5E3%2B((100-50)%2F100)*x),x%5D
 *
 * @param   x   input from [-1,1]
 * @param   exp sets the exponential amount [0,100]
 * ********************************************************************************************************************/
inline float linear_to_curve(float x, int8_t exp)
{
    return ((float)exp / 100.0f) * powf(x, 3) + ((float)(100 - exp) / 100.0f) * x;
}

/* end of file ****************************************************************************************************** */
