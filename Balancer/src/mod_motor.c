/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "mod_motor.h"
#include "drv_l298.h"

/* ****************************************************************************************************************** */

enum eMotorWheel
{
    eMotorWheel_Left,
    eMotorWheel_Right,
    eMotorWheel_N
};
typedef enum eMotorWheel    eMotorWheel;

struct sMotorData
{
    uActuator   _acting_rate[eMotorWheel_N];
};
typedef struct sMotorData   sMotorData;

/* ****************************************************************************************************************** */

static sMotorData   motor_data;

/* ****************************************************************************************************************** */

bool motor_init(void)
{
    motor_data._acting_rate[eMotorWheel_Left]._rol =  0.0f;
    motor_data._acting_rate[eMotorWheel_Left]._pit = -1.0f;
    motor_data._acting_rate[eMotorWheel_Left]._yaw = -1.0f;
    motor_data._acting_rate[eMotorWheel_Right]._rol =  0.0f;
    motor_data._acting_rate[eMotorWheel_Right]._pit = -1.0f;
    motor_data._acting_rate[eMotorWheel_Right]._yaw =  1.0f;

    drv_l298_init(eDrvL298_Left);
    drv_l298_init(eDrvL298_Right);

    return true;
}

void motor_output(uActuator *actuator)
{
    float   thrust[eMotorWheel_N];
    float   max_thrust = 0.0f;
    float   min_thrust = -1.0f;
    float   overshoot = 0.0f;
    float   undershoot = 0.0f;
    int8_t  wheel;

    for (wheel = 0; wheel < eMotorWheel_N; wheel++)
    {
        for (int8_t axis = 0; axis < eAxis_N; axis++)
            thrust[wheel] = actuator->_v[axis] * motor_data._acting_rate[wheel]._v[axis];
        if (max_thrust < thrust[wheel])
            max_thrust = thrust[wheel];
        else if (min_thrust > thrust[wheel])
            min_thrust = thrust[wheel];
    }
    if (max_thrust > 1.0f)
        overshoot = max_thrust - 1.0f;
    if (min_thrust < -1.0f)
        undershoot = min_thrust + 1.0f;
    for (wheel = 0; wheel < eMotorWheel_N; wheel++)
    {
        thrust[wheel] -= overshoot;
        thrust[wheel] -= undershoot;
        drv_l298_output(wheel, thrust[wheel]);
    }
}

void motor_test(int wheel, float power)
{
    drv_l298_output(wheel, power);
}

/* end of file ****************************************************************************************************** */
