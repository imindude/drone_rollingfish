/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "drv_l298.h"
#include "hal_intf.h"

/* ****************************************************************************************************************** */

struct sL298Data
{
    hHalPwmOut  _pwm;
    hHalGpio    _gpio_dir1;
    hHalGpio    _gpio_dir2;
};
typedef struct sL298Data    sL298Data;

/* ****************************************************************************************************************** */

static sL298Data    l298_data[eDrvL298_N];

/* ****************************************************************************************************************** */

bool drv_l298_init(eDrvL298 l298)
{
    /**
     * input[0:1] -> output[0:40] (unit:microsecond)
     */
    switch (l298)
    {
    case eDrvL298_Left:
        l298_data[l298]._pwm = hal_pwmout_init(eHalPwmOut_MotorLeft, 25000);
        l298_data[l298]._gpio_dir1 = hal_gpio_init(eHalGpio_L298_In1);
        l298_data[l298]._gpio_dir2 = hal_gpio_init(eHalGpio_L298_In2);
        break;
    case eDrvL298_Right:
        l298_data[l298]._pwm = hal_pwmout_init(eHalPwmOut_MotorRight, 25000);
        l298_data[l298]._gpio_dir1 = hal_gpio_init(eHalGpio_L298_In3);
        l298_data[l298]._gpio_dir2 = hal_gpio_init(eHalGpio_L298_In4);
        break;
    default:
        return false;
    }

    hal_gpio_reset(l298_data[l298]._gpio_dir1);
    hal_gpio_reset(l298_data[l298]._gpio_dir2);
    hal_pwmout_set_duty(l298_data[l298]._pwm, 0.0f);

    return true;
}

void drv_l298_output(eDrvL298 l298, float power)
{
    bool    forward = true;

    if (power < 0.0f)
    {
        forward = false;
        power   = -power;
    }

    if (forward)
    {
        hal_gpio_set(l298_data[l298]._gpio_dir1);
        hal_gpio_reset(l298_data[l298]._gpio_dir2);
    }
    else
    {
        hal_gpio_reset(l298_data[l298]._gpio_dir1);
        hal_gpio_set(l298_data[l298]._gpio_dir2);
    }

    // https://www.wolframalpha.com/input/?i=linear+fit+%7B0,+0%7D,+%7B1,+40%7D
    hal_pwmout_set_duty(l298_data[l298]._pwm, power * 40.0f);
}

void drv_l298_braking(eDrvL298 l298)
{
    hal_gpio_reset(l298_data[l298]._gpio_dir1);
    hal_gpio_reset(l298_data[l298]._gpio_dir2);
}

/* end of file ****************************************************************************************************** */
