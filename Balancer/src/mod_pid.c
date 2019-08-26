/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "mod_pid.h"
#include "lib_math.h"

/* ****************************************************************************************************************** */

bool pid_init(sPid *pid)
{
    if (pid->_kp > 0.0f)
    {
        pid_reset(pid);
        return true;
    }
    return false;
}

void pid_reset(sPid *pid)
{
    pid->_prev_rate = 0.0f;
    pid->_sum_iterm = 0.0f;
}

float pid_update(sPid *pid, bool rate_ctl, float desired, float actual, float rate, float dt)
{
    float   desired_rate = rate_ctl ? desired : (desired - actual);
    float   error = desired_rate - rate;
    float   pterm = pid->_kp * error;
    float   iterm = pid->_ki * error * dt + pid->_sum_iterm;
    float   dterm = pid->_kd * ((pid->_prev_rate - rate) / dt);

    iterm = TRIM(iterm, -pid->_max_iterm, pid->_max_iterm);
    pid->_prev_rate = rate;
    pid->_sum_iterm = iterm;
    return pterm + iterm + dterm;
}

/* end of file ****************************************************************************************************** */
