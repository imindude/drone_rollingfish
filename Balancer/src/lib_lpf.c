/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "lib_lpf.h"
#include "lib_math.h"

/* ****************************************************************************************************************** */

bool lpf_init(sLpf *lpf, uint32_t fc_hz, uint32_t fs_hz)
{
    if (fc_hz < fs_hz)
    {
        float   dt = 1.0f / fs_hz;
        float   rc = 1.0f / (2.0f * M_PI * (float)fc_hz);
        lpf->_a  = dt / (rc + dt);
        lpf_reset(lpf);
        return true;
    }
    return false;
}

void lpf_reset(sLpf *lpf)
{
    lpf->_y = 0;
}

float lpf_update(sLpf *lpf, float x)
{
    lpf->_y += lpf->_a * (x - lpf->_y);
    return lpf->_y;
}

/* end of file ****************************************************************************************************** */
