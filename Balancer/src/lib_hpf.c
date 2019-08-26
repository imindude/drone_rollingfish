/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "lib_hpf.h"
#include "lib_math.h"

/* ****************************************************************************************************************** */

bool hpf_init(sHpf *hpf, uint32_t fc_hz, uint32_t fs_hz)
{
    if (fc_hz < fs_hz)
    {
        float   dt = 1.0f / fs_hz;
        float   rc = 1.0f / (2.0f * M_PI * (float)fc_hz);
        hpf->_a  = rc / (rc + dt);
        hpf_reset(hpf);
        return true;
    }
    return false;
}

void hpf_reset(sHpf *hpf)
{
    hpf->_x = 0;
    hpf->_y = 0;
}

float hpf_update(sHpf *hpf, float x)
{
    hpf->_y = hpf->_a * (hpf->_y + x - hpf->_x);
    hpf->_x = x;
    return hpf->_y;
}

/* end of file ****************************************************************************************************** */
