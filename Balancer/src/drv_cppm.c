/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "drv_cppm.h"
#include "hal_intf.h"

/* ****************************************************************************************************************** */

#define MAX_CHANNELS        8
#define SYNC_DELAY_US       2200
#define FAILSAFE_US         4400

struct sCppmData
{
    hHalPwmIn   _pwm;
    bool        _sync;
    uint8_t     _n_rx;
    int16_t     *_rxch_buf;
    cbDrvCppm   _callback;
    void        *_param;
};
typedef struct sCppmData    sCppmData;

/* ****************************************************************************************************************** */

static sCppmData    cppm_data =
{
        ._pwm      = NULL,
        ._sync     = false,
        ._n_rx     = 0,
        ._rxch_buf = NULL,
        ._callback = NULL,
        ._param    = NULL
};

/* ****************************************************************************************************************** */

static void pwmin_callback(uint32_t dt_us, void *param)
{
    sCppmData   *this = (sCppmData*)param;
    if (dt_us > FAILSAFE_US)
    {
        this->_n_rx = 0;
        if (this->_callback)
            this->_callback(this->_n_rx, true, this->_param);
    }
    else if (dt_us >= SYNC_DELAY_US)
    {
        this->_n_rx = 0;
        this->_sync = true;
    }
    else if (this->_sync)
    {
        this->_rxch_buf[this->_n_rx] = (int16_t)dt_us;
        this->_n_rx++;
        if (this->_n_rx >= MAX_CHANNELS)
        {
            if (this->_callback)
                this->_callback(this->_n_rx, false, this->_param);
            this->_n_rx = 0;
            this->_sync = false;
        }
    }
}

bool drv_cppm_init(int16_t *rxch_buf, cbDrvCppm cb, void *param)
{
    cppm_data._pwm = hal_pwmin_init(eHalPwmIn_CPPM, pwmin_callback, (void*)&cppm_data);
    if (cppm_data._pwm != NULL)
    {
        cppm_data._rxch_buf = rxch_buf;
        cppm_data._callback = cb;
        cppm_data._param    = param;
        return true;
    }
    return false;
}

/* end of file ****************************************************************************************************** */
