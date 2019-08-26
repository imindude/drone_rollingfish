/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "mod_rcin.h"
#include "drv_cppm.h"
#include "drv_sbus.h"
#include "lib_math.h"

/* ****************************************************************************************************************** */

struct sRcinData
{
    sRcinMap    _map;
    int16_t     _rc_buf[MAX_RCIN_CHANNELS];
    float       _rc_key[MAX_RCIN_CHANNELS];
    uint8_t     _n_channels;
    cbRcin      _callback;
    void        *_param;
};
typedef struct sRcinData        sRcinData;

/* ****************************************************************************************************************** */

static sRcinData    rcin_data;

/* ****************************************************************************************************************** */

static void cppm_callback(int8_t n_ch, bool failsafe, void *param)
{
    sRcinData   *this = (sRcinData*)param;
    this->_n_channels = n_ch;
    // https://www.wolframalpha.com/input/?i=linear+fit+%7B1000,+-1%7D,+%7B2000,+1%7D,+%7B1500,+0%7D
    for (uint8_t ch = 0; ch < n_ch; ch++)
        this->_rc_key[ch] = 0.002f * this->_rc_buf[ch] - 3.0f;
    if (this->_callback)
        this->_callback(failsafe, this->_param);
}

static void sbus_callback(int8_t n_ch, bool failsafe, void *param)
{
    sRcinData   *this = (sRcinData*)param;
    this->_n_channels = n_ch;
    // [FrSky(X4RSB)] https://www.wolframalpha.com/input/?i=linear+fit+%7B172,+-1%7D,+%7B1811,+1%7D,+%7B992,+0%7D
    for (uint8_t ch = 0; ch < n_ch; ch++)
    {
        this->_rc_key[ch] = 0.00122026f * this->_rc_buf[ch] - 1.21009f;
        this->_rc_key[ch] = TRIM(this->_rc_key[ch], -1.0f, 1.0f);
    }
    if (this->_callback)
        this->_callback(failsafe, this->_param);
}

bool rcin_init(eRcinType type, sRcinMap map, cbRcin cb, void *param)
{
    switch (type)
    {
    default:
    case eRcinType_CPPM:
        drv_cppm_init(rcin_data._rc_buf, cppm_callback, &rcin_data);
        break;
    case eRcinType_SBUS:
        drv_sbus_init(rcin_data._rc_buf, sbus_callback, &rcin_data);
        break;
    }
    rcin_data._map      = map;
    rcin_data._callback = cb;
    rcin_data._param    = param;
    return true;
}

float rcin_get_sticks(uVectorF32 *rpy)
{
    rpy->_rol = rcin_data._rc_key[rcin_data._map._rol];
    rpy->_pit = rcin_data._rc_key[rcin_data._map._pit];
    rpy->_yaw = rcin_data._rc_key[rcin_data._map._yaw];
    return rcin_data._rc_key[rcin_data._map._thr];
}

float rcin_get_key(uint8_t ch)
{
    return rcin_data._rc_key[ch];
}

/* end of file ****************************************************************************************************** */
