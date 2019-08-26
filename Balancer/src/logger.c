/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "logger.h"
#include "hal_intf.h"

/* ****************************************************************************************************************** */

void logger_init(void)
{
    hal_vcp_init();
}

void logger_setup(sLogger *logger, uint16_t duty_ms)
{
    logger->_duty_ms = duty_ms;
    logger->_last_ms = hal_get_millis();
}

void logger_message(sLogger *logger, uint8_t *message, uint32_t length)
{
#if 0
    if (logger->_duty_ms != 0)
    {
        uint32_t    now_ms = hal_get_millis();
        if ((now_ms - logger->_last_ms) > logger->_duty_ms)
        {
            TRACE("%s", message);
            logger->_last_ms = now_ms;
        }
    }
#else
    hal_vcp_tx_timeout(message, length, 2);
#endif
}

/* ****************************************************************************************************************** */

// system call of "printf"
int _write(int file, char *ptr, int len)
{
    hal_vcp_tx_timeout((uint8_t*)ptr, len, 100);
    return len;
}

/* end of file ****************************************************************************************************** */
