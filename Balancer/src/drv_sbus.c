/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "drv_sbus.h"
#include "hal_intf.h"

/* ****************************************************************************************************************** */

#define PACKET_LEN         25
#define MAX_CHANNELS       18
#define SYNC_DELAY_MS      3
#define FAILSAFE_MS        100
#define SYNC_BYTE          0x0F
#define MIN_VALUE          172
#define MAX_VALUE          1812

__attribute__ ((__packed__))
union uPacket
{
    struct
    {
        /* SYNC(1byte) + DATA(22byte) + ... */
        uint8_t     _sync;
        uint32_t    _ch1    :11;
        uint32_t    _ch2    :11;
        uint32_t    _ch3    :11;
        uint32_t    _ch4    :11;
        uint32_t    _ch5    :11;
        uint32_t    _ch6    :11;
        uint32_t    _ch7    :11;
        uint32_t    _ch8    :11;
        uint32_t    _ch9    :11;
        uint32_t    _ch10   :11;
        uint32_t    _ch11   :11;
        uint32_t    _ch12   :11;
        uint32_t    _ch13   :11;
        uint32_t    _ch14   :11;
        uint32_t    _ch15   :11;
        uint32_t    _ch16   :11;

        uint8_t     _flag;
        uint8_t     _end;
    } _frame;
    uint8_t     _bytes[PACKET_LEN];
};
typedef union uPacket       uPacket;

struct sSbusData
{
    hHalUart    _uart;
    uPacket     _packet;
    bool        _sync;
    uint32_t    _last_ms;
    int16_t     *_rxch_buf;
    cbDrvSbus   _callback;
    void        *_param;
};
typedef struct sSbusData    sSbusData;

/* ****************************************************************************************************************** */

static sSbusData    sbus_data =
{
        ._uart     = NULL,
        ._sync     = false,
        ._last_ms  = 0,
        ._rxch_buf = NULL,
        ._callback = NULL,
        ._param    = NULL
};

/* ****************************************************************************************************************** */

static void sbus_notify(sSbusData *this, bool failsafe)
{
#define FLAG_CH17       0b0001
#define FLAG_CH18       0b0010
#define FLAG_SIG_LOSS   0b0100
#define FLAG_FAILSAFE   0b1000

    if (this->_callback) {

        this->_rxch_buf[ 0] = this->_packet._frame._ch1;
        this->_rxch_buf[ 1] = this->_packet._frame._ch2;
        this->_rxch_buf[ 2] = this->_packet._frame._ch3;
        this->_rxch_buf[ 3] = this->_packet._frame._ch4;
        this->_rxch_buf[ 4] = this->_packet._frame._ch5;
        this->_rxch_buf[ 5] = this->_packet._frame._ch6;
        this->_rxch_buf[ 6] = this->_packet._frame._ch7;
        this->_rxch_buf[ 7] = this->_packet._frame._ch8;
        this->_rxch_buf[ 8] = this->_packet._frame._ch9;
        this->_rxch_buf[ 9] = this->_packet._frame._ch10;
        this->_rxch_buf[10] = this->_packet._frame._ch11;
        this->_rxch_buf[11] = this->_packet._frame._ch12;
        this->_rxch_buf[12] = this->_packet._frame._ch13;
        this->_rxch_buf[13] = this->_packet._frame._ch14;
        this->_rxch_buf[14] = this->_packet._frame._ch15;
        this->_rxch_buf[15] = this->_packet._frame._ch16;
        this->_rxch_buf[16] = this->_packet._frame._flag & FLAG_CH17 ? MAX_VALUE : MIN_VALUE;
        this->_rxch_buf[17] = this->_packet._frame._flag & FLAG_CH18 ? MAX_VALUE : MIN_VALUE;

        if (this->_packet._frame._flag & (FLAG_FAILSAFE | FLAG_SIG_LOSS))
            failsafe = true;
        this->_callback(MAX_CHANNELS, failsafe, this->_param);
    }

#undef FLAG_CH17
#undef FLAG_CH18
#undef FLAG_SIG_LOSS
#undef FLAG_FAILSAFE
}

static void uart_callback(int16_t size, bool error, void *param)
{
    sSbusData   *this = (sSbusData*)param;

    if (error || (size == 0))
    {
        this->_sync = false;
        this->_packet._bytes[0] = 0;
        hal_uart_rx(this->_uart, this->_packet._bytes, 1);
    }
    else
    {
        uint32_t    now_ms = hal_get_millis();
        uint32_t    dt_ms = now_ms - this->_last_ms;

        if (dt_ms < FAILSAFE_MS)
        {
            if (this->_sync)
            {
                if (this->_packet._bytes[0] == SYNC_BYTE)
                {
                    sbus_notify(this, false);
                    this->_packet._bytes[0] = 0;
                    hal_uart_rx(this->_uart, this->_packet._bytes, PACKET_LEN);
                }
                else
                {
                    this->_sync = false;
                    this->_packet._bytes[0] = 0;
                    hal_uart_rx(this->_uart, this->_packet._bytes, 1);
                }
            }
            else
            {
                if ((dt_ms > SYNC_DELAY_MS) && (this->_packet._bytes[0] == SYNC_BYTE))
                {
                    this->_sync = true;
                    hal_uart_rx(this->_uart, this->_packet._bytes + 1, PACKET_LEN - 1);
                }
                else
                {
                    this->_packet._bytes[0] = 0;
                    hal_uart_rx(this->_uart, this->_packet._bytes, 1);
                }
            }
        }
        else
        {
            sbus_notify(this, true);
            this->_sync = false;
            this->_packet._bytes[0] = 0;
            hal_uart_rx(this->_uart, this->_packet._bytes, 1);
        }
        this->_last_ms = now_ms;
    }
}

bool drv_sbus_init(int16_t *rxch_buf, cbDrvSbus cb, void *param)
{
    sbus_data._uart = hal_uart_init(eHalUart_SerialRx, 100000,
            HAL_UART_PARITY_EVEN | HAL_UART_STOP_2B | HAL_UART_MODE_RX, uart_callback, &sbus_data);
    if (sbus_data._uart != NULL)
    {
        sbus_data._sync     = false;
        sbus_data._last_ms  = hal_get_millis();
        sbus_data._rxch_buf = rxch_buf;
        sbus_data._callback = cb;
        sbus_data._param    = param;
        hal_uart_rx(sbus_data._uart, sbus_data._packet._bytes, 1);
        return true;
    }
    return false;
}

/* end of file ****************************************************************************************************** */
