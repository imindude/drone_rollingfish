/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "usbd_cdc_if.h"

/* ****************************************************************************************************************** */

#define TX_DATA_SIZE  4
#define RX_DATA_SIZE  256

static uint8_t  usr_tx_buf[TX_DATA_SIZE];
static uint8_t  usr_rx_buf[RX_DATA_SIZE];
static int16_t  usr_rx_len = 0;

static USBD_CDC_LineCodingTypeDef   line_coding =
{
        460800,     // baud rate
        0x00,       // stop bits-1
        0x00,       // parity - none
        0x08        // nb. of bits 8
};

USBD_HandleTypeDef      husbd;

/* ****************************************************************************************************************** */

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&husbd, usr_tx_buf, 0);
    USBD_CDC_SetRxBuffer(&husbd, usr_rx_buf);
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    return USBD_OK;
}

/**
 * @brief  CDC_Control_FS
 *         Manage the CDC class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    switch (cmd) {

        /*******************************************************************************/
        /* Line Coding Structure                                                       */
        /*-----------------------------------------------------------------------------*/
        /* Offset | Field       | Size | Value  | Description                          */
        /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
        /* 4      | bCharFormat |   1  | Number | Stop bits                            */
        /*                                        0 - 1 Stop bit                       */
        /*                                        1 - 1.5 Stop bits                    */
        /*                                        2 - 2 Stop bits                      */
        /* 5      | bParityType |  1   | Number | Parity                               */
        /*                                        0 - None                             */
        /*                                        1 - Odd                              */
        /*                                        2 - Even                             */
        /*                                        3 - Mark                             */
        /*                                        4 - Space                            */
        /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
        /*******************************************************************************/
    case CDC_SET_LINE_CODING:

        line_coding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
        line_coding.format     = pbuf[4];
        line_coding.paritytype = pbuf[5];
        line_coding.datatype   = pbuf[6];

        break;

    case CDC_GET_LINE_CODING:

        pbuf[0] = (uint8_t)(line_coding.bitrate);
        pbuf[1] = (uint8_t)(line_coding.bitrate >> 8);
        pbuf[2] = (uint8_t)(line_coding.bitrate >> 16);
        pbuf[3] = (uint8_t)(line_coding.bitrate >> 24);
        pbuf[4] = line_coding.format;
        pbuf[5] = line_coding.paritytype;
        pbuf[6] = line_coding.datatype;

        break;

    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
    case CDC_SET_CONTROL_LINE_STATE:
    case CDC_SEND_BREAK:
    default:
        break;
    }

    return USBD_OK;
}

/**
 * @brief  CDC_Receive_FS
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         untill exiting this function. If you exit this function before transfer
 *         is complete on CDC interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    if (*Len > RX_DATA_SIZE)
        return USBD_FAIL;

    uint32_t    rx_idx = usr_rx_len;

    usr_rx_len += *Len;
    if (usr_rx_len > RX_DATA_SIZE)
    {
        rx_idx = 0;
        usr_rx_len = *Len;
    }

    USBD_CDC_SetRxBuffer(&husbd, &Buf[rx_idx]);
    USBD_CDC_ReceivePacket(&husbd);

    return USBD_OK;
}

/**
 * @brief  CDC_Transmit_FS
 *         Data send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be send
 * @param  Len: Number of data to be send (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    uint8_t result = USBD_OK;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)husbd.pClassData;


    if (hcdc->TxState != 0)
        return USBD_BUSY;

    USBD_CDC_SetTxBuffer(&husbd, Buf, Len);
    result = USBD_CDC_TransmitPacket(&husbd);

    return result;
}

int16_t CDC_ReadRx(uint8_t *buf, int16_t len)
{
    int16_t rxlen = usr_rx_len;

    if (rxlen > 0)
    {
        if (len < rxlen)
            rxlen = len;

        memcpy(buf, usr_rx_buf, rxlen);
        usr_rx_len = 0;
    }

    return rxlen;
}

/* ****************************************************************************************************************** */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {

        .Init    = CDC_Init_FS,
        .DeInit  = CDC_DeInit_FS,
        .Control = CDC_Control_FS,
        .Receive = CDC_Receive_FS
};

/* end of file ****************************************************************************************************** */
