/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* ****************************************************************************************************************** */

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

/* ****************************************************************************************************************** */

enum eHalGpio
{
    eHalGpio_LED,
    eHalGpio_BMP280_nCS,
    eHalGpio_MPU9255_nCS,
    eHalGpio_L298_In1,
    eHalGpio_L298_In2,
    eHalGpio_L298_In3,
    eHalGpio_L298_In4,
    eHalGpio_N
};
typedef enum eHalGpio   eHalGpio;
typedef void*           hHalGpio;

enum eHalPwmIn
{
    eHalPwmIn_CPPM,
    eHalPwmIn_N
};
typedef enum eHalPwmIn  eHalPwmIn;
typedef void*           hHalPwmIn;
typedef void    (*cbHalPwmIn)(uint32_t dt_us, void *param);

enum eHalPwmOut
{
    eHalPwmOut_MotorLeft,
    eHalPwmOut_MotorRight,
    eHalPwmOut_N
};
typedef enum eHalPwmOut eHalPwmOut;
typedef void*           hHalPwmOut;

enum eHalSpi
{
    eHalSpi_Port1,      // BMP280 + MPU9255
    eHalSpi_N
};
typedef enum eHalSpi    eHalSpi;
typedef void*           hHalSpi;
typedef void    (*cbHalSpi)(int16_t len, bool error, void *param);

enum eHalUart
{
    eHalUart_SerialRx,
    eHalUart_Telemetry,
    eHalUart_N
};
typedef enum eHalUart   eHalUart;
typedef void*           hHalUart;
typedef void    (*cbHalUart)(int16_t len, bool error, void *param);

#define HAL_UART_PARITY_MASK    0b00011000
#define HAL_UART_PARITY_EVEN    0b00010000
#define HAL_UART_PARITY_ODD     0b00001000
#define HAL_UART_PARITY_NONE    0b00000000

#define HAL_UART_STOP_MASK      0b00000100
#define HAL_UART_STOP_2B        0b00000100
#define HAL_UART_STOP_1B        0b00000000

#define HAL_UART_MODE_MASK      0b00000011
#define HAL_UART_MODE_TX        0b00000010
#define HAL_UART_MODE_RX        0b00000001
#define HAL_UART_MODE_TXRX      0b00000011

/* ****************************************************************************************************************** */

hHalGpio    hal_gpio_init(eHalGpio gpio);
void        hal_gpio_set(hHalGpio hgpio);
void        hal_gpio_reset(hHalGpio hgpio);

hHalPwmIn   hal_pwmin_init(eHalPwmIn pwm, cbHalPwmIn cb, void *param);

hHalPwmOut  hal_pwmout_init(eHalPwmOut pwm, uint32_t freq);
void        hal_pwmout_set_duty(hHalPwmOut hpwm, float duty_us);

hHalSpi     hal_spi_init(eHalSpi spi, eHalGpio gpio_ncs);
bool        hal_spi_set_speed(hHalSpi hspi, uint32_t khz);
bool        hal_spi_tx(hHalSpi hspi, uint8_t *dat, int16_t len, cbHalSpi cb, void *param);
bool        hal_spi_rx(hHalSpi hspi, uint8_t *buf, int16_t len, cbHalSpi cb, void *param);
bool        hal_spi_txrx(hHalSpi hspi, uint8_t *tx, uint8_t *rx, int16_t len, cbHalSpi cb, void *param);
int16_t     hal_spi_tx_timeout(hHalSpi hspi, uint8_t *dat, int16_t len, uint32_t to_ms);
int16_t     hal_spi_rx_timeout(hHalSpi hspi, uint8_t *buf, int16_t len, uint32_t to_ms);
int16_t     hal_spi_txrx_timeout(hHalSpi hspi, uint8_t *tx, uint8_t *rx, int16_t len, uint32_t to_ms);
bool        hal_spi_lock(hHalSpi hspi);
void        hal_spi_unlock(hHalSpi hspi);

hHalUart    hal_uart_init(eHalUart uart, uint32_t baudrate, uint8_t options, cbHalUart cb, void *param);
bool        hal_uart_tx(hHalUart huart, uint8_t *dat, int16_t len);
bool        hal_uart_rx(hHalUart huart, uint8_t *buf, int16_t len);
int16_t     hal_uart_tx_timeout(hHalUart huart, uint8_t *dat, int16_t len, uint32_t to_ms);
int16_t     hal_uart_rx_timeout(hHalUart huart, uint8_t *buf, int16_t len, uint32_t to_ms);

void        hal_vcp_init(void);
int16_t     hal_vcp_tx_timeout(uint8_t* dat, int16_t len, uint32_t to_ms);
int16_t     hal_vcp_rx_timeout(uint8_t* buf, int16_t len, uint32_t to_ms);

uint32_t    hal_get_micros(void);
void        hal_delay_micros(uint32_t us);
uint32_t    hal_get_millis(void);
void        hal_delay_millis(uint32_t ms);

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

/* end of file ****************************************************************************************************** */
