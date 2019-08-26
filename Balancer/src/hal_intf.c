/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "hal_intf.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

/* ****************************************************************************************************************** */

struct sGpioData
{
    GPIO_TypeDef    *_gpio;
    uint16_t        _pins;
};
typedef struct sGpioData    sGpioData;

struct sPwmInData
{
    TIM_HandleTypeDef   *_pwm;
    uint32_t            _channel;
    uint32_t            _last_v;
    cbHalPwmIn          _callback;
    void                *_param;
};
typedef struct sPwmInData   sPwmInData;

struct sPwmOutData
{
    TIM_HandleTypeDef   *_pwm;
    uint32_t            _channel;
    int16_t             _ticks;
};
typedef struct sPwmOutData  sPwmOutData;

#define PWM_MAX_PERIOD      0xFFFF

extern TIM_HandleTypeDef    htim2;
extern TIM_HandleTypeDef    htim3;
extern TIM_HandleTypeDef    htim4;

struct sSpiData
{
    SPI_HandleTypeDef   *_spi;
    DMA_HandleTypeDef   *_dmatx;
    DMA_HandleTypeDef   *_dmarx;
    hHalGpio            _ncs;
    cbHalSpi            _callback;
    void                *_param;
};
typedef struct sSpiData     sSpiData;

extern SPI_HandleTypeDef    hspi3;
extern DMA_HandleTypeDef    hdma_spi3_tx;
extern DMA_HandleTypeDef    hdma_spi3_rx;

struct sUartData
{
    UART_HandleTypeDef  *_uart;
    DMA_HandleTypeDef   *_dmatx;
    DMA_HandleTypeDef   *_dmarx;
    cbHalUart           _callback;
    void                *_param;
};
typedef struct sUartData    sUartData;

extern UART_HandleTypeDef   huart1;
extern UART_HandleTypeDef   huart6;
extern DMA_HandleTypeDef    hdma_usart1_tx;
extern DMA_HandleTypeDef    hdma_usart1_rx;

/* ****************************************************************************************************************** */

static sGpioData    gpio_data[eHalGpio_N] =
{
        [eHalGpio_LED] =
        {
                ._gpio = GPIOB,
                ._pins = GPIO_PIN_9
        },
        [eHalGpio_BMP280_nCS] =
        {
                ._gpio = GPIOC,
                ._pins = GPIO_PIN_2
        },
        [eHalGpio_MPU9255_nCS] =
        {
                ._gpio = GPIOC,
                ._pins = GPIO_PIN_3
        },
        [eHalGpio_L298_In1] =
        {
                ._gpio = GPIOD,
                ._pins = GPIO_PIN_14
        },
        [eHalGpio_L298_In2] =
        {
                ._gpio = GPIOA,
                ._pins = GPIO_PIN_3
        },
        [eHalGpio_L298_In3] =
        {
                ._gpio = GPIOA,
                ._pins = GPIO_PIN_2
        },
        [eHalGpio_L298_In4] =
        {
                ._gpio = GPIOA,
                ._pins = GPIO_PIN_1
        }
};

static sPwmInData   pwmin_data[eHalPwmIn_N] =
{
        [eHalPwmIn_CPPM] =
        {
                ._pwm      = &htim3,
                ._channel  = TIM_CHANNEL_2,
                ._last_v   = 0,
                ._callback = NULL,
                ._param    = NULL
        }
};

static sPwmOutData  pwmout_data[eHalPwmOut_N] =
{
        [eHalPwmOut_MotorLeft] =
        {
                ._pwm     = &htim4,
                ._channel = TIM_CHANNEL_4,
                ._ticks   = 0
        },
        [eHalPwmOut_MotorRight] =
        {
                ._pwm     = &htim2,
                ._channel = TIM_CHANNEL_1,
                ._ticks   = 0
        }
};

static sSpiData     spi_data[eHalSpi_N] =
{
        [eHalSpi_Port1] =
        {
                ._spi      = &hspi3,
                ._dmatx    = &hdma_spi3_tx,
                ._dmarx    = &hdma_spi3_rx,
                ._ncs      = NULL,
                ._callback = NULL,
                ._param    = NULL
        }
};

static sUartData    uart_data[eHalUart_N] =
{
        [eHalUart_SerialRx] =
        {
                ._uart     = &huart6,
                ._dmatx    = NULL,
                ._dmarx    = NULL,
                ._callback = NULL,
                ._param    = NULL
        },
        [eHalUart_Telemetry] =
        {
                ._uart     = &huart1,
                ._dmatx    = &hdma_usart1_tx,
                ._dmarx    = &hdma_usart1_rx,
                ._callback = NULL,
                ._param    = NULL
        }
};

volatile uint32_t   tick_of_microsecond;    // clock count every milli-second

/* ****************************************************************************************************************** */

hHalGpio hal_gpio_init(eHalGpio gpio)
{
    GPIO_InitTypeDef    gpio_init;
    if (gpio < eHalGpio_N)
    {
        gpio_init.Pin       = gpio_data[gpio]._pins;
        gpio_init.Mode      = GPIO_MODE_OUTPUT_PP;
        gpio_init.Pull      = GPIO_NOPULL;
        gpio_init.Speed     = GPIO_SPEED_FREQ_MEDIUM;
        gpio_init.Alternate = 0;
        HAL_GPIO_Init(gpio_data[gpio]._gpio, &gpio_init);
        return (hHalGpio)&gpio_data[gpio];
    }
    return NULL;
}

void hal_gpio_set(hHalGpio hgpio)
{
    sGpioData   *this = (sGpioData*)hgpio;
    HAL_GPIO_WritePin(this->_gpio, this->_pins, GPIO_PIN_SET);
}

void hal_gpio_reset(hHalGpio hgpio)
{
    sGpioData   *this = (sGpioData*)hgpio;
    HAL_GPIO_WritePin(this->_gpio, this->_pins, GPIO_PIN_RESET);
}

hHalPwmIn hal_pwmin_init(eHalPwmIn pwm, cbHalPwmIn cb, void *param)
{
    if (pwm < eHalPwmIn_N)
    {
        if (pwmin_data[pwm]._pwm->State == HAL_TIM_STATE_RESET)
        {
            TIM_MasterConfigTypeDef master_config;

            // TIM3 is on the APB1 - 84MHz
            pwmin_data[pwm]._pwm->Init.Prescaler = HAL_RCC_GetPCLK1Freq() * 2 / 1000000;    // 1us per one period
            pwmin_data[pwm]._pwm->Init.Period    = PWM_MAX_PERIOD;

            HAL_TIM_OC_Init(pwmin_data[pwm]._pwm);

            master_config.MasterOutputTrigger = TIM_TRGO_RESET;
            master_config.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

            HAL_TIMEx_MasterConfigSynchronization(pwmin_data[pwm]._pwm, &master_config);
        }

        TIM_IC_InitTypeDef  init_param;

        init_param.ICPolarity  = TIM_ICPOLARITY_RISING;
        init_param.ICSelection = TIM_ICSELECTION_DIRECTTI;
        init_param.ICPrescaler = TIM_ICPSC_DIV1;
        init_param.ICFilter    = 0;

        HAL_TIM_IC_ConfigChannel(pwmin_data[pwm]._pwm, &init_param, pwmin_data[pwm]._channel);
        HAL_TIM_IC_Start_IT(pwmin_data[pwm]._pwm, pwmin_data[pwm]._channel);

        pwmin_data[pwm]._callback = cb;
        pwmin_data[pwm]._param    = param;

        return (hHalPwmIn)&pwmin_data[pwm];
    }
    return NULL;
}

hHalPwmOut hal_pwmout_init(eHalPwmOut pwm, uint32_t freq)
{
    if (pwm < eHalPwmOut_N)
    {
        uint32_t    clock_hz = HAL_RCC_GetPCLK1Freq() * 2;  // TIM2 & TIM4 is on the APB1 - 84MHz
        uint32_t    prescalar = 1;

        if (pwmout_data[pwm]._pwm->State == HAL_TIM_STATE_RESET)
        {
            TIM_MasterConfigTypeDef     master_config;

            while (1)
            {
                if (((clock_hz / prescalar) / freq) <= PWM_MAX_PERIOD)
                    break;
                prescalar++;
            }

            clock_hz /= prescalar;

            pwmout_data[pwm]._pwm->Init.Prescaler = prescalar - 1;
            pwmout_data[pwm]._pwm->Init.Period    = (clock_hz / freq) - 1;

            HAL_TIM_PWM_Init(pwmout_data[pwm]._pwm);

            master_config.MasterOutputTrigger = TIM_TRGO_RESET;
            master_config.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

            HAL_TIMEx_MasterConfigSynchronization(pwmout_data[pwm]._pwm, &master_config);
        }
        else
        {
            prescalar = pwmout_data[pwm]._pwm->Init.Prescaler + 1;
            clock_hz /= prescalar;
        }

        TIM_OC_InitTypeDef  init_param;

        init_param.OCMode     = TIM_OCMODE_PWM1;
        init_param.Pulse      = 0;
        init_param.OCPolarity = TIM_OCPOLARITY_HIGH;
        init_param.OCFastMode = TIM_OCFAST_ENABLE;

        HAL_TIM_PWM_ConfigChannel(pwmout_data[pwm]._pwm, &init_param, pwmout_data[pwm]._channel);
        HAL_TIM_PWM_Start(pwmout_data[pwm]._pwm, pwmout_data[pwm]._channel);

        pwmout_data[pwm]._ticks = (int16_t)(clock_hz / 1000000);    // ticks per 1us per

        return (hHalPwmOut)&pwmout_data[pwm];
    }
    return NULL;
}

void hal_pwmout_set_duty(hHalPwmOut hpwm, float duty_us)
{
    sPwmOutData *this = (sPwmOutData*)hpwm;
    __HAL_TIM_SET_COMPARE(this->_pwm, this->_channel, (uint32_t)(duty_us * (float)this->_ticks));
}

hHalSpi hal_spi_init(eHalSpi spi, eHalGpio gpio_ncs)
{
    if (spi < eHalSpi_N)
    {
        if (spi_data[spi]._spi->State == HAL_SPI_STATE_RESET)
        {
            if (HAL_SPI_Init(spi_data[spi]._spi) == HAL_OK)
            {
                spi_data[spi]._ncs = hal_gpio_init(gpio_ncs);
                return (hHalSpi)&spi_data[spi];
            }
        }
    }
    return NULL;
}

bool hal_spi_set_speed(hHalSpi hspi, uint32_t khz)
{
    sSpiData    *this = (sSpiData*)hspi;
    int16_t     prescalar;
    int32_t     freq_khz;

    // SPI3 is on the APB1 - 84MHz
    freq_khz = HAL_RCC_GetPCLK1Freq() / 1000;
    for (prescalar = 1; prescalar <= 8; prescalar++)
    {
        if ((freq_khz / (1 << prescalar)) <= khz)
            break;
    }

    switch (1 << prescalar)
    {
    case 2:
        this->_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        break;
    case 4:
        this->_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
        break;
    case 8:
        this->_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        break;
    case 16:
        this->_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        break;
    case 32:
        this->_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
        break;
    case 64:
        this->_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        break;
    case 128:
        this->_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
        break;
    default:
    case 256:
        this->_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
        break;
    }

    return (HAL_SPI_Init(this->_spi) == HAL_OK) ? true : false;
}

bool hal_spi_tx(hHalSpi hspi, uint8_t *dat, int16_t len, cbHalSpi cb, void *param)
{
    sSpiData    *this = (sSpiData*)hspi;

    this->_callback = cb;
    this->_param    = param;
    if (HAL_SPI_Transmit_DMA(this->_spi, dat, len) == HAL_OK)
        return true;

    this->_callback = NULL;
    this->_param    = NULL;
    return false;
}

bool hal_spi_rx(hHalSpi hspi, uint8_t *buf, int16_t len, cbHalSpi cb, void *param)
{
    sSpiData    *this = (sSpiData*)hspi;

    this->_callback = cb;
    this->_param    = param;
    if (HAL_SPI_Receive_DMA(this->_spi, buf, len) == HAL_OK)
        return true;

    this->_callback = NULL;
    this->_param    = NULL;
    return false;
}

bool hal_spi_txrx(hHalSpi hspi, uint8_t *tx, uint8_t *rx, int16_t len, cbHalSpi cb, void *param)
{
    sSpiData    *this = (sSpiData*)hspi;

    this->_callback = cb;
    this->_param    = param;
    if (HAL_SPI_TransmitReceive_DMA(this->_spi, tx, rx, len) == HAL_OK)
        return true;

    this->_callback = NULL;
    this->_param    = NULL;
    return false;
}

int16_t hal_spi_tx_timeout(hHalSpi hspi, uint8_t *dat, int16_t len, uint32_t to_ms)
{
    sSpiData    *this = (sSpiData*)hspi;
    if (HAL_SPI_Transmit(this->_spi, dat, len, to_ms) == HAL_OK)
        return this->_spi->TxXferSize - this->_spi->TxXferCount - 1;
    return len;
}

int16_t hal_spi_rx_timeout(hHalSpi hspi, uint8_t *buf, int16_t len, uint32_t to_ms)
{
    sSpiData    *this = (sSpiData*)hspi;
    if (HAL_SPI_Receive(this->_spi, buf, len, to_ms) == HAL_OK)
        return this->_spi->RxXferSize - this->_spi->RxXferCount - 1;
    return len;
}

int16_t hal_spi_txrx_timeout(hHalSpi hspi, uint8_t *tx, uint8_t *rx, int16_t len, uint32_t to_ms)
{
    sSpiData    *this = (sSpiData*)hspi;
    if (HAL_SPI_TransmitReceive(this->_spi, tx, rx, len, to_ms) == HAL_OK)
        return this->_spi->RxXferSize - this->_spi->RxXferCount - 1;
    return len;
}

bool hal_spi_lock(hHalSpi hspi)
{
    sSpiData    *this = (sSpiData*)hspi;
    if (this->_spi->Lock == HAL_UNLOCKED)
    {
        hal_gpio_reset(this->_ncs);
        return true;
    }
    return false;
}

void hal_spi_unlock(hHalSpi hspi)
{
    sSpiData    *this = (sSpiData*)hspi;
    hal_gpio_set(this->_ncs);
}

hHalUart hal_uart_init(eHalUart uart, uint32_t baudrate, uint8_t options, cbHalUart cb, void *param)
{
    if (uart < eHalUart_N)
    {
        uart_data[uart]._uart->Init.BaudRate = baudrate;

        switch (options & HAL_UART_PARITY_MASK)
        {
        case HAL_UART_PARITY_EVEN:
            uart_data[uart]._uart->Init.Parity     = UART_PARITY_EVEN;
            uart_data[uart]._uart->Init.WordLength = UART_WORDLENGTH_9B;
            break;
        case HAL_UART_PARITY_ODD:
            uart_data[uart]._uart->Init.Parity     = UART_PARITY_ODD;
            uart_data[uart]._uart->Init.WordLength = UART_WORDLENGTH_9B;
            break;
        default:
        case HAL_UART_PARITY_NONE:
            uart_data[uart]._uart->Init.Parity     = UART_PARITY_NONE;
            uart_data[uart]._uart->Init.WordLength = UART_WORDLENGTH_8B;
            break;
        }

        switch (options & HAL_UART_STOP_MASK)
        {
        case HAL_UART_STOP_2B:
            uart_data[uart]._uart->Init.StopBits = UART_STOPBITS_2;
            break;
        default:
        case HAL_UART_STOP_1B:
            uart_data[uart]._uart->Init.StopBits = UART_STOPBITS_1;
            break;
        }

        switch (options & HAL_UART_MODE_MASK)
        {
        case HAL_UART_MODE_TX:
            uart_data[uart]._uart->Init.Mode = UART_MODE_TX;
            break;
        case HAL_UART_MODE_RX:
            uart_data[uart]._uart->Init.Mode = UART_MODE_RX;
            break;
        case HAL_UART_MODE_TXRX:
            uart_data[uart]._uart->Init.Mode = UART_MODE_TX_RX;
            break;
        }

        if (HAL_UART_Init(uart_data[uart]._uart) == HAL_OK)
        {
            uart_data[uart]._callback = cb;
            uart_data[uart]._param    = param;
            return (hHalUart)&uart_data[uart];
        }
    }
    return NULL;
}

bool hal_uart_tx(hHalUart huart, uint8_t *dat, int16_t len)
{
    sUartData   *this = (sUartData*)huart;
    HAL_StatusTypeDef   st;
    if (this->_dmatx)
        st = HAL_UART_Transmit_DMA(this->_uart, dat, len);
    else
        st = HAL_UART_Transmit_IT(this->_uart, dat, len);
    return (st == HAL_OK) ? true : false;
}

bool hal_uart_rx(hHalUart huart, uint8_t *buf, int16_t len)
{
    sUartData   *this = (sUartData*)huart;
    HAL_StatusTypeDef   st;
    if (this->_dmarx)
        st = HAL_UART_Receive_DMA(this->_uart, buf, len);
    else
        st = HAL_UART_Receive_IT(this->_uart, buf, len);
    return (st == HAL_OK) ? true : false;
}

int16_t hal_uart_tx_timeout(hHalUart huart, uint8_t *dat, int16_t len, uint32_t to_ms)
{
    sUartData   *this = (sUartData*)huart;
    if (HAL_UART_Transmit(this->_uart, dat, len, to_ms) != HAL_OK)
        return this->_uart->TxXferSize - this->_uart->TxXferCount - 1;
    return len;
}

int16_t hal_uart_rx_timeout(hHalUart huart, uint8_t *buf, int16_t len, uint32_t to_ms)
{
    sUartData   *this = (sUartData*)huart;
    if (HAL_UART_Receive(this->_uart, buf, len, to_ms) != HAL_OK)
        return this->_uart->RxXferSize - this->_uart->RxXferCount - 1;
    return len;
}

void hal_vcp_init(void)
{
    USBD_Init(&husbd, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&husbd, &USBD_CDC);
    USBD_CDC_RegisterInterface(&husbd, &USBD_Interface_fops_FS);
    USBD_Start(&husbd);
}

int16_t hal_vcp_tx_timeout(uint8_t* dat, int16_t len, uint32_t to_ms)
{
    (void)to_ms;
    return (CDC_Transmit_FS(dat, len) == USBD_OK) ? len : 0;
}

int16_t hal_vcp_rx_timeout(uint8_t* buf, int16_t len, uint32_t to_ms)
{
    int16_t nrx = 0;
    int16_t tmp;
    while (to_ms > 0)
    {
        tmp = CDC_ReadRx(buf + nrx, len - nrx);
        nrx += tmp;
        if (nrx >= len)
            break;
        to_ms--;
        HAL_Delay(1);
    }
    return nrx;
}

inline uint32_t hal_get_micros(void)
{
    volatile uint32_t   now_ticks = SysTick->VAL;   // down count
    return HAL_GetTick() * 1000 + (1000 - (now_ticks / tick_of_microsecond));
}

inline void hal_delay_micros(uint32_t us)
{
    volatile uint32_t   target_us = hal_get_micros() + us;
    while (hal_get_micros() < target_us)
        __asm("nop\r\n");
}

inline uint32_t hal_get_millis(void)
{
    return HAL_GetTick();
}

inline void hal_delay_millis(uint32_t ms)
{
    HAL_Delay(ms);
}

/* ****************************************************************************************************************** */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    for (int8_t i = 0; i < eHalPwmIn_N; i++)
    {
        if (pwmin_data[i]._pwm == htim)
        {
            sPwmInData  *this = &pwmin_data[i];
            uint32_t    now_value = HAL_TIM_ReadCapturedValue(htim, this->_channel);
            uint32_t    duty_us;

            if (now_value >= this->_last_v)
                duty_us = now_value - this->_last_v;
            else
                duty_us = PWM_MAX_PERIOD - this->_last_v + now_value;

            if (this->_callback)
                this->_callback(duty_us, this->_param);

            break;
        }
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for (int8_t i = 0; i < eHalSpi_N; i++)
    {
        if (spi_data[i]._spi == hspi)
        {
            if (spi_data[i]._callback)
                spi_data[i]._callback(hspi->TxXferSize, false, spi_data[i]._param);
            break;
        }
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for (int8_t i = 0; i < eHalSpi_N; i++)
    {
        if (spi_data[i]._spi == hspi)
        {
            if (spi_data[i]._callback)
                spi_data[i]._callback(hspi->RxXferSize, false, spi_data[i]._param);
            break;
        }
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for (int8_t i = 0; i < eHalSpi_N; i++)
    {
        if (spi_data[i]._spi == hspi)
        {
            if (spi_data[i]._callback)
                spi_data[i]._callback(hspi->RxXferSize, false, spi_data[i]._param);
            break;
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    for (int8_t i = 0; i < eHalSpi_N; i++)
    {
        if (spi_data[i]._spi == hspi)
        {
            if (spi_data[i]._callback)
                spi_data[i]._callback(0, true, spi_data[i]._param);
            break;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (int8_t i = 0; i < eHalUart_N; i++)
    {
        if (uart_data[i]._uart == huart)
        {
            if (uart_data[i]._callback)
                uart_data[i]._callback(huart->TxXferSize, false, uart_data[i]._param);
            break;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (int8_t i = 0; i < eHalUart_N; i++)
    {
        if (uart_data[i]._uart == huart)
        {
            if (uart_data[i]._callback)
                uart_data[i]._callback(huart->RxXferSize, false, uart_data[i]._param);
            break;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (int8_t i = 0; i < eHalUart_N; i++)
    {
        if (uart_data[i]._uart == huart)
        {
            if (uart_data[i]._callback)
                uart_data[i]._callback(0, true, uart_data[i]._param);
            break;
        }
    }
}

/* end of file ****************************************************************************************************** */
