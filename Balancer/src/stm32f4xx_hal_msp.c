/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 *
 *         +--------------------------------------------------------+
 *         |                                                        |
 *         |                                                        |
 *         +--------------------------------------------------------+
 *
 *                                    [TIM4]  [TIM2]          [TIM3 or UART6]
 *                                    [C4][C3][C4][C3][C2][C1][C2 or Rx]
 * [UART3]  +5V B11 B10 GND           D15 D14 A03 A02 A01 A00 C07
 * [UART2]  +5V D05 D06 GND           +5V +5V +5V +5V +5V +5V +5V
 * [UART1]  +5V A09 A10 GND           GND GND GND GND GND GND GND
 *
 * *********************************************************************************************************************
 */

#include "stm32f4xx.h"
#include "usbd_core.h"

/* ****************************************************************************************************************** */

PCD_HandleTypeDef   hpcd;
SPI_HandleTypeDef   hspi3 =
{
        .Instance = SPI3,
        .Init.Mode              = SPI_MODE_MASTER,
        .Init.Direction         = SPI_DIRECTION_2LINES,
        .Init.DataSize          = SPI_DATASIZE_8BIT,
        .Init.CLKPolarity       = SPI_POLARITY_HIGH,
        .Init.CLKPhase          = SPI_PHASE_2EDGE,
        .Init.NSS               = SPI_NSS_SOFT,
        .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
        .Init.FirstBit          = SPI_FIRSTBIT_MSB,
        .Init.TIMode            = SPI_TIMODE_DISABLE,
        .Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE,
        .Init.CRCPolynomial     = 1
};
DMA_HandleTypeDef hdma_spi3_tx =
{
        .Instance = DMA1_Stream7,
        .Init.Channel             = DMA_CHANNEL_0,
        .Init.Direction           = DMA_MEMORY_TO_PERIPH,
        .Init.PeriphInc           = DMA_PINC_DISABLE,
        .Init.MemInc              = DMA_MINC_ENABLE,
        .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
        .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
        .Init.Mode                = DMA_NORMAL,
        .Init.Priority            = DMA_PRIORITY_LOW,
        .Init.FIFOMode            = DMA_FIFOMODE_DISABLE
};
DMA_HandleTypeDef hdma_spi3_rx =
{
        .Instance = DMA1_Stream0,
        .Init.Channel             = DMA_CHANNEL_0,
        .Init.Direction           = DMA_PERIPH_TO_MEMORY,
        .Init.PeriphInc           = DMA_PINC_DISABLE,
        .Init.MemInc              = DMA_MINC_ENABLE,
        .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
        .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
        .Init.Mode                = DMA_NORMAL,
        .Init.Priority            = DMA_PRIORITY_LOW,
        .Init.FIFOMode            = DMA_FIFOMODE_DISABLE
};
TIM_HandleTypeDef htim2 =
{
        .Instance = TIM2,
        .Init.Prescaler     = 0,
        .Init.CounterMode   = TIM_COUNTERMODE_UP,
        .Init.Period        = 0,
        .Init.ClockDivision = TIM_CLOCKDIVISION_DIV1
};
TIM_HandleTypeDef htim3 =
{
        .Instance = TIM3,
        .Init.Prescaler     = 0,
        .Init.CounterMode   = TIM_COUNTERMODE_UP,
        .Init.Period        = 0,
        .Init.ClockDivision = TIM_CLOCKDIVISION_DIV1,
};
TIM_HandleTypeDef htim4 =
{
        .Instance = TIM4,
        .Init.Prescaler     = 0,
        .Init.CounterMode   = TIM_COUNTERMODE_UP,
        .Init.Period        = 0,
        .Init.ClockDivision = TIM_CLOCKDIVISION_DIV1,
};
UART_HandleTypeDef huart1 =
{
        .Instance = USART1,
        .Init.BaudRate     = 115200,
        .Init.WordLength   = UART_WORDLENGTH_8B,
        .Init.StopBits     = UART_STOPBITS_1,
        .Init.Parity       = UART_PARITY_NONE,
        .Init.Mode         = UART_MODE_TX_RX,
        .Init.HwFlowCtl    = UART_HWCONTROL_NONE,
        .Init.OverSampling = UART_OVERSAMPLING_16
};
DMA_HandleTypeDef hdma_usart1_tx =
{
        .Instance = DMA2_Stream7,
        .Init.Channel             = DMA_CHANNEL_4,
        .Init.Direction           = DMA_MEMORY_TO_PERIPH,
        .Init.PeriphInc           = DMA_PINC_DISABLE,
        .Init.MemInc              = DMA_MINC_ENABLE,
        .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
        .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
        .Init.Mode                = DMA_NORMAL,
        .Init.Priority            = DMA_PRIORITY_LOW,
        .Init.FIFOMode            = DMA_FIFOMODE_DISABLE
};
DMA_HandleTypeDef hdma_usart1_rx =
{
        .Instance = DMA2_Stream2,
        .Init.Channel             = DMA_CHANNEL_4,
        .Init.Direction           = DMA_PERIPH_TO_MEMORY,
        .Init.PeriphInc           = DMA_PINC_DISABLE,
        .Init.MemInc              = DMA_MINC_ENABLE,
        .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
        .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
        .Init.Mode                = DMA_NORMAL,
        .Init.Priority            = DMA_PRIORITY_LOW,
        .Init.FIFOMode            = DMA_FIFOMODE_DISABLE
};
UART_HandleTypeDef huart6 =
{
        .Instance = USART6,
        .Init.BaudRate     = 115200,
        .Init.WordLength   = UART_WORDLENGTH_8B,
        .Init.StopBits     = UART_STOPBITS_1,
        .Init.Parity       = UART_PARITY_NONE,
        .Init.Mode         = UART_MODE_TX_RX,
        .Init.HwFlowCtl    = UART_HWCONTROL_NONE,
        .Init.OverSampling = UART_OVERSAMPLING_16
};

extern volatile uint32_t    tick_of_microsecond;    // clock count every milli-second

/* ****************************************************************************************************************** */

static void init_sys_clock(void)
{
    RCC_OscInitTypeDef  RCC_OscInitStruct;
    RCC_ClkInitTypeDef  RCC_ClkInitStruct;

    /**
     * Configure the main internal regulator output voltage
     */

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**
     * Initializes the CPU, AHB and APB busses clocks
     */

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 25;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;

    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /**
     * Initializes the CPU, AHB and APB busses clocks
     */

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

    /**
     * Configure the Systick interrupt time
     */

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**
     * Configure the Systick
     */

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    tick_of_microsecond = HAL_RCC_GetSysClockFreq() / 1000000;
}

static void hal_usb_reset(void)
{
    GPIO_InitTypeDef    gpio_init;

    gpio_init.Pin       = GPIO_PIN_12;
    gpio_init.Mode      = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull      = GPIO_NOPULL;
    gpio_init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = 0;

    HAL_GPIO_Init(GPIOA, &gpio_init);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(150);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(150);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    init_sys_clock();
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef    GPIO_InitStruct;

    if (hspi->Instance == SPI3) {

        __HAL_RCC_SPI3_CLK_ENABLE();

        /**
         * SPI3 GPIO Configuration
         * PB3     ------> SPI3_SCK
         * PB4     ------> SPI3_MISO
         * PB5     ------> SPI3_MOSI
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;

        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* SPI3 DMA Init */

        HAL_DMA_Init(&hdma_spi3_tx);
        HAL_DMA_Init(&hdma_spi3_rx);

        __HAL_LINKDMA(hspi, hdmatx, hdma_spi3_tx);
        __HAL_LINKDMA(hspi, hdmarx, hdma_spi3_rx);

        HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

        HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_oc)
{
    GPIO_InitTypeDef    GPIO_InitStruct;

    if (htim_oc->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();

        /**
         * TIM2 GPIO Configuration
         * PA0-WKUP     ------> TIM2_CH1
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_0;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (htim_oc->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();

        /**
         * TIM4 GPIO Configuration
         * PD15     ------> TIM4_CH4
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_15;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;

        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{
    GPIO_InitTypeDef    GPIO_InitStruct;

    if (htim_ic->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();

        /**
         * TIM3 GPIO Configuration
         * PC7     ------> TIM3_CH2
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;

        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef    GPIO_InitStruct;

    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();

        /**
         * USART1 GPIO Configuration
         * PA9     ------> USART1_TX
         * PA10     ------> USART1_RX
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 DMA Init */

        HAL_DMA_Init(&hdma_usart1_tx);
        HAL_DMA_Init(&hdma_usart1_rx);

        __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);
        __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

        HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

        HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    }
    else if (huart->Instance == USART6)
    {
        __HAL_RCC_USART6_CLK_ENABLE();

        /**
         * USART6 GPIO Configuration
         * PC6     ------> USART6_TX
         * PC7     ------> USART6_RX
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;

        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    GPIO_InitTypeDef    GPIO_InitStruct;

    if (hpcd->Instance == USB_OTG_FS)
    {
        /* Configure USB FS GPIOs */

        __HAL_RCC_GPIOA_CLK_ENABLE();

        hal_usb_reset();

        /* Configure DM DP Pins */

        GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Enable USB FS Clocks */

        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

        /* Set USBFS Interrupt priority to 6 */

        HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);

        /* Enable USBFS Interrupt */

        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage(hpcd->pData, (uint8_t*)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF(hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

    switch (hpcd->Init.speed)
    {
    case PCD_SPEED_HIGH:
        speed = USBD_SPEED_HIGH;
        break;
    case PCD_SPEED_FULL:
        speed = USBD_SPEED_FULL;
        break;
    default:
        speed = USBD_SPEED_FULL;
        break;
    }

    USBD_LL_SetSpeed(hpcd->pData, speed);
    USBD_LL_Reset(hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Suspend(hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Resume(hpcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevConnected(hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevDisconnected(hpcd->pData);
}

/* end of file ****************************************************************************************************** */
