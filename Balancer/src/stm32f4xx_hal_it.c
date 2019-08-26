/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "stm32f4xx.h"

/* ****************************************************************************************************************** */

extern PCD_HandleTypeDef    hpcd;
extern SPI_HandleTypeDef    hspi3;
extern DMA_HandleTypeDef    hdma_spi3_tx;
extern DMA_HandleTypeDef    hdma_spi3_rx;
extern TIM_HandleTypeDef    htim3;
extern UART_HandleTypeDef   huart1;
extern DMA_HandleTypeDef    hdma_usart1_tx;
extern DMA_HandleTypeDef    hdma_usart1_rx;
extern UART_HandleTypeDef   huart6;

/* ****************************************************************************************************************** */

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}

void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi3);
}

void DMA1_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi3_tx);
}

void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi3_rx);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void DMA2_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart6);
}

/* end of file ****************************************************************************************************** */
