/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "stm32f4xx.h"
#include "usbd_core.h"

/* ****************************************************************************************************************** */

extern PCD_HandleTypeDef    hpcd;

/* ****************************************************************************************************************** */

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    if (pdev->id == DEVICE_FS)
    {
        pdev->pData = &hpcd;
        hpcd.pData = pdev;
        hpcd.Instance                 = USB_OTG_FS;
        hpcd.Init.dev_endpoints       = 4;
        hpcd.Init.speed               = PCD_SPEED_FULL;
        hpcd.Init.dma_enable          = DISABLE;
        hpcd.Init.ep0_mps             = DEP0CTL_MPS_64;
        hpcd.Init.phy_itface          = PCD_PHY_EMBEDDED;
        hpcd.Init.Sof_enable          = DISABLE;
        hpcd.Init.low_power_enable    = DISABLE;
        hpcd.Init.lpm_enable          = DISABLE;
        hpcd.Init.vbus_sensing_enable = DISABLE;
        hpcd.Init.use_dedicated_ep1   = DISABLE;

        HAL_PCD_Init(&hpcd);
        HAL_PCDEx_SetRxFiFo(&hpcd, 0x80);
        HAL_PCDEx_SetTxFiFo(&hpcd, 0, 0x40);
        HAL_PCDEx_SetTxFiFo(&hpcd, 1, 0x80);
    }

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_DeInit(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Start(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Stop(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
    HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_Close(pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_Flush(pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
    return USBD_OK;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    PCD_HandleTypeDef *hpcd = pdev->pData;
    if ((ep_addr & 0x80) == 0x80)
        return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
    else
        return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    HAL_PCD_SetAddress(pdev->pData, dev_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size)
{
    HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size)
{
    HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}

/* end of file ****************************************************************************************************** */
