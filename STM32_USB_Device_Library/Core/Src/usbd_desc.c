/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/* ****************************************************************************************************************** */

#define USBD_VID                        0x0483
#define USBD_PID                        0x5740
#define USBD_LANGID_STRING              1033
#define USBD_MANUFACTURER_STRING        (uint8_t*)"STMicroelectronics"
#define USBD_PRODUCT_STRING_FS          (uint8_t*)"STM32 Virtual ComPort"
#define USBD_SERIALNUMBER_STRING_FS     (uint8_t*)"00000000001A"
#define USBD_CONFIGURATION_STRING_FS    (uint8_t*)"CDC Config"
#define USBD_INTERFACE_STRING_FS        (uint8_t*)"CDC Interface"
#define USB_SIZ_BOS_DESC                0x0C

/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
        0x12,                       /*bLength */
        USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
        0x00,                       /* bcdUSB */
        0x02, 0x02,                 /*bDeviceClass*/
        0x02,                       /*bDeviceSubClass*/
        0x00,                       /*bDeviceProtocol*/
        USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
        LOBYTE(USBD_VID),           /*idVendor*/
        HIBYTE(USBD_VID),           /*idVendor*/
        LOBYTE(USBD_PID),           /*idVendor*/
        HIBYTE(USBD_PID),           /*idVendor*/
        0x00,                       /*bcdDevice rel. 2.00*/
        0x02,
        USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
        USBD_IDX_PRODUCT_STR,       /*Index of product string*/
        USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
        USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
        USB_LEN_LANGID_STR_DESC,
        USB_DESC_TYPE_STRING,
        LOBYTE(USBD_LANGID_STRING),
        HIBYTE(USBD_LANGID_STRING),
};

__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

/* ****************************************************************************************************************** */

static void int_to_unicode(uint32_t value , uint8_t *pbuf , uint8_t len)
{
    for (uint8_t idx = 0 ; idx < len; idx++)
    {
        if (((value >> 28)) < 0xA)
            pbuf[2 * idx] = (value >> 28) + '0';
        else
            pbuf[2 * idx] = (value >> 28) + 'A' - 10;
        value = value << 4;
        pbuf[2 * idx + 1] = 0;
    }
}

static void get_serial_no(void)
{
    memset(USBD_StrDesc, 0, sizeof(USBD_StrDesc));

    uint32_t    *uid_base = (uint32_t*)UID_BASE;
    uint32_t    deviceserial0, deviceserial1, deviceserial2;

    deviceserial0 = uid_base[0];
    deviceserial1 = uid_base[1];
    deviceserial2 = uid_base[2];

    deviceserial0 += deviceserial2;
    if (deviceserial0 != 0)
    {
        int_to_unicode(deviceserial0, &USBD_StrDesc[2] ,8);
        int_to_unicode(deviceserial1, &USBD_StrDesc[18] ,4);
    }
}

uint8_t* USBD_Class_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    *length = sizeof(USBD_DeviceDesc);
    return (uint8_t*)USBD_DeviceDesc;
}

uint8_t* USBD_Class_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    *length = sizeof(USBD_LangIDDesc);
    return (uint8_t*)USBD_LangIDDesc;
}

uint8_t* USBD_Class_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    USBD_GetString(USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t* USBD_Class_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    USBD_GetString(USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t* USBD_Class_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    get_serial_no();
    *length = strlen((char*)USBD_StrDesc);
    return USBD_StrDesc;
}

uint8_t* USBD_Class_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    USBD_GetString(USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t* USBD_Class_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    USBD_GetString(USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/* ****************************************************************************************************************** */

USBD_DescriptorsTypeDef FS_Desc =
{
        USBD_Class_DeviceDescriptor,
        USBD_Class_LangIDStrDescriptor,
        USBD_Class_ManufacturerStrDescriptor,
        USBD_Class_ProductStrDescriptor,
        USBD_Class_SerialStrDescriptor,
        USBD_Class_ConfigStrDescriptor,
        USBD_Class_InterfaceStrDescriptor,
};

/* end of file ****************************************************************************************************** */
