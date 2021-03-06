#include <stdint.h>
#include <stdbool.h>
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "usb_bulk_structs.h"

//*****************************************************************************
// The languages supported by this device.
//*****************************************************************************
const uint8_t g_pui8LangDescriptor[] = {
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

//*****************************************************************************
// The manufacturer string.
//*****************************************************************************
const uint8_t g_pui8ManufacturerString[] = {
    (17 + 1) * 2,
    USB_DTYPE_STRING,
    'T', 0, 'a', 0, 'u', 0, ' ', 0, 'S', 0, 'y', 0, 's', 0, 't', 0, 'e', 0,
    'm', 0, 's', 0, ',', 0, ' ', 0, 'I', 0, 'n', 0, 'c', 0, '.', 0,
};

//*****************************************************************************
// The product string.
//*****************************************************************************
const uint8_t g_pui8ProductString[] = {
    (19 + 1) * 2,
    USB_DTYPE_STRING,
    'M', 0, 'W', 0, 'P', 0, 'C', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 't', 0,
    'r', 0, 'o', 0, 'l', 0, 'l', 0, 'e', 0, 'r', 0, ' ', 0, ' ', 0, ' ', 0,
    ' ', 0
};

//*****************************************************************************
// The serial number string.
//*****************************************************************************
const uint8_t g_pui8SerialNumberString[] = {
    (8 + 1) * 2,
    USB_DTYPE_STRING,
    'M', 0, 'W', 0, 'P', 0, 'C', 0, '0', 0, '0', 0, 'A', 0, '1', 0
};

//*****************************************************************************
// The data interface description string.
//*****************************************************************************
const uint8_t g_pui8DataInterfaceString[] = {
    (19 + 1) * 2,
    USB_DTYPE_STRING,
    'B', 0, 'u', 0, 'l', 0, 'k', 0, ' ', 0, 'D', 0, 'a', 0, 't', 0,
    'a', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0,
    'a', 0, 'c', 0, 'e', 0
};

//*****************************************************************************
// The configuration description string.
//*****************************************************************************
const uint8_t g_pui8ConfigString[] = {
    (23 + 1) * 2,
    USB_DTYPE_STRING,
    'B', 0, 'u', 0, 'l', 0, 'k', 0, ' ', 0, 'D', 0, 'a', 0, 't', 0,
    'a', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 'f', 0, 'i', 0, 'g', 0,
    'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0, 'o', 0, 'n', 0
};

//*****************************************************************************
// The descriptor string table.
//*****************************************************************************
const uint8_t *const g_ppui8StringDescriptors[] = {
    g_pui8LangDescriptor,
    g_pui8ManufacturerString,
    g_pui8ProductString,
    g_pui8SerialNumberString,
    g_pui8DataInterfaceString,
    g_pui8ConfigString
};

#define NUM_STRING_DESCRIPTORS (sizeof(g_ppui8StringDescriptors) / sizeof(uint8_t *))

//*****************************************************************************
//
// The bulk device initialization and customization structures. In this case,
// we are using USBBuffers between the bulk device class driver and the
// application code. The function pointers and callback data values are set
// to insert a buffer in each of the data channels, transmit and receive.
//
// With the buffer in place, the bulk channel callback is set to the relevant
// channel function and the callback data is set to point to the channel
// instance data. The buffer, in turn, has its callback set to the application
// function and the callback data set to our bulk instance structure.
//
//*****************************************************************************
tUSBDBulkDevice g_sBulkDevice = {
    USB_VID_TI_1CBE,
    USB_PID_BULK,
    500,
    USB_CONF_ATTR_SELF_PWR,
    USBBufferEventCallback,
    (void *)&g_sRxBuffer,
    USBBufferEventCallback,
    (void *)&g_sTxBuffer,
    g_ppui8StringDescriptors,
    NUM_STRING_DESCRIPTORS
};

//*****************************************************************************
// Receive buffer (from the USB perspective).
//*****************************************************************************
uint8_t g_pui8USBRxBuffer[RX_BULK_BUFFER_SIZE];
tUSBBuffer g_sRxBuffer = {
    false,                          // This is a receive buffer.
    RxHandler,                      // pfnCallback
    (void *)&g_sBulkDevice,         // Callback data is our device pointer.
    USBDBulkPacketRead,             // pfnTransfer
    USBDBulkRxPacketAvailable,      // pfnAvailable
    (void *)&g_sBulkDevice,         // pvHandle
    g_pui8USBRxBuffer,              // pi8Buffer
    RX_BULK_BUFFER_SIZE,            // ui32BufferSize
};

//*****************************************************************************
// Transmit buffer (from the USB perspective).
//*****************************************************************************
uint8_t g_pui8USBTxBuffer[TX_BULK_BUFFER_SIZE];
tUSBBuffer g_sTxBuffer = {
    true,                           // This is a transmit buffer.
    TxHandler,                      // pfnCallback
    (void *)&g_sBulkDevice,         // Callback data is our device pointer.
    USBDBulkPacketWrite,            // pfnTransfer
    USBDBulkTxPacketAvailable,      // pfnAvailable
    (void *)&g_sBulkDevice,         // pvHandle
    g_pui8USBTxBuffer,              // pi8Buffer
    TX_BULK_BUFFER_SIZE,            // ui32BufferSize
};
