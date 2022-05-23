#ifndef __USB_BULK_STRUCTS_H__
#define __USB_BULK_STRUCTS_H__

//*****************************************************************************
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
// The size of the transmit and receive buffers used. The buffer should be at
// least twice the size of a maximum-sized USB packet.
//*****************************************************************************
#define TX_BULK_BUFFER_SIZE 69
#define RX_BULK_BUFFER_SIZE 2

extern uint32_t RxHandler(void *pvCBData, uint32_t ui32Event,
                          uint32_t ui32MsgValue, void *pvMsgData);
extern uint32_t TxHandler(void *pvi32CBData, uint32_t ui32Event,
                          uint32_t ui32MsgValue, void *pvMsgData);

extern tUSBBuffer g_sTxBuffer;
extern tUSBBuffer g_sRxBuffer;
extern tUSBDBulkDevice g_sBulkDevice;
extern uint8_t g_pui8USBTxBuffer[];
extern uint8_t g_pui8USBRxBuffer[];

//*****************************************************************************
// Mark the end of the C bindings section for C++ compilers.
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __USB_BULK_STRUCTS_H__
