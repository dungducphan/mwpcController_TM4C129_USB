#ifndef __USB_BULK_STRUCTS_H__
#define __USB_BULK_STRUCTS_H__

//*************************************************************************************************************************************
//  GLOBAL VARIABLES
//*************************************************************************************************************************************

//*****************************************************************************
// Definition for number of ADC samples to perform software-fit (per channel)
// With TM4C129 without external USB-PHY, the maximum bandwidth is 12Mbps. If
// TM4C129 ADC is clocked at 1.5MSPS and each sample package data size is 4
// bytes (12-bit value + metadata), then the maximum number of fit points as
// a function of LASER shot frequency is as following:
//
//          LASER frequency             |      N_fit
//          ----------------------------|----------------
//          1kHz                        |      7
//          2kHz                        |      3
//          3kHz                        |      2
//          4kHz                        |      1
//
//*****************************************************************************
#define ADC_SAMPLE_NUM_OF_FITPOINTS 3

//*****************************************************************************
// Definition for ADC sampling frequency (Hz)
//*****************************************************************************
#define ADC_SAMPLING_FREQ 1000000

//*****************************************************************************
// Definition for number of analog channels per ADC device
//*****************************************************************************
#define NUM_OF_CHANNELS 32

//*****************************************************************************
// Definition for ADC buffer size.
//*****************************************************************************
#define ADC_SAMPLE_BUF_SIZE NUM_OF_CHANNELS * ADC_SAMPLE_NUM_OF_FITPOINTS

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
#define TX_EMPTY_BUFFER_SIZE 12
#define TX_BULK_BUFFER_SIZE  NUM_OF_CHANNELS*2 + 16 + 1
#define RX_BULK_BUFFER_SIZE  2

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
