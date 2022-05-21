#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_adc.h"
#include "inc/hw_types.h"
#include "inc/hw_udma.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "drivers/pinout.h"
#include "usb_bulk_structs.h"

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
#define ADC_SAMPLE_NUM_OF_FITPOINTS 7

//*****************************************************************************
// Definition for ADC sampling frequency (Hz)
//*****************************************************************************
#define ADC_SAMPLING_FREQ 1333333

//*****************************************************************************
// Definition for number of analog channels per ADC device
//*****************************************************************************
#define NUM_OF_CHANNELS 32

//*****************************************************************************
// Definition for ADC buffer size.
//*****************************************************************************
#define ADC_SAMPLE_BUF_SIZE NUM_OF_CHANNELS * ADC_SAMPLE_NUM_OF_FITPOINTS

//*****************************************************************************
// Global buffers to store ADC sample data.
//*****************************************************************************
static uint16_t pui16ADCBuffer[ADC_SAMPLE_BUF_SIZE];

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line) {}
#endif

//*************************************************************************************************************************************
//  Configure system clock
//*************************************************************************************************************************************

//****************************************************************************
// System clock rate in Hz.
//****************************************************************************
uint32_t g_ui32SysClock;

void ConfigureSystemClock(void) {
    // Run from the PLL at 120 MHz.
    g_ui32SysClock = MAP_SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240,
                                            120000000);
}

//*************************************************************************************************************************************
//  Configure GPIO Ports
//*************************************************************************************************************************************

// SELECT BITS (For ADG732 analog multiplexer)
uint8_t g_ui8SelectBits;

// Flag of ADC data on USB FIFO
bool g_bADCDataOnUSBFIFO;

//*****************************************************************************
// The interrupt handler for PortN2
//*****************************************************************************

void PortNIntHandler(void) {
    // Clear Interrupt flag
    MAP_GPIOIntClear(GPIO_PORTN_BASE, GPIO_INT_PIN_2);

    // If PN2 at high-level, start digitizing by starting TIMER and uDMA services.
    // If PN2 at low-level, disable TIMER, perform other non-ADC services (display with UART, USB/Ethernet to PC, etc).
    if (MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) == 4) {

        // This is the time delay between the shot trigger and the actually laser fired
        // Need to change this based on the specific LASER system setup.
        // MAP_SysCtlDelay(1);

        // Set up the transfer parameters for the ADC0-SS3 pri/alt control structure. The mode is set to ping-pong, the transfer source is the ADC Sample
        // Sequence Result FIFO3 register, and the destination is the receive "A" buffer.  The transfer size is set to match the size of the buffer.
        // Same for alternative control structure.
        MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void*) (ADC0_BASE + ADC_O_SSFIFO3),
                                   &pui16ADCBuffer,
                                   ADC_SAMPLE_BUF_SIZE);

        // Enables DMA channel so it can perform transfers.  As soon as the channels are enabled, the peripheral will issue a transfer request and
        // the data transfers will begin.
        MAP_uDMAChannelEnable(UDMA_CHANNEL_ADC3);

        // When digitizing, data buffer not fully filled yet
        g_bADCDataOnUSBFIFO = false;

        // Start timer by enabling its
        g_ui8SelectBits = 0;
        MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock / ADC_SAMPLING_FREQ) - 1);
        MAP_TimerEnable(TIMER0_BASE, TIMER_A);
    } else {
        // Disable the timer
        MAP_TimerDisable(TIMER0_BASE, TIMER_A);

        // Put the pins into a defined initial state (all high)
        g_ui8SelectBits = 63;
        MAP_GPIOPinWrite(GPIO_PORTM_BASE,     GPIO_PIN_0 |
                                              GPIO_PIN_1 |
                                              GPIO_PIN_2 |
                                              GPIO_PIN_3 |
                                              GPIO_PIN_4 |
                                              GPIO_PIN_5, g_ui8SelectBits);

        // TODO: Setup another uDMA to move data within RAM.
        // 1. Copy data from pui16ADCBuffer (ADC data buffer)
        // to g_pui8USBTxBuffer (USB TX buffer).
        // 2. Now there is a catch. pui16ADCBuffer is 16-bit
        // while g_pui8USBTxBuffer is 8-bit. Need some code
        // to handle the bit breakup and re-order.

        // At this point, data buffer should be already filled
        g_bADCDataOnUSBFIFO = true;
    }
}

void ConfigureGPIOPorts(void) {
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    // Enable GPIO_PortM pin[0-5] for SELECT signals
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0 |
                                               GPIO_PIN_1 |
                                               GPIO_PIN_2 |
                                               GPIO_PIN_3 |
                                               GPIO_PIN_4 |
                                               GPIO_PIN_5);

    // Put the pins into a defined initial state (all high)
    g_ui8SelectBits = 63;
    MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0 |
                                      GPIO_PIN_1 |
                                      GPIO_PIN_2 |
                                      GPIO_PIN_3 |
                                      GPIO_PIN_4 |
                                      GPIO_PIN_5, g_ui8SelectBits);

    // Enable the GPIO pin for ADC0 Channel 0 (PE3) which configures it for analog functionality.
    MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    // Enable pin PB4 to trigger
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_4);
    MAP_GPIOADCTriggerEnable(GPIO_PORTB_BASE, GPIO_PIN_4);

    // Enable pin PN2 to mimic LASER shot trigger
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_2);
    MAP_GPIOIntTypeSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
    MAP_GPIOIntRegister(GPIO_PORTN_BASE, &PortNIntHandler);
    MAP_GPIOIntEnable(GPIO_PORTN_BASE, GPIO_INT_PIN_2);
}

//*************************************************************************************************************************************
// Configure UART
//*************************************************************************************************************************************
void ConfigureUART(void) {
    // Enable UART0
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

//*************************************************************************************************************************************
//  Configure uDMA for ADC0 channel
//*************************************************************************************************************************************

//*****************************************************************************
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
// The count of uDMA errors.
// This value is incremented by the uDMA error handler.
//*****************************************************************************
static uint32_t g_ui32DMAErrCount = 0u;

void ConfigureUDMA_ADC0(void) {
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    // Enable the uDMA controller.
    MAP_uDMAEnable();

    // Point at the control table to use for channel control structures.
    MAP_uDMAControlBaseSet(pui8ControlTable);

    // Put the attributes in a known state for the uDMA ADC0-SS3 channel.  These should already be disabled by default.
    MAP_uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC3, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    // Configure the control parameters for the pri/alt control structure for the ADC0-SS3 channel.  The primary control structure is used for the "A"
    // part of the ping-pong receive.  The transfer data size is 16 bits, the source address does not increment since it will be reading from a
    // register.  The destination address increment is 16-bits.  The arbitration size is set to 1 byte transfers.
    MAP_uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1024);

    // Set up the transfer parameters for the ADC0-SS3 pri/alt control structure. The mode is set to ping-pong, the transfer source is the ADC Sample
    // Sequence Result FIFO3 register, and the destination is the receive "A" buffer.  The transfer size is set to match the size of the buffer.
    // Same for alternative control structure.
    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void*) (ADC0_BASE + ADC_O_SSFIFO3),
                               &pui16ADCBuffer,
                               ADC_SAMPLE_BUF_SIZE);

    // Set the USEBURST attribute for the uDMA ADC0-SS3 channel.  This will force the controller to always use a burst when transferring data from the
    // TX buffer to the UART.  This is somewhat more efficient bus usage than the default which allows single or burst transfers.
    MAP_uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC3, UDMA_ATTR_USEBURST);

    // Enables DMA channel so it can perform transfers.  As soon as the channels are enabled, the peripheral will issue a transfer request and
    // the data transfers will begin.
    MAP_uDMAChannelEnable(UDMA_CHANNEL_ADC3);
}

//*****************************************************************************
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//*****************************************************************************
void uDMAErrorHandler(void) {
    uint32_t ui32Status;

    // Check for uDMA error bit.
    ui32Status = uDMAErrorStatusGet();

    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    if(ui32Status) {
        uDMAErrorStatusClear();
        g_ui32DMAErrCount++;
    }
}

//*************************************************************************************************************************************
//  Configure ADC0
//*************************************************************************************************************************************
void ConfigureADC0(void) {
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Use ADC0 sequence 3 to sample channel PE3 once for each timer period.
    // Use PLL 480MHz divided by 15 to 32MHz clock for the ADC
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 15);

    // Wait for the clock configuration to set.
    MAP_SysCtlDelay(10);

    // Disable the ADC0 sequence 3 interrupt on the processor (NVIC).
    // Then disable interrupts for ADC0 sample sequence 3 to configure it.
    // Then disable ADC0 sample sequence 3.  With the sequence disabled, it is now safe to load the new configuration parameters.
    MAP_IntDisable(INT_ADC0SS3);
    MAP_ADCIntDisable(ADC0_BASE, 3);
    MAP_ADCSequenceDisable(ADC0_BASE, 3);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the conversion.
    MAP_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_EXTERNAL, 0);

    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in single-ended mode (default). Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence 3 has only 1 programmable steps.
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_END);

    // Since sample sequence 0 is now configured, it must be enabled.
    // Then clear the interrupt status flag.  This is done to make sure the interrupt flag is cleared before we sample.
    // Then enables the DMA channel for the ADC0 sample sequence 0.
    // Then enable the ADC 0 sample sequence 0 interrupt.
    // Then enable the interrupt for ADC0 sequence 0 on the processor (NVIC).
    MAP_ADCSequenceEnable(ADC0_BASE, 3);
    MAP_ADCIntClear(ADC0_BASE, 3);
    MAP_ADCSequenceDMAEnable(ADC0_BASE, 3);
    MAP_ADCIntEnable(ADC0_BASE, 3);
    MAP_IntEnable(INT_ADC0SS3);
}

//*************************************************************************************************************************************
//  Configure Periodic Timer
//*************************************************************************************************************************************
void ConfigureTimer(void) {
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Configure the periodic timer.
    // When running at high frequencies, try to use the 16-bit half-width counter.
    // At low frequencies, use the 32-bit full-width counter.
    if (ADC_SAMPLING_FREQ > 2000) {
        MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    } else {
        MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    }

    // Set ADG732 SELECT switch to 250kHz (TIMER1)
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock / ADC_SAMPLING_FREQ) - 1);

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    // Enable TIMER0 interrupt
    MAP_IntEnable(INT_TIMER0A);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

//*****************************************************************************
// The interrupt handler for the first timer interrupt.
//*****************************************************************************
void Timer0IntHandler(void) {
    // Clear the timer interrupt.
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Output the value of SELECT bits to PM0-5 pins
    MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0 |
                                      GPIO_PIN_1 |
                                      GPIO_PIN_2 |
                                      GPIO_PIN_3 |
                                      GPIO_PIN_4 |
                                      GPIO_PIN_5, g_ui8SelectBits);
    g_ui8SelectBits++;
}

//*************************************************************************************************************************************
//  Configure USB
//*************************************************************************************************************************************

// Flags used to pass commands from interrupt context to the main loop.
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002
volatile uint32_t g_ui32Flags = 0;

// Global flag indicating that a USB configuration has been set.
static volatile bool g_bUSBConfigured = false;

//*****************************************************************************
// Process the data request from host
//*****************************************************************************
typedef enum {
    DATA_AVAILABILITY_QUERY = 114,
    DATA_TRANSFER_REQUEST = 117,
    DATA_RECEIVED_ACKNOWLEDGE = 137
} HOSTREQ_T;

static uint32_t ProcessHostRequest(tUSBDBulkDevice *psDevice, uint8_t *pi8Data, uint_fast32_t ui32NumBytes) {
    /*
     * How the device-host communication works in this case.
     * 1. PC is the host. USB bus is controlled by the host
     * so the host needs to be the one initiate any comms.
     * 2. Host will start by asking if device has produced
     * data and put the data on the USB transfer buffer yet
     * by sending a DATA_AVAILABILITY_QUERY.
     * 3. Device responses to DATA_AVAILABILITY_QUERY depending
     * on where or not the data filled the TX buffer. If the
     * all the data is not there yet, DEV_DATA_UNAVAILABLE is
     * the response. On the other hand, if all the data are
     * present in the TX buffer and ready to be sent to host,
     * the device responses with DEV_DATA_AVAILABLE. The device
     * actual response will be 4 identical bytes followed by
     * a terminal byte
     *     115-115-115-115-000: DEV_DATA_AVAILABLE
     *     116-116-116-116-000: DEV_DATA_UNAVAILABLE
     * 4. If the host receives DEV_DATA_UNAVAILABLE as the
     * response, it will wait for a few hundreds of microseconds
     * before sending another DATA_AVAILABILITY_QUERY. It will
     * keep doing this until the response from the device is
     * DEV_DATA_AVAILABLE.
     * 5. If the host receives DEV_DATA_AVAILABLE as the
     * response, it will send a DATA_TRANSFER_REQUEST. The
     * device will response to this request by start sending
     * the data. The first 5 bytes indicate to the host that
     * this package is the ADC sampling data:
     *     136-136-136-136-000
     * 6. If host receive the data, it will acknowledge that
     * by a DATA_RECEIVED_ACKNOWLEDGE. It will then wait for
     * some long period (~ milliseconds) before sending other
     * queries and requests. That wait time is enough for the
     * device to collect new data and put them on the USB
     * buffer.
     */

    HOSTREQ_T uiHostReq = (HOSTREQ_T) g_pui8USBRxBuffer[0];
    switch(uiHostReq) {
        case DATA_AVAILABILITY_QUERY: {
            if (g_bADCDataOnUSBFIFO) {
                // Response to DATA_AVAILABILITY_QUERY
                // Let host know data is available for transfer
                g_pui8USBTxBuffer[0] = 115;
                g_pui8USBTxBuffer[1] = 115;
                g_pui8USBTxBuffer[2] = 115;
                g_pui8USBTxBuffer[3] = 115;
                g_pui8USBTxBuffer[4] = 000;
                USBBufferDataWritten(&g_sTxBuffer, 5);
                return 5;
            } else {
                // Response to DATA_AVAILABILITY_QUERY
                // Let host know data is not available yet.
                g_pui8USBTxBuffer[0] = 116;
                g_pui8USBTxBuffer[1] = 116;
                g_pui8USBTxBuffer[2] = 116;
                g_pui8USBTxBuffer[3] = 116;
                g_pui8USBTxBuffer[4] = 000;
                USBBufferDataWritten(&g_sTxBuffer, 5);
                return 5;
            }
        }
        case DATA_TRANSFER_REQUEST: {
            // Writing token first (let host know that this
            // is the actual data, not just symbolic response).
            g_pui8USBTxBuffer[0] = 136;
            g_pui8USBTxBuffer[1] = 136;
            g_pui8USBTxBuffer[2] = 136;
            g_pui8USBTxBuffer[3] = 136;
            g_pui8USBTxBuffer[4] = 000;
            // TODO: Handling data transfer here
            // 1. Copy data from pui16ADCBuffer (ADC data buffer)
            // to g_pui8USBTxBuffer (USB TX buffer).
            // 2. Now there is a catch. pui16ADCBuffer is 16-bit
            // while g_pui8USBTxBuffer is 8-bit. Need some code
            // to handle the bit breakup and re-order.
            // FIXME: in the future, when handle 2 ADC channels
            // at the same time, need to change this line of
            // code as well.
            USBBufferDataWritten(&g_sTxBuffer, 5 + ADC_SAMPLE_BUF_SIZE * 2);
        }
        case DATA_RECEIVED_ACKNOWLEDGE: {
            // Data sent successfully to host.
            // Ready to put new data on USB FIFO.
            g_bADCDataOnUSBFIFO = false;
            break;
        }
        default:
            // Unknown request. Check host code.
            break;
    }

    return 0;
}

//*****************************************************************************
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//*****************************************************************************
uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData) {
    return 0;
}

//*****************************************************************************
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//*****************************************************************************
uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData) {
    switch(ui32Event) {
        case USB_EVENT_CONNECTED: {
            g_bUSBConfigured = true;
            g_ui32Flags |= COMMAND_STATUS_UPDATE;

            // Flush our buffers.
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            break;
        }

        case USB_EVENT_DISCONNECTED: {
            g_bUSBConfigured = false;
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            break;
        }

        case USB_EVENT_RX_AVAILABLE: {
            tUSBDBulkDevice* psDevice = (tUSBDBulkDevice*) pvCBData;
            return ProcessHostRequest(psDevice, pvMsgData, ui32MsgValue);
        }

        case USB_EVENT_SUSPEND:
            break;

        case USB_EVENT_RESUME:
            break;

        default:
            break;
    }

    return 0;
}

void ConfigureUSB(void) {
    uint32_t ui32PLLRate = 0;

    // Configure the device pins.
    PinoutSet(false, true);

    // Not configured initially.
    g_bUSBConfigured = false;

    // Tell the user what we are up to.
    UARTprintf("Configuring USB... \n");

    // Initialize the transmit and receive buffers.
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);

    // Tell the USB library the CPU clock and the PLL frequency.
    MAP_SysCtlVCOGet(SYSCTL_XTAL_25MHZ, &ui32PLLRate);
    USBDCDFeatureSet(0, USBLIB_FEATURE_CPUCLK, &g_ui32SysClock);
    USBDCDFeatureSet(0, USBLIB_FEATURE_USBPLL, &ui32PLLRate);

    // Initialize the USB stack for device mode.
    USBStackModeSet(0, eUSBModeDevice, 0);

    // Pass our device information to the USB library and place the device on the bus.
    USBDBulkInit(0, &g_sBulkDevice);

    // Wait for initial configuration to complete.
    UARTprintf("Waiting for host...\n");
}

//*************************************************************************************************************************************
//  MAIN
//*************************************************************************************************************************************
int main(void) {
    // Initially no data on USB FIFO
    g_bADCDataOnUSBFIFO = false;

    ConfigureSystemClock();
    ConfigureGPIOPorts();
    ConfigureUART();
    ConfigureUDMA_ADC0();
    ConfigureADC0();
    ConfigureTimer();
    ConfigureUSB();

    while (!g_bUSBConfigured) {
        // Have we been asked to update the status display?
        if (g_ui32Flags & COMMAND_STATUS_UPDATE) {
            g_ui32Flags &= ~COMMAND_STATUS_UPDATE;
            if (g_bUSBConfigured) {
                UARTprintf("Host Connected.     \n");
            }
        }
    }

    while (1) {
    }
}
