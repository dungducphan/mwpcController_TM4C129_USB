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
#include "driverlib/usb.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "drivers/pinout.h"
#include "usb_bulk_structs.h"

//*****************************************************************************
// Global value for LASER shot number
//*****************************************************************************
static uint32_t g_ui32LASERShotID;

//*****************************************************************************
// Global buffers to store ADC sample data.
//*****************************************************************************
static uint16_t pui16ADCBuffer[ADC_SAMPLE_BUF_SIZE];

//*****************************************************************************
// Global buffers to store ADC-USB conversion buffer.
//*****************************************************************************
static uint8_t pui8ADC2USBConversionBuffer[NUM_OF_CHANNELS*2 + 16];

//*****************************************************************************
// Constant empty buffer
//*****************************************************************************
static uint8_t pui8EmptyBuffer[12];

void SetEmptyBuffer(void) {
    pui8EmptyBuffer[0]  = 1;
    pui8EmptyBuffer[1]  = 1;
    pui8EmptyBuffer[2]  = 1;
    pui8EmptyBuffer[3]  = 1;

    pui8EmptyBuffer[4]  = 0;
    pui8EmptyBuffer[5]  = 0;
    pui8EmptyBuffer[6]  = 0;
    pui8EmptyBuffer[7]  = 0;

    pui8EmptyBuffer[8]  = 1;
    pui8EmptyBuffer[9]  = 1;
    pui8EmptyBuffer[10] = 1;
    pui8EmptyBuffer[11] = 1;
}

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
bool g_bADCDataOnConversionBuffer;

void RAMtoRAM_uDMA(void) {
    // Copy data from ADC buffer to USB Tx buffer
    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, pui16ADCBuffer, g_pui8USBTxBuffer, ADC_SAMPLE_BUF_SIZE * 2);

    // Start transfer
    MAP_uDMAChannelEnable(UDMA_CHANNEL_SW);
    MAP_uDMAChannelRequest(UDMA_CHANNEL_SW);
}

//*****************************************************************************
// Software-averaged for ADC buffer
//*****************************************************************************
void SoftwareAveragedADC(void) {
    uint8_t uiChCount = 0;
    uint8_t uiFitPointCount = 0;

    pui8ADC2USBConversionBuffer[0] = 0;
    pui8ADC2USBConversionBuffer[1] = 0;
    pui8ADC2USBConversionBuffer[2] = 0;
    pui8ADC2USBConversionBuffer[3] = 0;

    pui8ADC2USBConversionBuffer[4] = ((uint32_t)g_ui32LASERShotID >> 0) & 0xFF;
    pui8ADC2USBConversionBuffer[5] = ((uint32_t)g_ui32LASERShotID >> 8) & 0xFF;
    pui8ADC2USBConversionBuffer[6] = ((uint32_t)g_ui32LASERShotID >> 16) & 0xFF;
    pui8ADC2USBConversionBuffer[7] = ((uint32_t)g_ui32LASERShotID >> 24) & 0xFF;

    pui8ADC2USBConversionBuffer[8] = 0;
    pui8ADC2USBConversionBuffer[9] = 0;
    pui8ADC2USBConversionBuffer[10] = 0;
    pui8ADC2USBConversionBuffer[11] = 0;

    for (uiChCount = 0; uiChCount < NUM_OF_CHANNELS; uiChCount++) {
        for (uiFitPointCount = 0; uiFitPointCount < ADC_SAMPLE_NUM_OF_FITPOINTS; uiFitPointCount++) {
            pui16ADCBuffer[uiChCount] += pui16ADCBuffer[uiChCount + 32*uiFitPointCount];
        }
        pui16ADCBuffer[uiChCount] = pui16ADCBuffer[uiChCount]/ADC_SAMPLE_NUM_OF_FITPOINTS;

        // Conversion from 16-bit to 2 of 8-bit
        pui8ADC2USBConversionBuffer[12 + 2 * uiChCount + 0] = ((uint16_t)pui16ADCBuffer[uiChCount] >> 0) & 0xFF;
        pui8ADC2USBConversionBuffer[12 + 2 * uiChCount + 1] = ((uint16_t)pui16ADCBuffer[uiChCount] >> 8) & 0xFF;
    }

    pui8ADC2USBConversionBuffer[NUM_OF_CHANNELS*2 + 12] = 0;
    pui8ADC2USBConversionBuffer[NUM_OF_CHANNELS*2 + 13] = 0;
    pui8ADC2USBConversionBuffer[NUM_OF_CHANNELS*2 + 14] = 0;
    pui8ADC2USBConversionBuffer[NUM_OF_CHANNELS*2 + 15] = 0;

    g_bADCDataOnConversionBuffer = true;
}

//*****************************************************************************
// Restart TIMER
//*****************************************************************************
void RestartTimer(void) {
    g_ui8SelectBits = 0;
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock / ADC_SAMPLING_FREQ) - 1);
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}

//*****************************************************************************
// Reset timer after ADC sampling
//*****************************************************************************
void ResetTimer(void) {
    // Disable the timer
    MAP_TimerDisable(TIMER0_BASE, TIMER_A);

    // Put the pins into a defined initial state (all high)
    g_ui8SelectBits = 255;
    MAP_GPIOPinWrite(GPIO_PORTM_BASE,   GPIO_PIN_0 |
                                        GPIO_PIN_1 |
                                        GPIO_PIN_2 |
                                        GPIO_PIN_3 |
                                        GPIO_PIN_4 |
                                        GPIO_PIN_5, g_ui8SelectBits);
    // Software-averaged ADC buffer
    SoftwareAveragedADC();

/*
    // For Debug only
    // Measure the execution time of SoftwareAveragedADC();
    if (MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) == 0x04) RestartTimer();
*/
}

//*****************************************************************************
// The interrupt handler for PortN2
//*****************************************************************************

void PortNIntHandler(void) {
    // Clear Interrupt flag
    MAP_GPIOIntClear(GPIO_PORTN_BASE, GPIO_INT_PIN_2);

    // Update LASER shot ID
    g_ui32LASERShotID++;

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
    g_bADCDataOnConversionBuffer = false;

    // Restart timer
    RestartTimer();
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
    g_ui8SelectBits = 255;
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
    MAP_GPIOIntTypeSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);
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
//  Configure uDMA
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

// The count of uDMA errors. Incremented by the uDMA error handler.
static uint32_t g_ui32DMAErrCount = 0u;

//*****************************************************************************
// uDMA configuration
//*****************************************************************************
void ConfigureUDMA(void) {
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

    // Set the USEBURST attribute for the uDMA ADC0-SS3 channel.  This will force the controller to always use a burst when transferring data from the
    // TX buffer to the UART.  This is somewhat more efficient bus usage than the default which allows single or burst transfers.
    MAP_uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC3, UDMA_ATTR_USEBURST);
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

    if (g_ui8SelectBits >= ADC_SAMPLE_BUF_SIZE * 2) ResetTimer();
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

static uint32_t ProcessHostRequest(tUSBDBulkDevice *psDevice, uint8_t *pi8Data, uint_fast32_t ui32NumBytes) {
    /*
     * How the device-host communication works:
     *   1. PC is the host. USB bus is controlled by the host
     * so the host needs to be the one initiate any comms.
     *   2. Host keeps asking if device has data and wait for
     * host to response. There is a time out. If within
     * timeout, host receives no response, host will wait
     * for some time before asking again.
     *   3. If device has data, it transmits the data.
     */

    USBBufferFlush(&g_sTxBuffer);

    if (pi8Data[0] == 114) {
        if (g_bADCDataOnConversionBuffer) {
            USBBufferWrite(&g_sTxBuffer, pui8ADC2USBConversionBuffer, TX_BULK_BUFFER_SIZE);
        } else {
            USBBufferWrite(&g_sTxBuffer, pui8EmptyBuffer, TX_EMPTY_BUFFER_SIZE);
        }
    }

    USBBufferFlush(&g_sRxBuffer);

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
            tUSBDBulkDevice* psDevice;
            psDevice = (tUSBDBulkDevice *)pvCBData;
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
    MAP_SysCtlDelay(100);
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
    MAP_SysCtlDelay(100);
    UARTprintf("Waiting for host...\n");
}

//*************************************************************************************************************************************
//  MAIN
//*************************************************************************************************************************************
int main(void) {
    // Initially no data on USB buffer
    g_bADCDataOnConversionBuffer = false;

    // Initialize LASER shot ID
    g_ui32LASERShotID = 0;

    ConfigureSystemClock();
    ConfigureGPIOPorts();
    ConfigureUART();
    ConfigureUDMA();
    ConfigureADC0();
    ConfigureTimer();

    // For USB interface
    SetEmptyBuffer();
    ConfigureUSB();

    while (1) {
        // Have we been asked to update the status display?
        if (g_bUSBConfigured) {
            UARTprintf("Host Connected. \n");
            break;
        }
        MAP_SysCtlDelay(200);
    }

    while (1) {}
}
