#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "inc/hw_uart.h"

#define SIGNAL_SIZE 50*2 // (Fs/f)*Nc

uint32_t ui32SysClkFreq;
float signal[SIGNAL_SIZE];


// Configuração da Tabela de controle do uDMA
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif



// ---------------------------------------------
// UART
// ---------------------------------------------

void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,(UART_CONFIG_WLEN_8 |
            UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    UARTEnable(UART0_BASE);
}

// ---------------------------------------------
// uDMA
// ---------------------------------------------

void ConfigureUDMA(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA)){}

    uDMAEnable();
    uDMAControlBaseSet(pui8ControlTable);

    UARTDMAEnable(UART0_BASE, UART_DMA_TX | UART_DMA_RX);

    uDMAChannelAssign(UDMA_CH8_UART0RX);

    uDMAChannelAttributeDisable(UDMA_CHANNEL_UART0RX, UDMA_ATTR_ALL);
    uDMAChannelControlSet(UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);
}

// ---------------------------------------------
// Receber o vetor via uDMA
// ---------------------------------------------


void Receive_Vector(void)
{


    uDMAChannelTransferSet(UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC,
                           (void *)(UART0_BASE + UART_O_DR),
                           signal,
                           SIGNAL_SIZE * sizeof(float));

    uDMAChannelEnable(UDMA_CHANNEL_UART0RX);

    while(uDMAChannelIsEnabled(UDMA_CHANNEL_UART0RX));

}

void Reset_Vector(void)
{
    memset(signal, 0, sizeof(signal));
}

// ---------------------------------------------
// Main
// ---------------------------------------------
int main(void)
{

    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    ConfigureUART();

    ConfigureUDMA();

    while(1)
    {
        Receive_Vector();
    }
}

