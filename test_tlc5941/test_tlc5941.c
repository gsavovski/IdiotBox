//*****************************************************************************
//
// test_tlc5941.c - Test application to setup and try the TLC5941 PWM LED
//                  driver chip.
//
// Copyright (c) 2013 Idiotronics Incorporated.  All rights reserved.
// 
// This is part of revision 1.0 of the Idiotbox Firmware Package.
//
//*****************************************************************************

//*****************************************************************************
//
// This program will set up the TLC5941 PWM LED driver chip. The chip is set
// up with SPI port writes to set the Dot Correction values and the Grayscale
// values. Then the grayscale clock will be set up. A Grayscale PWM cycle is
// 4096 clocks. Every 4096 clocks an interrupt will toggle the blank line
// which will start the Grayscale cycle over again.
//
// This program uses the following peripherals and I/O signals:
// - SSI1 peripheral
// - SSI1Clk - PD0
// - SSI1Tx  - PD3
// - TIMER0 peripheral
// - T0CCP1  - PF1
// - GPIO port A
// - MODE    - PA5
// - XLAT    - PA6
// - BLANK   - PA7
// - TIMER1 peripheral (interrupt)
//
// The following UART signals are configured only for displaying console
// messages for this program.
// - UART0 peripheral
// - GPIO Port A peripheral (for UART0 pins)
// - UART0RX - PA0
// - UART0TX - PA1
//
// This program uses the following interrupt handlers:
// - Timer1IntHandler
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "utils/uartstdio.h"

//#define MODEL1 1
#define MODEL2 2

// Model 1 vs Model 2 definitions

#ifdef MODEL1
#define TIMER_PWM_BASE WTIMER3_BASE
#define GPIO_TIMER_PWM GPIO_PD2_WT3CCP0
#define GPIO_PORT_TIMER_PWM_BASE GPIO_PORTD_BASE
#define GPIO_PIN_TIMER_PWM GPIO_PIN_2
#define SYSCTL_PERIPH_TIMER_PWM SYSCTL_PERIPH_WTIMER3
#define INT_TIMER_PWM INT_WTIMER3A
#define SSI_GS_BASE SSI1_BASE
#define SYSCTL_PERIPH_SSI_GS SYSCTL_PERIPH_SSI1
#define GPIO_SSICLK_GS GPIO_PD0_SSI1CLK
#define GPIO_SSITX_GS  GPIO_PD3_SSI1TX
#define GPIO_PORT_GS_BASE GPIO_PORTD_BASE
#define GPIO_PIN_SSICLK_GS GPIO_PIN_0
#define GPIO_PIN_SSITX_GS GPIO_PIN_3
#define GPIO_PORT_LEDROW_LO_BASE GPIO_PORTB_BASE
#define GPIO_PORT_LEDROW_HI_BASE GPIO_PORTB_BASE
#define TIMER_GSCLK TIMER_B
#define TIMER_CFG_GSCLK TIMER_CFG_B_PWM
#define GPIO_TIMER_GSCLK GPIO_PF1_T0CCP1
#define GPIO_PIN_GSCLK GPIO_PIN_1
#define GPIO_PORT_KBLO_BASE GPIO_PORTC_BASE
#define GPIO_PIN_KB0 GPIO_PIN_4
#define GPIO_PIN_KB1 GPIO_PIN_5
#define GPIO_PORT_KBHI_BASE GPIO_PORTD_BASE
#define GPIO_PIN_KB2 GPIO_PIN_6
#define GPIO_PIN_KB3 GPIO_PIN_7
#define KB_ROW_SHIFT 0
#define KB_COL_SHIFT 2
#define GPIO_PIN_MODE GPIO_PIN_5
#define GPIO_PIN_XLAT GPIO_PIN_6
#define GPIO_PIN_BLANK GPIO_PIN_7
#define REDIDX(colIdx) (colIdx+4)
#define GREENIDX(colIdx) (colIdx+20)
#define BLUEIDX(colIdx) (colIdx+12+3*(colIdx&4))

// Table to convert LED row value to GPIO pin number
static uint32_t ledRowPin[8] = {
   GPIO_PIN_2,
   GPIO_PIN_3,
   GPIO_PIN_5,
   GPIO_PIN_0,
   GPIO_PIN_1,
   GPIO_PIN_4,
   GPIO_PIN_7,
   GPIO_PIN_6
};

#endif

#ifdef MODEL2
#define GPIO_TIMER_PWM GPIO_PB4_T1CCP0
#define GPIO_PORT_TIMER_PWM_BASE GPIO_PORTB_BASE
#define GPIO_PIN_TIMER_PWM GPIO_PIN_4
#define SYSCTL_PERIPH_TIMER_PWM SYSCTL_PERIPH_TIMER1
#define TIMER_PWM_BASE TIMER1_BASE
#define INT_TIMER_PWM INT_TIMER1A
#define SSI_GS_BASE SSI0_BASE
#define SYSCTL_PERIPH_SSI_GS SYSCTL_PERIPH_SSI0
#define GPIO_SSICLK_GS GPIO_PA2_SSI0CLK
#define GPIO_SSITX_GS  GPIO_PA5_SSI0TX
#define GPIO_PORT_GS_BASE GPIO_PORTA_BASE
#define GPIO_PIN_SSICLK_GS GPIO_PIN_2
#define GPIO_PIN_SSITX_GS GPIO_PIN_5
#define GPIO_PORT_LEDROW_LO_BASE GPIO_PORTD_BASE
#define GPIO_PORT_LEDROW_HI_BASE GPIO_PORTC_BASE
#define TIMER_GSCLK TIMER_B
#define TIMER_CFG_GSCLK TIMER_CFG_B_PWM
#define GPIO_TIMER_GSCLK GPIO_PF1_T0CCP1
#define GPIO_PIN_GSCLK GPIO_PIN_1
// 131216: Couldn't get PF0 working as GSCLK
//#define TIMER_GSCLK TIMER_A
//#define TIMER_CFG_GSCLK TIMER_CFG_A_PWM
//#define GPIO_TIMER_GSCLK GPIO_PF0_T0CCP0
//#define GPIO_PIN_GSCLK GPIO_PIN_0
#define GPIO_PORT_KBLO_BASE GPIO_PORTB_BASE
#define GPIO_PIN_KB0 GPIO_PIN_0
#define GPIO_PIN_KB1 GPIO_PIN_1
#define GPIO_PORT_KBHI_BASE GPIO_PORTB_BASE
#define GPIO_PIN_KB2 GPIO_PIN_2
#define GPIO_PIN_KB3 GPIO_PIN_3
#define KB_ROW_SHIFT 2
#define KB_COL_SHIFT 0
#define GPIO_PIN_MODE GPIO_PIN_4
#define GPIO_PIN_XLAT GPIO_PIN_6
#define GPIO_PIN_BLANK GPIO_PIN_7
#define REDIDX(colIdx) (31-colIdx)
#define GREENIDX(colIdx) (colIdx+4)
#define BLUEIDX(colIdx) (23-colIdx)

// Table to convert LED row value to GPIO pin number
static uint32_t ledRowPin[8] = {
   GPIO_PIN_7,
   GPIO_PIN_6,
   GPIO_PIN_5,
   GPIO_PIN_4,
   GPIO_PIN_0,
   GPIO_PIN_1,
   GPIO_PIN_2,
   GPIO_PIN_3
};

#endif

// Bit map to turn off all pins on a port
#define GPIO_PIN_ALL (0xFF)

//*****************************************************************************
//
// The number of 40MHz clock ticks in a grayscale cycle. 80000 sets the
// cycle time to 2ms.
//
//*****************************************************************************
// Display one matrix row every 2ms. Whole display will be swept every 16ms.
#define GRAYSCALE_CYCLE 80000

//*****************************************************************************
//
// The number of 40MHz clock ticks in a PWMDAC period. 2500 sets the
// cycle time to 0.05ms or sampling frequency of 16kHz.
//
//*****************************************************************************
#define PWMDAC_PERIOD 2500

//*****************************************************************************
//
// Globals
//
//*****************************************************************************

// font table for ASCII values 32 to 127.
extern char font8x8_basic[128-32][8];

// Sine wave approximation function
extern int32_t SineApprox(uint16_t phase, uint16_t scale);

// Bit indicating an new grayscale cycle should begin
volatile uint32_t g_NewGSCycle;

// Pointer to PWMDAC waveform table
volatile uint32_t g_pwmDacValues[2*32];

// The PWM chip allows for intensity values at a resolution of 1/4096 full scale.
// Full scale is 4096.
// This array holds values for the 8 RED LEDs followed by the 8 GREEN LEDs followed
// by the 8 BLUE LEDs.
volatile uint32_t g_gsValues[2*32];
volatile uint8_t g_count16kHz;

// LED row cyles from 0 to 7 as each LED row is refreshed
volatile uint8_t g_ledRow[2];

// Keyboard Row to scan
uint8_t g_kbRow;

//*****************************************************************************
//
// Data buffers
//
//*****************************************************************************

//
// Bit maps for big heart and little heart
//
//static char heartMap[2][8] = {
//  { 0x00, 0x36, 0x49, 0x41, 0x41, 0x22, 0x14, 0x08},   // Big heart
//  { 0x00, 0x00, 0x36, 0x2A, 0x22, 0x14, 0x08, 0x00}    // Little heart
//};

//
// Bit map for a circle
//
//static char circle[1][8] = {
//  { 0x3C, 0x42, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3C}   // circle
//};

//
// Pixel index table for tracing a circle
//
#define CIRCLE_PATT_LEN 20
static char circlePatt[CIRCLE_PATT_LEN] = {
   2,  3,  4,  5, 14, 23, 31, 39, 47, 54,
  61, 60, 59, 58, 49, 40, 32, 24, 16,  9
};


//
// Array of RBG colors
//
#define NUM_COLORS 4
static uint32_t colors[NUM_COLORS] = {
  0x0000FF,  // red
  0x00FF00,  // Green
  0xFF0000,  // Blue
  0x2480F0 // yellow
};

//
// Table to map keypad codes to ASCII charaters
//
//static char keyCodeMap[16] = {
//  '1', '2', '3', 'A',
//  '4', '5', '6', 'B',
//  '7', '8', '9', 'C',
//  '*', '0', '#', 'D'
//};

//
// Table to get different trace frequencies for pattern tracing
//
static uint32_t traceFreqMap[16] = {
  80, 66, 54, 44,
  36, 29, 24, 20,
  16, 13, 11, 9,
   7,  6, 5,  4 // 4 is really too fast. 6 is OK
};

//
// Table to get different tone frequencies for the speaker
// This is the phase increment every 16000Hz sample. Phase is 16 bits.
// Frequency runs from 366Hz to 4883Hz.
//
static uint32_t toneFreqMap[16] = {
  1500, 1783, 2119, 2518,
  2993, 3557, 4227, 5024,
  5791, 7097, 8434, 10024,
  11914, 14159, 16828, 20000
};


//*****************************************************************************
//
// PWMIntHandler
// Inputs: None
// Outputs: None
// Description:
// The interrupt handler for PWM DAC interrupt.
// This is a 16kHz interrupt for playing out DAC samples.
// It is also used to clock out SSI data to the TLC5941 1 word per interrupt.
//
//*****************************************************************************
void
PWMIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER_PWM_BASE, TIMER_CAPA_EVENT);

    // Increment index into 64-entry double-buffered values
    g_count16kHz = 0x3F & (g_count16kHz + 1);

    //
    // Write a new match event value from the table.
    //
    TimerMatchSet(TIMER_PWM_BASE, TIMER_A, g_pwmDacValues[g_count16kHz]);

    if ((g_count16kHz & 0x1F) == 0)
    {
      //
      // Every 32nd interrupt is the end of a grayscale cycle. Set BLANK output.
      // Pulse XLAT to clock in grayscale data and row driver data.
      // Clear BLANK output to start next grayscale cycle.
      //
      // BLANK high
      ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
      // Delay about 3 clocks
      SysCtlDelay(1);
      // XLAT high
      ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
      // Delay about 3 clocks
      SysCtlDelay(1);
      // XLAT low
      ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
      // Delay about 3 clocks
      SysCtlDelay(1);
      // BLANK low
      ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
      // Enable LED row for this grayscale cycle
      ROM_GPIOPinWrite(GPIO_PORT_LEDROW_LO_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, ledRowPin[g_ledRow[g_count16kHz >> 5]]);
      ROM_GPIOPinWrite(GPIO_PORT_LEDROW_HI_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, ledRowPin[g_ledRow[g_count16kHz >> 5]]);
      g_NewGSCycle = 1;
    }

    // Output grayscale values to the SSI port
    HWREG(SSI_GS_BASE + SSI_O_DR) = g_gsValues[g_count16kHz];

}

//*****************************************************************************
//
// ConfigureSSI
// Inputs: None
// Outputs: None
// Description:
// Configure the SSI port and its pins.
//
//*****************************************************************************
void
ConfigureSSI(void)
{
    //
    // The SSI1 peripheral must be enabled for use.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI_GS);

    //
    //
    // Configure the pin muxing for SSI1 functions on port D0 and D3.
    //
    ROM_GPIOPinConfigure(GPIO_SSICLK_GS);
    ROM_GPIOPinConfigure(GPIO_SSITX_GS);

    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware. Only clock and data out are used.
    // The pins are assigned as follows:
    //         Model 1  Model 2
    // SSI1CLK PD0      PA2
    // SSI1Tx  PD3      PA5
    //
    ROM_GPIOPinTypeSSI(GPIO_PORT_GS_BASE, GPIO_PIN_SSICLK_GS | GPIO_PIN_SSITX_GS);

    //
    // Configure and enable the SSI port for SPI master mode.  Use SSI1,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 12-bit data.
    //
    ROM_SSIConfigSetExpClk(SSI_GS_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, 1000000, 12);

    //
    // Enable the SSI1 module.
    //
    ROM_SSIEnable(SSI_GS_BASE);

}

//*****************************************************************************
//
// ConfigureRowDriver
// Inputs: None
// Outputs: None
// Description:
// Configure output pins for the LED row driver.
//
//*****************************************************************************
void
ConfigureRowDriver(void)
{
    //
    // Enable the GPIO pins for LED row driver.
    // Enable the ~CLR pin (PF4)
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORT_LEDROW_LO_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORT_LEDROW_HI_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    // Turn all rows off
    ROM_GPIOPinWrite(GPIO_PORT_LEDROW_LO_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    ROM_GPIOPinWrite(GPIO_PORT_LEDROW_HI_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0);

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);

    // Keep LED row driver off
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);

}

//*****************************************************************************
//
// ConfigureUART
// Inputs: None
// Outputs: None
// Description:
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// ConfigureGSCLK
// Inputs: None
// Outputs: None
// Description:
// Configure TIMER0 to output grayscale clock
//
//*****************************************************************************
void
ConfigureGSCLK(void)
{
    uint32_t regValue;

    //
    // Set PF2 and PF3 to inputs. This will allow the GREEN LED
    // and BLUE LED to be usedfor debug.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //
    // Enable the GPIO pin for GSCLK.
    //
    ROM_GPIOPinConfigure(GPIO_TIMER_GSCLK);
    ROM_GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_GSCLK);

    //
    // Enable Timer0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // Set the global timer configuration for 16-bit split pair.
    //
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_GSCLK);

    //
    // Configure TIMER0B for PWM operation, period = GRAYSCALE_CYCLE / 4096
    //
    regValue = GRAYSCALE_CYCLE / 4096;
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_GSCLK, regValue);
    ROM_TimerPrescaleSet(TIMER0_BASE, TIMER_GSCLK, 0);
    //
    // Configure TIMER0B for  50% duty
    //
    regValue = GRAYSCALE_CYCLE / 8192;
    TimerMatchSet(TIMER0_BASE, TIMER_GSCLK, regValue);
    TimerPrescaleMatchSet(TIMER0_BASE, TIMER_GSCLK, 0);
    //
    // Enable the timer.
    //
    ROM_TimerEnable(TIMER0_BASE, TIMER_GSCLK);
}

//*****************************************************************************
//
// ConfigurePWMDAC
// Inputs: None
// Outputs: None
// Description:
// Configure PWM DAC using Wide Timer3, WT3CCP0 on pin PD2
//
//*****************************************************************************
void
ConfigurePWMDAC(void)
{
    //
    // Enable the GPIO pin for the PWM timer.
    //
    ROM_GPIOPinConfigure(GPIO_TIMER_PWM);
    ROM_GPIOPinTypeTimer(GPIO_PORT_TIMER_PWM_BASE, GPIO_PIN_TIMER_PWM);

    //
    // Enable PWM Timer
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER_PWM);

    //
    // Set the global timer configuration for 16-bit split pair.
    //
    ROM_TimerConfigure(TIMER_PWM_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);

    //
    // Configure PWM timer for PWM operation with a 16kHz frequency
    //
    ROM_TimerLoadSet(TIMER_PWM_BASE, TIMER_A, PWMDAC_PERIOD);
    ROM_TimerPrescaleSet(TIMER_PWM_BASE, TIMER_A, 0);

    //
    // Configure  duty cycle for 1st entry in waveform table
    //
    TimerMatchSet(TIMER_PWM_BASE, TIMER_A, PWMDAC_PERIOD/2);
    TimerPrescaleMatchSet(TIMER_PWM_BASE, TIMER_A, 0);
    //
    // Configure PWM timer to interrupt on negative edge of PWM signal
    //
    TimerControlEvent(TIMER_PWM_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    IntEnable(INT_TIMER_PWM);
    ROM_TimerIntEnable(TIMER_PWM_BASE, TIMER_CAPA_EVENT);

    //
    // Enable the timer.
    //
    ROM_TimerEnable(TIMER_PWM_BASE, TIMER_A);
}

//*****************************************************************************
//
// ConfigureKeybdScan
// Inputs: None
// Outputs: None
// Description:
// Setup pins used to scan the keypad.
//
//*****************************************************************************
void
ConfigureKeybdScan(void)
{
    //
    // Unlock commit bit for PD7 for model 1 board
    //
#ifdef MODEL1
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
#endif

    //
    // Keyboard scan row pins to GPIO outputs
    //
    GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_DIR_MODE_OUT);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    //
    // Keyboard scan column pins to GPIO inputs, weak pulldown
    //
    GPIODirModeSet(GPIO_PORT_KBLO_BASE, GPIO_PIN_KB0 | GPIO_PIN_KB1, GPIO_DIR_MODE_IN);
    GPIODirModeSet(GPIO_PORT_KBHI_BASE, GPIO_PIN_KB2 | GPIO_PIN_KB3, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORT_KBLO_BASE, GPIO_PIN_KB0 | GPIO_PIN_KB1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOPadConfigSet(GPIO_PORT_KBHI_BASE, GPIO_PIN_KB2 | GPIO_PIN_KB3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
}

//*****************************************************************************
//
// WriteDotCorrection
// Inputs: None
// Outputs: None
// Description:
// Write data to the PWM LED driver chips to which sets an intensity for
// each LED row and each color. For now all intensity values are the same.
// This only needs to be done on startup.
//
//*****************************************************************************
void
WriteDotCorrection(void)
{
    uint32_t ticks;

    //
    // Set mode high for Dot Correction write
    //
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_MODE, GPIO_PIN_MODE);
    // Delay about 3 clocks
    SysCtlDelay(1);

    //
    // Each of the 16 outputs can have its LED on current set separately.
    // This is done by writing 16 6-bit words, MSB first. Since the serial
    // port is set for 12-bit data, these values are output 2 at a time.
    // A 6-bit word, N, sets the output current to Imax * N/63.
    // We only use 12 outputs per chip to distribute power dissipation.
    // Set current to 1/8 max by writing 8 packed words of 0x208.
    //

    // Write values for 2nd TLC5941 in daisy-chain
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    // Write values for 1st TLC5941 in daisy-chain
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);
    ROM_SSIDataPut(SSI_GS_BASE, 0x208);

    //
    // Wait until SSI1 is done transferring all the data in the transmit FIFO.
    //
    while(ROM_SSIBusy(SSI_GS_BASE))
    {
    }
    // Wait 10us to make sure serial data transfer is done
    ticks = (SysCtlClockGet() / 3) / 100000;
    SysCtlDelay(ticks);

    //
    // Pulse XLAT high, then low. Minimum high is 20ns. With max of 80MHz clock
    // that's only 2 clock cycles high. Setup from SCLK low to XLAT high is only 10ns.
    //

    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_XLAT, GPIO_PIN_XLAT);
    // Delay about 3 clocks
    SysCtlDelay(1);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_XLAT, 0);

}
      
//*****************************************************************************
//
// RenderCharacter
// Inputs:
//   1. Code for charater to output
//   2. FONT table for 8x8 pixel characters
//   3. Current LED row being scanned
//   4. Color to output
//   5. Pointer to grayscale code output buffer
// Outputs: None
// Description:
// Given a character, a 8x8 matrix row, a 8x8 font table and RGB values,
// construct grayscale values for the red, green and blue LEDs in given row.
//
//*****************************************************************************
#define ABC_PATT_LEN 26

void
RenderCharacter(char charAddr, char fontTable[128-32][8], uint8_t row, uint32_t color, \
                uint32_t* gsValues)
{

  char bitmap;
  uint32_t colIdx;

  bitmap = fontTable[charAddr][row];
  for (colIdx = 0; colIdx < 8; colIdx++)
  {
    // RED grayscale values
    gsValues[REDIDX(colIdx)] = (bitmap & (1 << colIdx)) ? ((color&0xFF) << 4) : 0;
    // GREEN grayscale values
    gsValues[GREENIDX(colIdx)] = (bitmap & (1 << colIdx)) ? ((color&0xFF00) >> 4) : 0;
    // BLUE grayscale values
    gsValues[BLUEIDX(colIdx)] = (bitmap & (1 << colIdx)) ? ((color&0xFF0000) >> 12) : 0;
  }
}

//*****************************************************************************
//
// RenderPixel
// Inputs:
//   1. Index to a pixel coordinate array
//   2. Current LED row being scanned
//   3. Color to output
//   4. Pointer to grayscale code output buffer
// Outputs: None
// Description:
// Get the pixel coordinate from the array. Turn on the pixel with the given
// color if that pixel is in the current LED row being refreshed.
//
//*****************************************************************************
void
RenderPixel(uint32_t pixelIdx, uint32_t row, uint32_t color, uint32_t* gsValues)
{
  // Turn on the one pixel indicated by pixelIdx

  uint32_t colIdx;
  uint32_t match;

  for (colIdx = 0; colIdx < 8; colIdx++)
  {
    match = ((pixelIdx >> 3) & 0x7) == row && (pixelIdx & 0x7) == colIdx;
    // Turn on RED if match
    gsValues[REDIDX(colIdx)] = match ? ((color&0xFF) << 4) : 0;
    // Turn on GREEN if match
    gsValues[GREENIDX(colIdx)] = match ? ((color&0xFF00) >> 4) : 0;
    // Turn on BLUE if match
    gsValues[BLUEIDX(colIdx)] = match ? ((color&0xFF0000) >> 12) : 0;
  }
}

//*****************************************************************************
//
// RenderDomino
// Inputs:
//   1. Index to the domino pattern
//   2. Pointer to grayscale code output buffer
// Outputs: None
// Description:
// Turn on pixels from beginning to end in each row one at a time until all
// pixels are on. Then turn them off from end to beginning. Repeat with a
// different color. pattIdx determines what to turn on or off and what color
// to use.
//
//*****************************************************************************
#define DOMINO_PATT_LEN 48
void
RenderDomino(uint32_t pattIdx, uint32_t* gsValues)
{
  uint32_t colorIdx;
  uint32_t colIdx;

  colorIdx = pattIdx >> 4;
  if (pattIdx & 0x8)
  {
    // Turn off next column
    colIdx = 7 - (pattIdx & 0x7);
    gsValues[REDIDX(colIdx)] = 0;
    gsValues[GREENIDX(colIdx)] = 0;
    gsValues[BLUEIDX(colIdx)] = 0;
  }
  else
  {
    // Turn on next column
    colIdx = pattIdx & 0x7;
    gsValues[REDIDX(colIdx)] = (colors[colorIdx]&0xFF) << 4;
    gsValues[GREENIDX(colIdx)] = (colors[colorIdx]&0xFF00) >> 4;
    gsValues[BLUEIDX(colIdx)] = (colors[colorIdx]&0xFF0000) >> 12;
  }
}

//*****************************************************************************
//
// KeybdRead
// Inputs: None
// Outputs: Code indicating if key has been pressed and which one
// Description: Set an output to scan one keypad row. Read in 4 keypad columns
// to see if a key in that row was pressed. Output keyCode if key is pressed,
// otherwise output 0.
// keyCode is 0x10 | column<<2 | row. 0x10 indicates that some key was pressed.
//
//*****************************************************************************
uint32_t
KeybdRead(void)
{
  uint32_t keyCode;
  uint8_t keyRow, keyCol;
  
#ifdef MODEL1
  keyRow = 0x3 & (g_ledRow[g_count16kHz >> 5] - 2);
#endif
#ifdef MODEL2
  keyRow = g_kbRow;
  ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, (1 << keyRow));
  g_kbRow = 0x3 & (g_kbRow + 1);
#endif

  //
  // If keyRow < 4, check for keyboard input
  //
  keyCode = 0;
  if (keyRow < 4)
  {
    // Bits 0 and 1 of keyCode is ledRow-2
    keyCode = keyRow << KB_ROW_SHIFT;

    // Read inputs coming from column of keypad
    keyCol = ROM_GPIOPinRead(GPIO_PORT_KBLO_BASE, GPIO_PIN_KB0 | GPIO_PIN_KB1);
    keyCol |= ROM_GPIOPinRead(GPIO_PORT_KBHI_BASE, GPIO_PIN_KB2 | GPIO_PIN_KB3);
    // Do priority encode to set keyCode bits 2 and 3
    if (keyCol & GPIO_PIN_KB3)
    {
      keyCode |= 3 << KB_COL_SHIFT;
    }
    else if (keyCol & GPIO_PIN_KB2)
    {
      keyCode |= 2 << KB_COL_SHIFT;
    }
    else if (keyCol & GPIO_PIN_KB1)
    {
      keyCode |= 1 << KB_COL_SHIFT;
    }
    // Set bit 4 if any key at all was pressed.
    if (keyCol)
    {
      keyCode |= 16;
    }
  }
  return(keyCode);
}

//*****************************************************************************
//
// main
//
//*****************************************************************************
int
main(void)
{
  uint32_t idx;
  uint32_t idleCount;
  uint8_t displayChar;
  uint32_t gsCycleCount;
  uint8_t keyCode, keyPressed, keyValue;
  uint32_t debounceCount;
  uint32_t pixelIdx, pattIdx;
  uint32_t freqIdx;
  uint8_t ledRow;
  uint8_t gsIdx;
  uint32_t* gsPtr;
  // Current phase of sinewave, 0 to 65535
  uint16_t sinePhase;
  // Frequency of sine wave = phase increment every 16kHz sample
  uint16_t sineFreq;
  uint8_t function;


    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the system clock to run at 40Mhz off PLL with external crystal as
    // reference.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    ROM_IntMasterDisable();

    /*********************
     * ENABLE GPIO PORTS *
     *********************/
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Set up the serial console to use for displaying messages.
    //
    ConfigureUART();

    //
    // Enable the GPIO pins for TLC5941 control (PA5 - PA7).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_MODE | GPIO_PIN_XLAT | GPIO_PIN_BLANK);

    //
    // Set MODE high, set XLAT low and set BLANK high).
    //
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_MODE, GPIO_PIN_MODE);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_XLAT, 0);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_BLANK, GPIO_PIN_BLANK);

    UARTprintf("TLC5941 controls configured\n");

    /************************
     * SETUP LED ROW DRIVER *
     ************************/

    ConfigureRowDriver();
    UARTprintf("Row driver configured\n");

    /*******************************************
     * SETUP SPI PORT TO WRITE DATA TO TLC5941 *
     *******************************************/

    ConfigureSSI();
    UARTprintf("SSI configured\n");

    /*******************************************
     * SETUP Keyboard row pins and outputs and *
     * column pins as inputs                   *
     *******************************************/

    ConfigureKeybdScan();
    UARTprintf("Keyboard scan configured\n");

    /****************************************
     * WRITE DOT CORRECTION DATA TO TLC5941 *
     ****************************************/

    WriteDotCorrection();
    UARTprintf("Dot correction data written\n");

    /*****************************
     * WRITE PWM DATA TO TLC5941 *
     *****************************/

    // Initialize bit indicating new grayscale cycle
    g_NewGSCycle = 0;
    for (idx = 0; idx < 2*32; idx++)
    {
      g_gsValues[idx] = 0;
    }
    //
    // Set mode low for PWM write
    //
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_MODE, 0);

    // Enable LED row driver (only for model 1)
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
    ledRow = 0;
    g_ledRow[0] = 0;
    g_ledRow[1] = 0;
    g_count16kHz = 0;

    // Clear keyboard row selector
    g_kbRow = 0;
    keyPressed = 0;

    /*************************
     * SETUP GRAYSCALE CLOCK *
     *************************/

    ConfigureGSCLK();
    UARTprintf("Grayscale clock configured\n");

    /*****************************
     * SETUP PWM DAC for speaker *
     *****************************/

    ConfigurePWMDAC();
    UARTprintf("PWM DAC configured\n");

    //
    // Load 1 kHz sine wave into PWM DAC buffer
    //
    sinePhase = 0;
    sineFreq = toneFreqMap[0];
    for (idx = 0; idx < 64; idx++)
    {
      g_pwmDacValues[idx] = 0.5*PWMDAC_PERIOD + SineApprox(sinePhase, 0.45*PWMDAC_PERIOD);
      sinePhase += sineFreq;
    }
    UARTprintf("PWM DAC configured\n");

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Loop while the grayscale cycle interrupt toggles blank to start
    // a new grayscale cycle.
    //
    idleCount = 0;
    //displayChar = 32;

    freqIdx = 0;
    gsCycleCount = traceFreqMap[freqIdx];
    pattIdx = 0;
    pixelIdx = circlePatt[pattIdx];
    function = 0;
    while(1)
    {
      if (g_NewGSCycle != 0)
      {
        //
        // Set next LED row
        //
        //ledRow = 0x7 & (ledRow + 1);
        ledRow = 0x7 & (g_ledRow[g_count16kHz >> 5] + 1);
        g_ledRow[(g_count16kHz >> 5) ^ 1] = ledRow;
        //
        // Set next Grayscale and PWM DAC buffer half
        //
        gsIdx = (g_count16kHz & 0x20) ^ 0x20;
        gsPtr = (uint32_t*)&g_gsValues[gsIdx];

        //
        // Read the keypad and debounce
        //
        keyCode = KeybdRead();
        if (keyCode & 0x10)
        {
          debounceCount = 50;
          keyPressed = 1;
          keyValue = keyCode & 0xF;
        }
        if (debounceCount)
        {
          debounceCount--;
          if (debounceCount == 0)
          {
            keyPressed = 0;
            function = 1 ^ function;
          }
        }

        if (keyPressed)
        {
          //
          // If Keypad pressed perform display and sound function
          //

          // Select new display character
          //displayChar = keyCodeMap[keyCode & 0xF];

          // Select new pattern-trace frequency.
          freqIdx = keyValue;

          // Select new tone frequency
          sineFreq = toneFreqMap[freqIdx];

          //RenderPixel(pixelIdx, ledRow, colors[0], gsPtr);
          //RenderCharacter( 0, circle, g_ledRow[ledRow], colors[0], &g_gsValues[g_gsIdx]);
          //RenderCharacter((gsCycleCount&0x100) >> 8, HeartMap, g_ledRow[ledRow], colors[1], &g_gsValues[g_gsIdx]);
          if (function == 0)
          {
            displayChar = pattIdx + 'A';
            RenderCharacter(displayChar - ' ', font8x8_basic, ledRow, colors[0], gsPtr);
          }
          else
          {
            RenderDomino(pattIdx, gsPtr);
          }

          //
          // Fill half of PWM DAC buffer with sine wave values.
          //
          for (idx = 0; idx < 32; idx++)
          {
            g_pwmDacValues[gsIdx + idx] = 0.5*PWMDAC_PERIOD + SineApprox(sinePhase, 0.45*PWMDAC_PERIOD);
            sinePhase += sineFreq;
          }

          //
          // Update the grayscale cycle count
          //
          if (gsCycleCount)
          {
            gsCycleCount--;
          }

          if (gsCycleCount == 0)
          {
            //pattIdx = (pattIdx + 1 >= CIRCLE_PATT_LEN) ? 0 : pattIdx + 1;
            //pixelIdx = circlePatt[pattIdx];
            if (function == 0)
            {
              pattIdx = (pattIdx + 1 >= ABC_PATT_LEN) ? 0 : pattIdx + 1;
              gsCycleCount = 8*traceFreqMap[freqIdx];
            }
            else
            {
              pattIdx = (pattIdx + 1 >= DOMINO_PATT_LEN) ? 0 : pattIdx + 1;
              gsCycleCount = traceFreqMap[freqIdx];
            }
          }

        }

        else
        {
          //
          // If keypad press expired output 0s to Grayscale value and PWM DAC
          //
          for (idx = 0; idx < 32; idx++)
          {
            gsPtr[idx] = 0;
            g_pwmDacValues[gsIdx + idx] = 0;
          }
          pattIdx = 0;
        }

        g_NewGSCycle = 0;
        idleCount = 0;
      }
      else
      {
        idleCount++;
      }
    }

}
