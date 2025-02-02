
/* 
 * Input Compare example code for PIC32MK1024GPK064
 * 
 * 12MHz Crystal on Primary Oscillator
 * 32.768KHz Crystal on Secondary Oscillator
 * 
 * UART:
 *  TX = RG9/AN16
 *  RX = RG7/AN18
 * 
 * RB2 = Input Compare Input Pin
 */

#include <stdio.h>
#include <sys/attribs.h>
#include <xc.h>
#include "main.h"

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration bit (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO1 = ON           // USB1 USBID Selection (USBID pin is controlled by the USB1 module)
#pragma config FVBUSIO1 = ON            // USB2 VBUSON Selection bit (VBUSON pin is controlled by the USB1 module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_8         // System PLL Input Divider (8x Divider)
#pragma config FPLLRNG = RANGE_54_64_MHZ// System PLL Input Range (54-64 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_128       // System PLL Multiplier (PLL Multiply by 128)
#pragma config FPLLODIV = DIV_32        // System PLL Output Clock Divider (32x Divider)
#pragma config BORSEL = HIGH            // Brown-out trip voltage (BOR trip voltage 2.1v (Non-OPAMP deviced operation))
#pragma config UPLLEN = OFF             // USB PLL Enable (USB PLL Disabled)

// DEVCFG1
#pragma config FNOSC = POSC             // Oscillator Selection Bits (Primary Osc (HS,EC))
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enable Secondary Oscillator)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = ON              // Deadman Timer Enable (Deadman Timer is enabled)

// DEVCFG0
#pragma config DEBUG = ON               // Background Debugger Enable (Debugger is enabled)
#pragma config JTAGEN = ON              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config TRCEN = ON               // Trace Enable (Trace features in the CPU are enabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = ECC_DECC_DISABLE_ECCON_WRITABLE// Dynamic Flash ECC Configuration Bits (ECC and Dynamic ECC are disabled (ECCCON<1:0> bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = G3            // Secondary Oscillator Gain Control bits (Gain is G3)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = G3            // Primary Oscillator Coarse Gain Control bits (Gain Level 3 (highest))
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCFGAIN = G3           // Primary Oscillator Fine Gain Control bits (Gain is G3)
#pragma config POSCAGCDLY = AGCRNG_x_25ms// AGC Gain Search Step Settling Time Control (Settling time = 25ms x AGCRNG)
#pragma config POSCAGCRNG = ONE_X       // AGC Lock Range bit (Range 1x)
#pragma config POSCAGC = Automatic      // Primary Oscillator Gain Control bit (Automatic Gain Control for Oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot Enable (Normal EJTAG functionality)

// DEVCP
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ
#pragma config TSEQ = 0xFFFF            // Boot Flash True Sequence Number (Enter Hexadecimal value)
#pragma config CSEQ = 0xFFFF            // Boot Flash Complement Sequence Number (Enter Hexadecimal value)

#define LED1            LATBbits.LATB13    
#define LED1_DIR        TRISBbits.TRISB13

#define LED2            LATBbits.LATB15
#define LED2_DIR        TRISBbits.TRISB15

void init_OSC(void);
void init_TMR2(void);
void init_UART2(void);

void main(void)
{
    ANSELB = 0x0000;    // All PortB pins are digital
    LED1 = 0;
    LED2 = 0;
    LED1_DIR = 0;
    LED2_DIR = 0;
    
    init_OSC();    
    init_TMR2();
    //init_TMR3();
    init_UART2();
    init_IC1();
    
    // Enable Multi-Vector Interrupts
     INTCONSET = _INTCON_MVEC_MASK;  
    __builtin_enable_interrupts();  
    
    printf("\n\rPIC32MK Input Compare Example\n\r");
    
    while (1) {
        //printf("Hello World\r\n");
    }
}

void init_OSC(void)
{
    // Unlock System for Clock Configuration
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
        
    // Peripheral Clock Default Values:
    // PBCLK 1 to 5 = SYSCLK / 2
    // PBCLK 6      = SYSCLK / 4
    // PBCLK 7      = SYSCLK / 1
    
    // Set PBCLK2 to SYSCLK / 0;
    // Used by TMR 2-9, UART
    PB2DIVbits.PBDIV = 0;
  
    // Lock system since done with clock configuration
    SYSKEY = 0x33333333;
}

void init_TMR2(void)
{
    // Set up Timer 2 as timebase for Input Compare modules
    // Timers 2 to 9 are clocked from Peripheral Bus Clock 2. 
    // PBCLK2 = 12MHz, increment every 83.3uS
    PR2 = 0xFFFFFFFF;
    T2CONbits.TCKPS = 0b000;    // 1:1 Prescaler
    T2CONbits.TCS = 0;          // Use internal peripheral clock as source
    T2CONbits.T32 = 1;          // 32-Bit Timer
    IPC2bits.T2IP = 1;          // Interrupt Priority (Low)
    IFS0bits.T2IF = 0;          // Clear Flag
    IEC0bits.T2IE = 0;          // Enable Interrupt
    T2CONbits.ON = 1;           // Enable Timer
}

void __ISR (_TIMER_2_VECTOR, IPL1AUTO) Timer2Handler (void)
{
    IFS0bits.T2IF = 0;          // Clear Flag
}

void init_TMR3(void)
{
    // Set up Timer 3 to flash LED once every second
    // Timers 2 to 9 are clocked from Peripheral Bus Clock 2. 
    // PBCLK3 = 12MHz / 16 Prescaler = 750,000
    PR3 = 375000 - 16;
    T3CONbits.TCKPS = 4;        // 1:16 Prescaler
    T3CONbits.TCS = 0;          // Use internal peripheral clock as source
    T3CONbits.T32 = 1;          // 32-Bit Timer
    IPC3bits.T3IP = 1;          // Interrupt Priority (Low)
    IFS0bits.T3IF = 0;          // Clear Flag
    IEC0bits.T3IE = 1;          // Enable Interrupt
    T3CONbits.ON = 1;           // Enable Timer
}

void __ISR (_TIMER_3_VECTOR, IPL1AUTO) Timer3Handler (void)
{
    LED1 = ~LED1;
    IFS0bits.T3IF = 0;          // Clear Flag
}

void init_UART2(void)
{
    // Set up for transmit only (printf). 
    // No requirement for receive.
    // Configured for 9600bps, 8N1
    // UART2 is clocked from Peripheral Bus Clock 2. 
    // TX = RG9/RPG9/AN16
    // RX = RG7/RPG7/AN18 (Not currently used)

    __XC_UART = 2;              // Redirect stdout to UART1
    
    U2MODEbits.ON = 0;          // Disable UART
    // Set up remappable pins:
    U2RXRbits.U2RXR = 0b1010;   // Receive on RG8 (Input)
    RPG9Rbits.RPG9R = 2;        // Transmit on RG9 (Output)
    // Set up baud rate:
    U2MODEbits.CLKSEL = 0b00;   // Clock from PBCLK2
    U2MODEbits.BRGH = 0;        // Standard Speed Mode
    U2BRG = 78;                 // UxBRG = ((CLKSEL Frequency / (16 * Desired Baud Rate)) ? 1)   
    // Set up interrupts: (not enabled)
    IFS1bits.U2TXIF = 0;        // Clear the Transmit Interrupt Flag
    IEC1bits.U2TXIE = 0;        // Disable Transmit Interrupts
    IFS1bits.U2RXIF = 0;        // Clear the Receive Interrupt Flag
    IEC1bits.U2RXIE = 0;        // Disable Receive Interrupts
    // Enable only transmit: 
    U2STAbits.UTXEN = 1;        // Enable Transmit
    U2MODEbits.ON = 1;          // Enable UART
}

void init_IC1(void)
{
    IC1Rbits.IC1R = 0b0100;     // Use RPB2 as input
    IC1CONbits.ICM = 0b011;     // Every rising edge
    IC1CONbits.ICI = 0b00;      // Interrupt on every capture event
    IC1CONbits.ICTMR = 0;       // Use TIMERY (TMR2) as timebase
    IC1CONbits.C32 = 1;         // 32 bits
    IPC1bits.IC1IP = 1;         // Low Priority
    IFS0bits.IC1IF = 0;         // Clear Flag
    IEC0bits.IC1IE = 1;         // Enable Interrupts
    IC1CONbits.ON = 1;          // Module Enabled
}

void __ISR (_INPUT_CAPTURE_1_VECTOR, IPL1AUTO) IC1Handler (void)
{
    static uint32_t last_value;
    uint32_t current_value = IC1BUF;
    uint32_t diff = current_value - last_value;
    int32_t error = (uint32_t)diff - 12000000;
    printf("IC2 %d, Error %0.1fppm (%.0fnS)\r\n", diff, ((float)error/12.0), (error * 83.3));
    last_value = current_value;
    IFS0bits.IC1IF = 0;          // Clear Flag
    //IFS0bits.IC1EIF = 0;         // Error Flag    
}