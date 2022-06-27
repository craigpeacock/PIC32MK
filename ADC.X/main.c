
/* 
 * Example code for PIC32MK0512MCJ064
 * ADC Interrupt Example. Using three dedicated ADC modules, scan AN0, 
 * AN1 & AN2 and print results from End of Scan Interrupt.
 * Process triggered by software, once every 500mS from Timer 2 ISR
 * 
 * 8MHz Crystal on Primary Oscillator
 * 32.768KHz Crystal on Secondary Oscillator
 * 
 * UART:
 *  TX = RG9/AN16
 *  RX = RG7/AN18
 */

#include <stdio.h>
#include <sys/attribs.h>
#include <xc.h> 

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config PWMLOCK = OFF            // PWM IOxCON lock (PWM IOxCON register writes accesses are not locked or protected)
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

#define LCD_BL_EN       LATDbits.LATD5
#define LCD_BL_EN_DIR   TRISDbits.TRISD5

void init_OSC(void);
void init_TMR2(void);
void init_UART2(void);
void init_ADC(void);

void print_reset_cause(void)
{
    int temp = RCON;
    printf("RCON 0x%04X ",RCON);
    if (RCON & 0x01) printf("Power-on Reset\r\n");
    if (RCON & 0x02) printf("Brown-out Reset\r\n");
    if (RCON & 0x04) printf("Wake from Idle\r\n");
    if (RCON & 0x08) printf("Wake from Sleep\r\n");
    if (RCON & 0x10) printf("Watchdog Time-out\r\n");
    if (RCON & 0x20) printf("Deadman Time-Out\r\n");
    if (RCON & 0x40) printf("Software Reset\r\n");
    if (RCON & 0x80) printf("MCLR Reset\r\n");
    RCON = 0x00;
}

void main(void)
{
    ANSELB = 0x0000;    // All PortB pins are digital
    LED1 = 0;
    LED2 = 0;
    LED1_DIR = 0;
    LED2_DIR = 0;
    
    // Turn off LCD backlight
    LCD_BL_EN_DIR = 0;  
    LCD_BL_EN = 0;

    init_OSC();    
    init_TMR2();
    init_UART2();
    init_ADC();
    
    // Enable Multi-Vector Interrupts
     INTCONSET = _INTCON_MVEC_MASK;  
    __builtin_enable_interrupts();  
    
    printf("\n\rPIC32MK Example\n\r");
    print_reset_cause();
   
    while (1) {
      
        // If polling, wait for data to be ready and then print to UART.

        //while (ADCDSTAT1bits.ARDY0 == 0);
        //printf("AN0 0x%04X\r\n", ADCDATA0);

        //while (ADCDSTAT1bits.ARDY1 == 0);
        //printf("AN1 0x%04X ", ADCDATA1);
        
        //while (ADCDSTAT1bits.ARDY2 == 0);
        //printf("AN2 0x%04X\n\r", ADCDATA2);
                
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
    // Timers 2 to 9 are clocked from Peripheral Bus Clock 2. 
    // PBCLK2 = 8MHz / 16 Prescaler = 500,000
    PR2 = 250000 - 16;
    // Interrupt every 500mS
    T2CONbits.TCKPS = 4;        // 1:16 Prescaler
    T2CONbits.TCS = 0;          // Use internal peripheral clock as source
    T2CONbits.T32 = 1;          // 32-Bit Timer
    IPC2bits.T2IP = 1;          // Interrupt Priority
    IFS0bits.T2IF = 0;          // Clear Flag
    IEC0bits.T2IE = 1;          // Enable Interrupt
    T2CONbits.ON = 1;           // Enable Timer
}

void __ISR (_TIMER_2_VECTOR, IPL1AUTO) Timer2Handler (void)
{
    // Trigger a scan of three channels:
    ADCCON3bits.GSWTRG = 1;     // Trigger a scan

    // Alternatively, we can select a channel and initiate a conversion:
    //ADCCON3bits.ADINSEL = 0;    // Select AN0
    //ADCCON3bits.RQCNVRT = 1;    // Individual ADC Input Conversion Request bit
    
    LED1 = ~LED1;
    IFS0bits.T2IF = 0;          // Clear Flag
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
    U2BRG = 52;                 // UxBRG = ((CLKSEL Frequency / (16 * Desired Baud Rate)) ? 1)   
    // Set up interrupts: (not enabled)
    IFS1bits.U2TXIF = 0;        // Clear the Transmit Interrupt Flag
    IEC1bits.U2TXIE = 0;        // Disable Transmit Interrupts
    IFS1bits.U2RXIF = 0;        // Clear the Receive Interrupt Flag
    IEC1bits.U2RXIE = 0;        // Disable Receive Interrupts
    // Enable only transmit: 
    U2STAbits.UTXEN = 1;        // Enable Transmit
    U2MODEbits.ON = 1;          // Enable UART
}

void init_ADC(void)
{
    // Enable three (of six) dedicated ADC modules
    //  RA0 = AN_VOLTAGE
    //  RA1 = AN_CURRENT_MA
    //  RA2 = AN_CURRENT_UA
    
    ADCCON1bits.ON = 0;             // Ensure peripheral is off
    
    ADC0CFG = DEVADC0;              // Copy ADC calibration value from factory 
    ADC1CFG = DEVADC1;              // programmed registers
    ADC2CFG = DEVADC2;
    
    // Set up timing and resolution:
    ADC0TIMEbits.SAMC = 100;        // 102 TADX
    ADC0TIMEbits.ADCDIV = 8;        // Clock Divisor 
    ADC0TIMEbits.SELRES = 0b11;     // 12 Bit Resolution
    
    ADC1TIMEbits.SAMC = 100;        // 102 TADX
    ADC1TIMEbits.ADCDIV = 8;        // Clock Divisor 
    ADC1TIMEbits.SELRES = 0b11;     // 12 Bit Resolution
    
    ADC2TIMEbits.SAMC = 100;        // 102 TADX
    ADC2TIMEbits.ADCDIV = 8;        // Clock Divisor 
    ADC2TIMEbits.SELRES = 0b11;     // 12 Bit Resolution
    
    ADCCON1bits.STRGSRC = 1;        // Scan Trigger = Software
    ADCCON3bits.CONCLKDIV = 11;     // ADC Control Clock Divider
    ADCCON3bits.VREFSEL = 1;        // Use VREF+/AVSS as Voltage Ref
   
    ADCCSS1bits.CSS0 = 1;           // Add AN0 to scan
    ADCCSS1bits.CSS1 = 1;           // Add AN1 to scan
    ADCCSS1bits.CSS2 = 1;           // Add AN2 to scan
    
    ADCTRG1bits.TRGSRC0 = 3;        // Trigger Source for AN0 = Scan Trigger 
    ADCTRG1bits.TRGSRC1 = 3;        // Trigger Source for AN1 = Scan Trigger
    ADCTRG1bits.TRGSRC2 = 3;        // Trigger Source for AN2 = Scan Trigger

    ADCIMCON1 = 0x00;               // All inputs Single Ended (Default)
    ADCIMCON2 = 0x00;               // All inputs Single Ended (Default)
    ADCIMCON3 = 0x00;               // All inputs Single Ended (Default)
    ADCIMCON4 = 0x00;               // All inputs Single Ended (Default)
    
    // Turn on ADC
    ADCCON1bits.ON = 1;
    while(!ADCCON2bits.BGVRRDY);    // Wait until the reference voltage is ready
    while(ADCCON2bits.REFFLT);      // Wait if there is a fault with the reference voltage

    // ADC 0
    ADCANCONbits.ANEN0 = 1;         // Enable the clock to analog bias
    while(!ADCANCONbits.WKRDY0);    // Wait until ADC is ready
    ADCCON3bits.DIGEN0 = 1;         // Enable ADC
    //ADCGIRQEN1bits.AGIEN0 = 1;    // Enable Global Interrupt
    IPC26bits.AD1D0IP = 1;          // Priority
    IFS3bits.AD1D0IF = 0;           // Clear Flag
    //IEC3bits.AD1D0IE = 1;         // Enable Interrupt
    
    // ADC 1 
    ADCANCONbits.ANEN1 = 1;         // Enable the clock to analog bias
    while(!ADCANCONbits.WKRDY1);    // Wait until ADC is ready
    ADCCON3bits.DIGEN1 = 1;         // Enable ADC
    //ADCGIRQEN1bits.AGIEN1 = 1;    // Enable Global Interrupt
    IPC26bits.AD1D1IP = 1;          // Priority
    IFS3bits.AD1D1IF = 0;           // Clear Flag
    //IEC3bits.AD1D1IE = 1;         // Enable Interrupt
    
    // ADC 3 
    ADCANCONbits.ANEN2 = 1;         // Enable the clock to analog bias
    while(!ADCANCONbits.WKRDY2);    // Wait until ADC is ready
    ADCCON3bits.DIGEN2 = 1;         // Enable ADC
    //ADCGIRQEN1bits.AGIEN2 = 1;    // Enable Global Interrupt
    IPC27bits.AD1D2IP = 1;          // Priority
    IFS3bits.AD1D2IF = 0;           // Clear Flag
    //IEC3bits.AD1D2IE = 1;         // Enable Interrupt
    
    // End of Scan Interrupt
    ADCCON2bits.EOSIEN = 1;
    IPC25bits.AD1EOSIP = 2;         // Priority
    IFS3bits.AD1EOSIF = 0;          // Clear Flag
    IEC3bits.AD1EOSIE = 1;          // Enable End of Scan Interrupt

    // ADC Global Interrupt
    IPC23bits.AD1IP = 1;            // Priority
    IFS2bits.AD1IF = 0;             // Clear Flag
    //IEC2bits.AD1IE = 1;           // Enable
}

void __ISR (_ADC_EOS_VECTOR, IPL2AUTO) EOS_Handler(void)
{
    // End of Scan Interrupt. Triggered after a scan of all channels is complete.
    printf("AN0:0x%04X AN1:0x%04X AN2:0x%04X\r\n", ADCDATA0, ADCDATA1, ADCDATA2);
    // We need to read ADCCON2 to clear EOSRDY bit, otherwise interrupt keeps
    // occurring
    int temp = ADCCON2;
    IFS3bits.AD1EOSIF = 0;      // Clear Flag
}

void __ISR (_ADC_VECTOR, IPL1AUTO) ADC_Handler(void)
{
    // ADC Global Interrupt. 
    // A single vector that can be used for all ADC interrupt events. 
    printf("ADC Global Interrupt\r\n");
    IFS2bits.AD1IF = 0;         // Clear Flag
}

void __ISR (_ADC_DATA0_VECTOR, IPL1AUTO) AN0_Handler(void)
{
    // AN0 Channel Interrupt. Generally triggered after ADC conversion 
    printf("AN0:0x%04X\r\n", ADCDATA0);
    IFS3bits.AD1D0IF = 0;       // Clear Flag
}

void __ISR (_ADC_DATA1_VECTOR, IPL1AUTO) AN1_Handler(void)
{
    // AN1 Channel Interrupt. Generally triggered after ADC conversion 
    printf("AN1:0x%04X\r\n", ADCDATA1);
    IFS3bits.AD1D1IF = 0;       // Clear Flag
}

void __ISR (_ADC_DATA2_VECTOR, IPL1AUTO) AN2_Handler(void)
{
    // AN2 Channel Interrupt. Generally triggered after ADC conversion 
    printf("AN2:0x%04X\r\n", ADCDATA2);
    IFS3bits.AD1D2IF = 0;       // Clear Flag
}
