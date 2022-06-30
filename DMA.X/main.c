
/* 
 * Example code for PIC32MK0512MCJ064
 * DMA Example. Copies data from BufferA to BufferB on software trigger.
 * 
 * 8MHz Crystal on Primary Oscillator
 * 32.768KHz Crystal on Secondary Oscillator
 * 
 * UART:
 *  TX = RG9/AN16
 *  RX = RG7/AN18
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/attribs.h>
#include <xc.h> 
#include <sys/kmem.h>                   // Included for Address Translation Macros, KVA_TO_PA

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
void init_UART2(void);
void init_DMA(void);
void print_reset_cause(void);

#define MAX_BUFFER 32
uint16_t BufferA[MAX_BUFFER];
uint16_t BufferB[MAX_BUFFER];

void main(void)
{
    int i;
    
    ANSELB = 0x0000;    // All PortB pins are digital
    LED1 = 0;
    LED2 = 0;
    LED1_DIR = 0;
    LED2_DIR = 0;
    
    // Turn off LCD backlight
    LCD_BL_EN_DIR = 0;  
    LCD_BL_EN = 0;

    init_OSC();    
    init_UART2();
    init_DMA();
    
    // Enable Multi-Vector Interrupts
     INTCONSET = _INTCON_MVEC_MASK;  
    __builtin_enable_interrupts();  
    
    printf("\n\rPIC32MK Example\n\r");
    print_reset_cause();
   
    // Fill buffer A with data
    for (i = 0; i < MAX_BUFFER; i++)
        BufferA[i] = rand();
        
    // Print buffer A
    printf("Buffer A:\r\n");
    for (i = 0; i < MAX_BUFFER; i++) {
        printf("%05d ",BufferA[i]);
        if (((i+1) % 8) == 0) printf("\r\n");
    }
    
    // Start DMA Transfer
    DCH0ECONbits.CFORCE = 1;
        
    // Print buffer B
    printf("Buffer B:\r\n");
    for (i = 0; i < MAX_BUFFER; i++) {
        printf("%05d ",BufferB[i]);
        if (((i+1) % 8) == 0) printf("\r\n");
    }
           
    while (1) {
                
    }
}

void init_DMA(void)
{
    DMACONbits.ON = 1;              // Enable DMA Controller
    DCH0CON = 0x03;                 // Channel off, priority 3, no chaining
    DCH0ECON = 0;                   // Clear all start/stop IRQs and pattern match
    
    DCH0SSA = KVA_TO_PA((void *)BufferA);  // Source
    DCH0DSA = KVA_TO_PA((void *)BufferB);  // Destination
    DCH0SSIZ = sizeof(BufferA);     // Source Size (bytes)
    DCH0DSIZ = sizeof(BufferB);     // Destination Size (bytes)
    DCH0CSIZ = sizeof(BufferA);     // Bytes transferred per trigger event
    
    // DMA transfer can be triggered by interrupt source. In this example
    // we trigger in the main loop using a forced trigger (CFORCE). 
    //DCH0ECONbits.CHSIRQ = 106;      // Channel Transfer start IRQ (ADC Data 0)
    //DCH0ECONbits.SIRQEN = 1;        // Start channel cell transfer if an interrupt matching above occurs
    
    DCH0INT = 0;                    // Clear all Interrupts
    DCH0INTbits.CHDDIE = 1;         // Channel Destination Done Interrupt Enable bit  
    IPC18bits.DMA0IP = 2;           // Priority 
    IFS2bits.DMA0IF = 0;            // Clear Flag
    IEC2bits.DMA0IE = 1;            // Enable Interrupts
    
    DCH0CONbits.CHEN = 1;           // Enable Channel 0
}

void __ISR (_DMA0_VECTOR, IPL2AUTO) DMA0_Handler(void)
{
    if (DCH0INTbits.CHSDIF)
        printf("DMA0 Channel Source Done Interrupt\r\n");
    IFS2bits.DMA0IF = 0;       // Clear Flag
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