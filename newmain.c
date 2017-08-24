

// PIC18F4331 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = IRC        // Oscillator Selection bits (Internal oscillator block, CLKO function on RA6 and port function on RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switchover bit (Internal External Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDPS = 32768     // Watchdog Timer Postscale Select bits (1:32768)
#pragma config WINEN = OFF      // Watchdog Timer Window Enable bit (WDT window disabled)

// CONFIG3L
#pragma config PWMPIN = OFF     // PWM output pins Reset state control (PWM outputs disabled upon Reset (default))
#pragma config LPOL = HIGH      // Low-Side Transistors Polarity (PWM0, 2, 4 and 6 are active-high)
#pragma config HPOL = HIGH      // High-Side Transistors Polarity (PWM1, 3, 5 and 7 are active-high)
#pragma config T1OSCMX = ON     // Timer1 Oscillator MUX (Low-power Timer1 operation when microcontroller is in Sleep mode)

// CONFIG3H
#pragma config FLTAMX = RC1     // FLTA MUX bit (FLTA input is multiplexed with RC1)
#pragma config SSPMX = RC7      // SSP I/O MUX bit (SCK/SCL clocks and SDA/SDI data are multiplexed with RC5 and RC4, respectively. SDO output is multiplexed with RC7.)
#pragma config PWM4MX = RB5     // PWM4 MUX bit (PWM4 output is multiplexed with RB5)
#pragma config EXCLKMX = RC3    // TMR0/T5CKI External clock MUX bit (TMR0/T5CKI external clock input is multiplexed with RC3)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Low-Voltage ICSP Enable bit (Low-voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000200-000FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (001000-001FFF) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFF) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFF) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic18f4331.h>


#define _XTAL_FREQ 8000000

unsigned int period = 55;
unsigned int dutyy = 00; // multiply by 4, by own
const unsigned int spwm[128]= {110,114,119,123,127,131,136,140,144,148,151,155,159,162,166,169,172,175,178,181,183,185,188,190,191,193,194,195,196,197,198,198,198,198,198,197,196,195,194,193,191,190,188,185,183,181,178,175,172,169,166,162,159,155,151,148,144,140,136,131,127,123,119,114,110,106,101,97,93,89,84,80,76,72,69,65,61,58,54,51,48,45,42,39,37,35,32,30,29,27,26,25,24,23,22,22,22,22,22,23,24,25,26,27,29,30,32,35,37,39,42,45,48,51,54,58,61,65,69,72,76,80,84,89,93,97,101,106};
unsigned char i=0;
unsigned char j=0;
unsigned char k=0;
unsigned int inc = 128; // Frequency SPWM manupilator
unsigned int m = 0;

unsigned int ADCResult=0;


void main ()

{
    OSCTUNEbits.TUN = 0x0F;
    OSCCONbits.IRCF = 7;
    while(!OSCCONbits.IOFS ==1);
    
    
    
    
    TRISAbits.TRISA0 = 1;
    ANSEL0bits.ANS0 = 1;
    ADCHS &= 0xFC ; 
    ADCON0 = 0x20 ; // Continous; Single channel mode enable; Single channed mode 1 ; 0;0
    ADCON1 &= 0x2F; // Avref ; Fifo disabled
    ADCON2 = 0x7C; // Right Justified ; Aquisition time 64 TAD ; Clock Fosc/4
    PIR1bits.ADIF = 0; // A/D Flag 0
    ADCON3 = 0x00 ; // Interrupt per word conversion ; Trigger disabled
    
    
    
    
    
    PTCON0 = 0x02;      // Post scaler TB 1/2; TB input scaler 1; PTMOD = Up/   DOwn count // Currently post scaller is 2 to create lookup increment by 1 per (2 interaction))
    PTPERL = period;// PTPER 55
    PTPERH = period >> 8;
    
    PDC0H = spwm[0]>>8;
    PDC0L = spwm[0];
    PDC1H = spwm[32]>>8;
    PDC1L = spwm[32];
    PDC2H = spwm[64]>>8;
    PDC2L = spwm[64];
    
   // DTCON = 0x42; // DTPS = 01b; DT=000010b
    
    
    
    
    
    // Enabling and output pin configurations
    PWMCON0bits.PMOD = 0; // PWM0 and PWM1 complementary output
    PWMCON0bits.PWMEN = 4; // Enable PWM output for PWM0 and PWM1
    
    PIE3bits.PTIE = 1;
    
    
    PIR3bits.PTIF = 0;
    INTCONbits.GIE = 1; // Enable Global Interrupt
    INTCONbits.PEIE = 1;


    ADCON0bits.ADON = 1;
    __delay_ms(10);
    PTCON1bits.PTEN = 1; // Enable PWM module
    ADCON0bits.GO = 1;
    while(1)
    {
     
        while(!PIR1bits.ADIF);
        inc = 1 + ADRESH;
        PIR1bits.ADIF = 0;
     
        
        /*
    while(!PIR3bits.PTIF);
    
    i++;
    PDC0H = spwm[i]>>8;
    PDC0L = spwm[i];
    if(i==127)
        i=0;
    PIR3bits.PTIF = 0;
    */
    
    }
}
void interrupt isr(void)
{
    m=m+inc;
    i = m>>9;
    
    j= (m+16354>>9);
    //m= m+32768;
    k= ((unsigned int)(m+32768)>>9);
    
    
    PDC0H = spwm[i]>>8;
    PDC0L = spwm[i];
    PDC1H = spwm[j]>>8;
    PDC1L = spwm[j];
    PDC2H = spwm[k]>>8;
    PDC2L = spwm[k];
    //i=i>>1;
    PIR3bits.PTIF = 0;
}