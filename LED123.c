


// PIC18F4331 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = HSPLL //IRC //HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (clock frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = ON       // Internal External Oscillator Switchover bit (Internal External Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled)
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
#pragma config MCLRE = ON       // MCLR Pin Enable bit (Enabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Low-Voltage ICSP Enable bit (Low-voltage ICSP enabled)

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


#define _XTAL_FREQ 40000000

unsigned int period = 55;
unsigned int dutyy = 00; // multiply by 4, by own
const unsigned int spwm[128]= {110,114,119,123,127,131,136,140,144,148,151,155,159,162,166,169,172,175,178,181,183,185,188,190,191,193,194,195,196,197,198,198,198,198,198,197,196,195,194,193,191,190,188,185,183,181,178,175,172,169,166,162,159,155,151,148,144,140,136,131,127,123,119,114,110,106,101,97,93,89,84,80,76,72,69,65,61,58,54,51,48,45,42,39,37,35,32,30,29,27,26,25,24,23,22,22,22,22,22,23,24,25,26,27,29,30,32,35,37,39,42,45,48,51,54,58,61,65,69,72,76,80,84,89,93,97,101,106};
unsigned char i=0;
unsigned char j=0;
unsigned char k=0;
unsigned int inc = 500; // Frequency SPWM manupilator
unsigned int m = 0;

unsigned int ADCResult=0;


void main ()

{
    //OSCTUNEbits.TUN = 0x0F;
    //OSCCONbits.IRCF = 7;
    while(!OSCCONbits.OSTS ==1);
    
    
    TRISAbits.TRISA0 = 1;
    ANSEL0bits.ANS0 = 1;
    ADCHS &= 0xFC ; 
    ADCON0 = 0x20 ; // Continous; Single channel mode enable; Single channed mode 1 ; 0;0
    ADCON1 &= 0x2F; // Avref ; Fifo disabled
    ADCON2 = 0x7C; // Right Justified ; Aquisition time 64 TAD ; Clock Fosc/4
    PIR1bits.ADIF = 0; // A/D Flag 0
    ADCON3 = 0x00 ; // Interrupt per word conversion ; Trigger disabled
   
    
    /*
    TRISAbits.TRISA1 = 1;
    ANSEL0bits.ANS1 = 1;
    ADCHS &= 0xCF ; 
    ADCON0 = 0x24 ; // Continous; Single channel mode enable; Single channed mode 1 ; 0;0 SCM2, CONVERSION OF ChANNEL B ONLY
    ADCON1 &= 0x2F; // Avref ; Fifo disabled
    ADCON2 = 0x7C; // Right Justified ; Aquisition time 64 TAD ; Clock Fosc/4
    PIR1bits.ADIF = 0; // A/D Flag 0
    ADCON3 = 0x00 ; // Interrupt per word conversion ; Trigger disabled
    */
    
    
    TRISD &= 0xF8 ; // xxxx x000
    LATD |= 0x07;
    i = 0x01;
    ADCON0bits.ADON = 1;
   __delay_ms(10);
    ADCON0bits.GO = 1;
    while(1)
    {
     //   i = 0x01;
        /*
        LATD |= 0X07;
        __delay_ms(500);
         LATD &= 0XF8;
         __delay_ms(500);
         */
   
        PORTD &= 0xF8;
        PORTD |=  0x01;//(0xF8 | 0x01);
        for(m = inc; m>2;m-- )
            __delay_ms(1);
        PORTD &= 0xF8;
        PORTD |= 0x02; // (0xF8 | 0x02);
        for(int m = inc; m>2;m-- )
            __delay_ms(1);
        PORTD &= 0xF8;
        PORTD |= 0x04;  //(0xF8 | 0x04);
        for(int m = inc; m>2;m-- )
            __delay_ms(1);
        
        
        if(PIR1bits.ADIF)
        {
            inc = 1 + ADRESH;
            PIR1bits.ADIF = 0;
        }
        
    }
}
