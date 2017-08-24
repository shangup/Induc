

// PIC18F4331 Configuration Bit Settings
#include "main.h"
// 'C' source line config statements

// CONFIG1H
#ifdef CRYSTALL
    #pragma config OSC = HSPLL        // Oscillator Selection bits (Internal oscillator block, CLKO function on RA6 and port function on RA7)
    #define _XTAL_FREQ 40000000
#else
    #pragma config OSC = IRC
    #define _XTAL_FREQ 8000000
#endif

#define PERIOD_FACTOR  ((_XTAL_FREQ)/(8000000))

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



typedef struct {
    bool ADC : 1;
    bool buttonON;
    bool buttonOFF;
} INTERRUPTS;

typedef enum
{
    INIT, IDLE, MOTOR_START, MOTOR_STOP
} STATES;

INTERRUPTS flags;

static void Init(void);
static void temp_start(void);
void modulation_spwm (const unsigned int *in,unsigned int *out, unsigned char div_fac);

unsigned int period = 55*PERIOD_FACTOR; // <= NEED TO BE FUCTION OF XTAL
unsigned int dutyy = 00; // multiply by 4, by own
unsigned char i=0;
unsigned char j=0;
unsigned char k=0;
unsigned int inc = 128; // Frequency SPWM manupilator
unsigned int m = 0;
unsigned int n = 0;
uint16_t spwm[64] = {0};

unsigned char flag[3] = {0};
//static unsigned int spwm[SIZE_OF_SINETABLE] ={0};
unsigned int ADCResult=0;


void main ()

{
STATES mainState = INIT; 
#ifdef CRYSTALL
    while(!OSCCONbits.OSTS ==1);
    flags.ADC = 1;
#else
    OSCTUNEbits.TUN = 0x0F;
    OSCCONbits.IRCF = 7;
    while(!OSCCONbits.IOFS ==1);
    flags.ADC = 0;
#endif
    while(1)
    {
        switch(mainState)
        {
            case INIT:
                Init();
                mainState = IDLE;
                break;
            case IDLE:
                // <- ADD here SW1 START button
                mainState = MOTOR_START;
                break;
            case MOTOR_START:
                // <- ADD HERE IF WANT TO STOP
                // THEN make STATE = STOP
                temp_start();
                break;
            case MOTOR_STOP:
                break;
        }
    }
}

static void Init(){
    
    TRISAbits.TRISA0 = 1;
    ANSEL0bits.ANS0 = 1;
    ADCHS &= 0xFC ; 
    ADCON0 = 0x20 ; // Continous; Single channel mode enable; Single channed mode 1 ; 0;0
    ADCON1 &= 0x2F; // Avref ; Fifo disabled
    ADCON2 = 0x7C; // Left Justified ; Aquisition time 64 TAD ; Clock Fosc/4
    PIR1bits.ADIF = 0; // A/D Flag 0
    ADCON3 = 0x00 ; // Interrupt per word conversion ; Trigger disabled
    
    
    OVDCONS = 0x00;
   
    PTCON0 = 0x02;      // Post scaler TB 1/2 Cause of Center, and 1 for 40Mhz ; 
    //TB input scaler 1; PTMOD = Up/   DOwn count 
    // Currently post scaller is 2 to create lookup increment by 1 per (2 interaction))
    PTPERL = period;// PTPER 55
    PTPERH = period >> 8;
    
    modulation_spwm( pwm, spwm, 2);
            
    PDC0H = spwm[0]>>8;
    PDC0L = spwm[0];
    PDC1H = spwm[SIZE_OF_SINETABLE/4]>>8;
    PDC1L = spwm[SIZE_OF_SINETABLE/4];
    PDC2H = spwm[SIZE_OF_SINETABLE/2]>>8;
    PDC2L = spwm[SIZE_OF_SINETABLE/2];
    
   // DTCON = 0x42; // DTPS = 01b; DT=000010b
    
    TRISD &= 0xF8 ; // xxxx x000
    LATD |= 0x07;
    
    
    
    // Enabling and output pin configurations
    PWMCON0bits.PMOD = 15; // PWM0 and PWM1 Independent 
    PWMCON0bits.PWMEN = 4; // Enable all odd PWMs 1 3 5 PWMS
    OVDCOND = 0x9A;             // To make the FORCED OUTPUT alternating.
    
    PIE3bits.PTIE = 1;
   
    PIR3bits.PTIF = 0;
    INTCONbits.GIE = 1; // Enable Global Interrupt
    INTCONbits.PEIE = 1;


    ADCON0bits.ADON = 1;
    __delay_ms(10);
    PTCON1bits.PTEN = 1; // Enable PWM module
    ADCON0bits.GO = 1;
}

static void temp_start()
{
    if(flags.ADC == 1)
    {
        while(!PIR1bits.ADIF);
        inc = 1 + ADRESH;           // Transfering only the most 8 MSBs
    }
    else 
        inc = 500;
        PIR1bits.ADIF = 0;  
        LATDbits.LATD0 = !(LATDbits.LATD0);
}


void interrupt isr(void)
{
    m=m+(inc*2);
    n = m + 2*16384;
    if ( i > m>>(16-SIZE_OF_SINTABLE_IN_POWER_OF_2) )
    {
        OVDCONDbits.POVD0 = !OVDCONDbits.POVD0;
        if(OVDCONDbits.POVD0 == 0)
        {
            OVDCONSbits.POUT1 = 1;
            OVDCONDbits.POVD1 = 0;
        }
        else
        {
            OVDCONSbits.POUT1 = 0;
            OVDCONDbits.POVD1 = 0;    
        }

        OVDCONDbits.POVD4 = !OVDCONDbits.POVD4;
        
        if(OVDCONDbits.POVD4 == 0)
        {
            OVDCONSbits.POUT5 = 1;
            OVDCONDbits.POVD5 = 0;
        }
        else
        {
            OVDCONSbits.POUT5 = 0;
            OVDCONDbits.POVD5 = 0;    
        }
    }
    i = m>>(16-SIZE_OF_SINTABLE_IN_POWER_OF_2);          // As the Table is of [64 = 2^6], need to shift the register m by (16 - 6 ) = 10 )
    
    if ( j >  n >>(16-SIZE_OF_SINTABLE_IN_POWER_OF_2) )
    {
            OVDCONDbits.POVD2 = !OVDCONDbits.POVD2;
            if(OVDCONDbits.POVD2 == 0){
                OVDCONSbits.POUT3 = 1;
                OVDCONDbits.POVD3 = 0;
            }
            else{
                OVDCONSbits.POUT3 = 0;
                OVDCONDbits.POVD3 = 0;    
            }
    }
    
    j=  n >>(16-SIZE_OF_SINTABLE_IN_POWER_OF_2);

    PDC0H = spwm[i]>>8;
    PDC0L = spwm[i];
    PDC1H = spwm[j]>>8;
    PDC1L = spwm[j];
    PDC2H = spwm[i]>>8;
    PDC2L = spwm[i];
    PIR3bits.PTIF = 0;
}


void modulation_spwm(const unsigned int *in,unsigned int *out, unsigned char div_fac)
{

    for(uint8_t z = 0; z < (SIZE_OF_SINETABLE-1);z++)
        *(out + z) = (*(in + z))*PERIOD_FACTOR/10;
    
}