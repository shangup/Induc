

// PIC18F4331 Configuration Bit Settings
#include "main.h"
// 'C' source line config statements

// CONFIG1H
#define _XTAL_FREQ 40000000

#if _XTAL_FREQ == 40000000
    #pragma config OSC = HSPLL        // Oscillator Selection bits (Internal oscillator block, CLKO function on RA6 and port function on RA7)
#else
    #pragma config OSC = IRC
#endif

// OTHERS

#define RISING_EDGE 1
#define FALLING_EDGE 0

#define FREQ_4545 4545
#define SET_FREQ   FREQ_4545

// In Hz.
#define MAX_FREQ 60
#define MIN_FREQ 25
#define RATED_FREQ 50

// Current
#define AMP_VOLT_CONV 2 // 2AMP per Volt
#define MAX_CURRENT 10 // Need to make a fuction as this is an instataneosu current.
#define CYCLE_AVG SIZE_OF_SINETABLE
#define I_MAX 155

#ifdef POT_BASED_FREQVOLT 
#define MATCH_SPEED 10 // FOR NOW A RANDOM NUMBER. THIS IS TO slowly changingn
// the frequenct and voltage
#endif


// Voltage
#define REF_VOLTAGE 220
#define MAX_VOLTAGE 220 //(IN DIVISIBLE OF 2power) NOT ACTUAL, need to be tested.
#define MIN_VOLTAGE (MAX_VOLTAGE/2)
#define Q_VOLTAGE 5  // I GUESS THE COMPARATOR WORKS ON PERIOD TIMES 4 : *NEED TO MAKE A GENERALISE CODE*
#define MAX_VOLT_FACT MAX_VOLTAGE/REF_VOLTAGE
#define PERIOD_FACTOR  ((_XTAL_FREQ)/(8000000))

#if(SET_FREQ == FREQ_4545)
#define PERIOD 220 // 220, For 4545
#endif

#define TEMP_SLOPE    // 0.98304 READ/Degree celcium
#define MAX_TEMP 73

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
    uint8_t voltQ7; // (MIN_VOLTAGE/MAXVOTLAGE)/(128))
    uint16_t accum_m;
    uint16_t accum_n;
    uint16_t increment;
    uint8_t Volt_inc;
    uint8_t uvw[3];
    //uint16_t Voltuvw[3] =0;
}Modulate;

typedef struct{
    uint16_t VoltClckComp;
    
}Comp;

Comp compensate;
Modulate modulate;


typedef struct {
    bool ADC : 1;
    bool buttonON_OFF;
    bool FAULT;
    bool overflow_i;
    bool overflow_j;
    bool TEMP;
    bool OVER_CURRENT;
} INTERRUPTS;

typedef struct{
    uint8_t Pot1;
    uint8_t Pot2;
    uint8_t Temp;
    uint8_t Idc;
}ADC_buffer;

typedef enum
{
    INIT = 0 , IDLE = 2, MOTOR_START = 1, MOTOR_STOP=4
} STATES;

INTERRUPTS flags = {0,0,0,0,0,0,0};
ADC_buffer ADC_BUF;

static void Init(void);
void UART(void);
//static void temp_start(void);

STATES mainState;
unsigned int period = (SET_FREQ*8); // <= NEED TO BE FUCTION OF XTAL CURRENT 220 for 4.54kHz
//unsigned int period = (uint16_t)((uint32_t)_XTAL_FREQ/period); // <= NEED TO BE FUCTION OF XTAL CURRENT 220 for 4.54kHz
unsigned int dutyy = 00; // multiply by 4, by own
unsigned char i=0;
unsigned char j=0;
unsigned char k=0;
unsigned int ADCResult=0;


void main ()

{
    period = (uint16_t)((uint32_t)_XTAL_FREQ/period);
    mainState = INIT; 
#if _XTAL_FREQ == 40000000
    while(!OSCCONbits.OSTS ==1);
    flags.ADC = 1;
#else
    OSCTUNEbits.TUN = 0x0F;
    OSCCONbits.IRCF = 7;
    while(!OSCCONbits.IOFS ==1);
    flags.ADC = 1;
#endif
    LED_D3_DIR = 0;
    LED_D2_DIR = 0;
    LED_D1_DIR = 0;
    IPM_SW = 1;
    IPM_SW_DIR = 0;
    IPM_SW = 1;
        
    LED_D3_ON = 0;
    LED_D2_IDC = 0;
    LED_D1_TEMP = 0;

    __delay_ms(500);
    while(1)
    {
        switch(mainState)
        {
            case INIT:
                flags.FAULT = 1;
                IPM_SW = 1;                
                Init();
                mainState = IDLE;
                break;
            case IDLE:
                ADCON0bits.GO = 1;
                if(INTCONbits.INT0F == 1 && PIR1bits.ADIF)
                {
                    PIR1bits.ADIF = 0;
                    modulate.increment = 127;
                    LED_D3_ON = 1;
                    INTCONbits.INT0F = 0;
                    mainState = MOTOR_START;
                    ADCON0bits.GO = 1;
                    PTCON1bits.PTEN = 1; // Enable PWM module
                    __delay_ms(500);
                    IPM_SW = 0; 
                }
                break;
            case MOTOR_START:
                
                //temp_start();
                while(mainState == MOTOR_START)
                {
                    if(INTCONbits.INT0F == 1)
                    {
                        mainState = MOTOR_STOP;
                        INTCONbits.INT0IF = 0;
                    }
                }
                break;
            case MOTOR_STOP:
                
                INTCONbits.GIE = 0; // Enable Global Interrupt
                INTCONbits.PEIE = 0;
                LED_D3_ON = 0;
                OVDCONS = 0x00; // DECEIDE FORCE STATE
                OVDCOND = 0x00;
                __delay_ms(1);
                
                OVDCONS = 0xAA; // DECEIDE FORCE STATE 
                OVDCOND = 0x00; // APPLY FORCED STATE When 0
                __delay_ms(500);
                //PTCON1bits.PTEN = 0; // Enable PWM module
                ADCON0bits.ADON = 0;
                mainState = INIT;
                // NEED TO REINITIATE EVERYTHING
                break;
        }
        //if(flags.FAULT)
          //  LED_D2_IDC = 1;
        //LED_D1_ON =( mainState==MOTOR_START);
    }
}

static void Init()
{
    compensate.VoltClckComp = period>>2;
    modulate.voltQ7 = 64; // (MIN_VOLTAGE/MAXVOTLAGE)/(128))
    modulate.accum_m = 0;
    modulate.accum_n = 0;
    modulate.uvw[0] = 0;
    modulate.uvw[1] = 0;
    modulate.uvw[2] = 0;
    
    
    PTCON1bits.PTEN = 0; // Enable PWM module
    //ADCON0bits.GO = 0;        
    UART();
    
    POT1_DIR = 1; // Setting A0:A3 as Input Direction
    POT2_DIR = 1;
    TEMP_DIR = 1;
    Idc_DIR = 1;
    
    POT1_ADC = 1; // Setting pins as Analog Input A0:A3
    POT2_ADC = 1;
    TEMP_ADC = 1;
    Idc_ADC = 1;
    
    ADCHS = 0x00 ; 
    ADCON0bits.ACONV = 0; // Single Mode
    ADCON0bits.ACSCH = 1; // Multichannel mode enable
    ADCON0bits.ACMOD = 1; // SEQM2 Sample all four Group
    
    //ADCON0 = 0x20 ; // Continous; Single channel mode enable; Single channed mode 1 ; 0;0
    ADCON1bits.VCFG = 0; // Supplies as Vref
    ADCON1bits.FIFOEN= 1; // FIFO EnABLED
    
    ADCON2 = 0x60; // Left Justified ; Aquisition time 36 TAD ; Clock Fosc/2
    PIR1bits.ADIF = 0; // A/D Flag 0
    
    ADCON3 = 0xB0; // Interrupt when 4th word conversion ; Trigger disabled, default 0x00
    
    
    OVDCONS = 0x00;
    PTCON0 = 0x02; 
    // TIME BASE POST SCALER PTCON0[4:7] ; 
    //INOPUT CLOCK SCALER [2:3] (1/4))
    // PWM MODE, CONTINOUS [0:1] 
    PTPERL = period;// PTPER 55
    PTPERH = period >> 8;
    
            
    PDC0H = pwm[0]>>8;
    PDC0L = pwm[0];
    PDC1H = pwm[SIZE_OF_SINETABLE/4]>>8;
    PDC1L = pwm[SIZE_OF_SINETABLE/4];
    PDC2H = pwm[SIZE_OF_SINETABLE/2]>>8;
    PDC2L = pwm[SIZE_OF_SINETABLE/2];
    
    // Enabling and output pin configurations
    PWMCON0bits.PMOD = 15; // PWM0 and PWM1 Independent 
    PWMCON0bits.PWMEN = 4; // Enable all odd PWMs 1 3 5 PWMS
    OVDCOND = 0xAA;             // To make the FORCED OUTPUT alternating. // DEF 0x9A
    
    PIR3bits.PTIF = 0;
    INTCONbits.INT0IF = 0;
    PIE3bits.PTIE = 1; // TIME BASE INTERRUPT
    
    INTCON2bits.INTEDG0 = RISING_EDGE;
    INTCONbits.INT0F = 0; 
    INTCONbits.INT0IE = 0; // NO NEED TO ON THE INTTERUPT, AS NOT HIGH PRIOR
    
    INTCONbits.GIE = 1; // Enable Global Interrupt
    INTCONbits.PEIE = 1;

    ADCON0bits.ADON = 1;
    __delay_ms(10);

}

void interrupt isr(void)
{

    if(PIR3bits.PTIF == 1 && mainState == MOTOR_START )
    {
        if(ADCON1bits.ADPNT == 00)
        {
            ADC_BUF.Pot1 = ADRESH;
            ADC_BUF.Pot2 = ADRESH;
            ADC_BUF.Temp = ADRESH;
            ADC_BUF.Idc = ADRESH;
        }
        
        if(ADC_BUF.Idc > I_MAX)
        {
            mainState = MOTOR_STOP;
            LED_D2_IDC = 1;
            IPM_SW = 1; 
        }
        ADCON0bits.GO = 1;
        modulate.uvw[0]++;
        if(modulate.uvw[0] > 5)
        {
            modulate.uvw[0] = 0;
            TXREG = ADC_BUF.Pot1;
        }
            
        uint16_t buff_voltage = 0;
        uint16_t m_past,n_past;
        m_past = modulate.accum_m;
        n_past = modulate.accum_n;
        //inc  += 720;
        //inc = inc*2;
        uint16_t sat_inc = modulate.increment*8;
        if(sat_inc > 1024)
            sat_inc = 1024;
        modulate.accum_m += sat_inc;   // (4545/Min_freq) <- intersect times.
        modulate.accum_m = modulate.accum_m+720;   // (4545/Min_freq) <- intersect times.
                        // 65536/ (Intersect time)) <- Increment Values
        modulate.accum_n = modulate.accum_m + 2*16384;
        //buff_voltage = 64 +  (modulate.increment/11)*4; // Actually it should be 11.25
        buff_voltage = modulate.Volt_inc; // Actually it should be 11.25

        if (buff_voltage > 128)
             buff_voltage = 128;
        
        buff_voltage = buff_voltage*compensate.VoltClckComp;      // Because PERIOD IS 220 in this case
        buff_voltage = buff_voltage>>(7+2);   // originally Doont need this. just copy

        modulate.voltQ7 = buff_voltage;
        if (m_past > modulate.accum_m )
            flags.overflow_i = 1;
        if( i < 10 && flags.overflow_i)
        {
            flags.overflow_i = 0;
            OVDCONDbits.POVD0 = !OVDCONDbits.POVD0;
            if(OVDCONDbits.POVD0 == 0)
            {
                OVDCONSbits.POUT1 = 1;
                OVDCONDbits.POVD1 = 0;
            }
            else
            {
                //PDC0L = 0;
                OVDCONSbits.POUT1 = 0;
                OVDCONDbits.POVD1 = 0;    
            }

            OVDCONDbits.POVD4 = !OVDCONDbits.POVD0;

            if(OVDCONDbits.POVD0 == 1)
            {
                OVDCONSbits.POUT5 = 1;
                OVDCONDbits.POVD5 = 0;
            }
            else
            {
                //PDC2L = 0;
                OVDCONSbits.POUT5 = 0;
                OVDCONDbits.POVD5 = 0;    
            }
            
            int diff_inc = modulate.increment - (uint8_t)ADC_BUF.Pot1;
            if (diff_inc < -5)
            {
                modulate.increment++;
            }
            else if(diff_inc > 5)
                modulate.increment--;
            
            int diff_inc = modulate.Volt_inc - (uint8_t)ADC_BUF.Pot2;
            if (diff_inc < -3)
            {
                modulate.Volt_inc++;
            }
            else if(diff_inc > 3)
                modulate.Volt_inc--;
            
            if(ADC_BUF.Temp > MAX_TEMP)
            {
                flags.TEMP = 1;
                mainState = MOTOR_STOP;
                IPM_SW = 1; 
            }
        
        }
                // As the Table is of [64 = 2^6], need to shift the register m by (16 - 6 ) = 10 )
        
        i = modulate.accum_m>>(16-SIZE_OF_SINTABLE_IN_POWER_OF_2); 
        
        if ( n_past>  modulate.accum_n )
            flags.overflow_j = 1;
        
        if(j < 10 && flags.overflow_j)
        {
            flags.overflow_j = 0;
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
        j=  modulate.accum_n >>(16-SIZE_OF_SINTABLE_IN_POWER_OF_2);
        
        // LATER change the variable i and j 
        uint16_t Vabc[3];
        Vabc[0] = (pwm[i]*modulate.voltQ7)>>(Q_VOLTAGE-2); 
        // Already compensated 2^2 above, REASON, 10-bit period register,buff_voltage for 
        //1100 period is 275 (9-bit),, hence made it (9-2 bit); SIN TABLE has 9 bit table
        // LOOSING VOLTAGE MODULATION RESOLUTION
        PDC0H = Vabc[0] >>8;
        PDC0L = Vabc[0] ;
        PDC2H = Vabc[0] >>8;
        PDC2L = Vabc[0] ;
        Vabc[0] = (pwm[j]*modulate.voltQ7)>>(Q_VOLTAGE-2);
        PDC1H = Vabc[0]>>8;
        PDC1L = Vabc[0];

        PIR3bits.PTIF = 0;
    }
}

void UART(void)
{
    BAUDCONbits.BRG16 = 0;
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    SPBRGH = 0x00;
    SPBRG = 21; // Baud Rate 113.636 ~ 115.20
    RCSTAbits.SPEN = 1;
    TXSTAbits.TX9 = 1; // Transmit, one start and 8 data bits.
    TXSTAbits.TXEN = 1;
    // TXREG load data. 
    // Check TXIF for whether no process in transission. CLEARS AUTOMATICALLY
    
}