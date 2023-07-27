// CONFIG1
#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdlib.h>
#include <stdio.h>
#include "ADC.h"
//#include "LCD.h"
#include "LCD8.h"

#define _XTAL_FREQ 4000000
#define RS RC6
#define EN RC7
#define D0 RD0
#define D1 RD1
#define D2 RD2
#define D3 RD3
#define D4 RD4
#define D5 RB5
#define D6 RD6
#define D7 RD7

char volt [4];
char volt2 [4];
int V;
// Prototipos
void setup (void);
void __interrupt() isr(void)
{
    if (ADIF)
    {
        adc_change_channel(1);
        PORTA = adc_get_channel();
        PORTD = adc_read();              // Asiganr el valor del ADC a la variable
        V = map(adc_read(),0,255,0,500);
        ADIF = 0;                            // Apagar la bandera de interrupcion
        return;
    }

}

void main(void)
{
    setup();
    unsigned int a;
    //Lcd_Init();
    Lcd_Init8();
    
    while (1)
    {
        if(ADCON0bits.GO == 0)
        {
           __delay_ms(10);
            ADCON0bits.GO = 1;  
        }
        
        /*Lcd_Clear();
        Lcd_Set_Cursor(1,1);
        Lcd_Write_String("Voltaje");
        Lcd_Set_Cursor(2,1);
        sprintf(volt, "%d\r.", (V/100));
        sprintf(volt2,"%d\r", (V%100));
        Lcd_Write_String(volt);
        Lcd_Write_String(volt2);
        __delay_ms(2000);*/
        
        Lcd_Clear8();
        Lcd_Set_Cursor8(1,1);
        Lcd_Write_String8("Voltaje");
        Lcd_Set_Cursor8(2,1);
        sprintf(volt, "%d\r.", (V/100));
        sprintf(volt2,"%d\r", (V%100));
        Lcd_Write_String8(volt);
        Lcd_Write_String8(volt2);
        __delay_ms(2000);

    
        return;
    }
    
   
    
  }
    


void setup(void)
{
    TRISA = 0b00000011;
    TRISD = 0;
    TRISC = 0;
    
    
    PORTA = 0;
    PORTC = 0;
    PORTD = 0;
    
    ANSEL = 0b00000011;
    ANSELH= 0;
    
    OSCCONbits.IRCF =0b111; 
    OSCCONbits.SCS = 1; 
    
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.T0IE = 1;
    
    INTCONbits.T0IF = 0;
    INTCONbits.RBIF = 0;
    
    adc_init(1);
    
    return;
}

