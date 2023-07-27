/* 
 * File: LCD.c  
 * Se utilizó y se adaptaron las librerías de Ligo George 
 * de la página www.electrosome.com
 * Enlace: https://electrosome.com/lcd-pic-mplab-xc8/
 * Revision history: 
 */

//LCD Functions Developed by electroSome

#include "LCD8.h"

void Lcd_Port8(char a) {
    PORTD = a;
    /*if (a & 1)
        D1 = 1;
    else
        D1 = 0;

    if (a & 2)
        D2 = 1;
    else
        D2 = 0;

    if (a & 4)
        D3 = 1;
    else
        D3 = 0;

    if (a & 8)
        D4 = 1;
    else
        D4 = 0;

    if (a & 32)
        D5 = 1;
    else
        D5 = 0;

    if (a & 64)
        D6 = 1;
    else
        D6 = 0;

    if (a & 128)
        D7 = 1;
    else
        D7 = 0;*/
}

void Lcd_Cmd8(char a) {
    RS = 0; // => RS = 0 // Dato en el puerto lo va interpretar como comando
    Lcd_Port8(a);
    EN = 1; // => E = 1
    __delay_ms(4);
    EN = 0; // => E = 0
}

void Lcd_Clear8(void) {
    Lcd_Cmd8(0);
    Lcd_Cmd8(1);
}

void Lcd_Set_Cursor8(char a, char b) {
    char temp;
    if (a == 1) {
        temp = 0x80 + b - 1;
        Lcd_Cmd8(temp);
    } else if (a == 2) {
        temp = 0xC0 + b - 1;
        Lcd_Cmd8(temp);
    }
}

void Lcd_Init8(void) {
    Lcd_Port8(0x00);
    __delay_ms(20);
    Lcd_Cmd8(0x30);
    __delay_ms(5);
    Lcd_Cmd8(0x30);
    __delay_ms(11);
    Lcd_Cmd8(0x30);
    /////////////////////////////////////////////////////
    Lcd_Cmd8(0x3C);
    //Lcd_Cmd(0x02);
    Lcd_Cmd8(0x0C);
    Lcd_Cmd8(0x01);
    Lcd_Cmd8(0x0C);
    Lcd_Cmd8(0x00);
    Lcd_Cmd8(0x06);
}

void Lcd_Write_Char8(char a) {
    Lcd_Port8(a);
    RS = 1;
    EN = 1;
    __delay_us(40);
    EN = 0;
}

void Lcd_Write_String8(char *a) {
    int i;
    for (i = 0; a[i] != '\0'; i++)
        Lcd_Write_Char8(a[i]);
}

void Lcd_Shift_Right8(void) {
    Lcd_Cmd8(0x01);
    Lcd_Cmd8(0x0C);
}

void Lcd_Shift_Left8(void) {
    Lcd_Cmd8(0x01);
    Lcd_Cmd8(0x08);
}


