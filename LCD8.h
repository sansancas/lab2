#ifndef LCD8_H
#define	LCD8_H

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000
#endif

#ifndef RS
#define RS PORTCbits.RC6
#endif

#ifndef EN
#define EN RC7
#endif

#ifndef D0
#define D0 RD0
#endif

#ifndef D1
#define D1 RD1
#endif

#ifndef D2
#define D2 RD2
#endif


#ifndef D3
#define D3 RD3
#endif

#ifndef D4
#define D4 RD4
#endif

#ifndef D5
#define D5 RD5
#endif

#ifndef D6
#define D6 RD6
#endif

#ifndef D7
#define D7 RD7
#endif

#include <xc.h> // include processor files - each processor file is guarded.  

//LCD Functions Developed by electroSome


void Lcd_Port8(char a);

void Lcd_Cmd8(char a);

void Lcd_Clear8(void);

void Lcd_Set_Cursor8(char a, char b);

void Lcd_Init8(void);

void Lcd_Write_Char8(char a);

void Lcd_Write_String8(char *a);

void Lcd_Shift_Right8(void);

void Lcd_Shift_Left8(void);

#endif	/* LCD8_H */

