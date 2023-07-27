#ifndef ADC_H
#define	ADC_H

#include <xc.h>
// Configuracion del ADC
void adc_init(int channel);
int adc_read ();
void adc_change_channel(int channel);
int adc_get_channel();
int map (int val, int minx, int maxx, int miny, float maxy);

#endif	/* ADC_H */