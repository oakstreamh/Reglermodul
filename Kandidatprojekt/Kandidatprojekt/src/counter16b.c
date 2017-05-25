/*
 * counter16b.c
 *
 * Created: 5/15/2017 10:35:10 AM
 *  Author: hjaek237
 */ 

#define F_CPU 14745600

#include <asf.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "counter16b.h"
#include <stdint.h>
#include <string.h>


void ISR(TIMER3_OVF)
{
    overflow++;
}


void countInit(int req_delay)
{
    OCR3A = (int) req_delay*F_CPU/1024000-1
    
    TCCR3A = (1<<WGM32);                // CTC-mode compare with OCR1A
    TIMSK = (1<<TOIE3);
	TCCR3B = (1<<CS32)|(1<<CS30);
	
    TCNT3 = 0;

	
}

int checkCount(uint16_t req_delay)
{
	uint16_t req_count = (uint16_t) req_delay*F_CPU/1024000-1;
	if(TCNT3<req_count)
	{
		return 0;
	}
	return 1;
}
