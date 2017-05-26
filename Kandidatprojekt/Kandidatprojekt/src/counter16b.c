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


ISR(TIMER3_COMPA_vect)
{
    step=step+1;
}


void countInit(int req_delay)
{
    OCR3A =  req_delay;
	TCNT3 = 0;
    TIMSK3 = (1<<OCIE3A);
	TCCR3B = (1<<WGM32)|(1<<CS32)|(1<<CS30); // CTC-mode 4 to compare with OCR1A, 
	
    
	step = 0;
	
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
