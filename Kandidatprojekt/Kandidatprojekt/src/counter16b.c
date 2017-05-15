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



void count(int mode)
{
	if (mode == 1)
	{
		TCNT3 = 0;
		TCCR3B = (1<<CS32)|(1<<CS30);
	}
	else if (mode == 0)
	{
		TCCR3B = (0<<CS32)|(0<<CS30);
	}
	
}

int checkCount(int req_delay)
{
	int req_count = (int) req_delay*F_CPU/1024-1;
	if(TCNT3<req_count)
	{
		return 0;
	}
	return 1;
}