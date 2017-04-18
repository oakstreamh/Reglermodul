/*
 * test_methods.c
 *
 * Created: 4/17/2017 9:13:20 AM
 *  Author: hjaek237
 */ 

#define F_CPU 14745600


#include "test_methods.h"
#include "servo.h"
#include <util/delay.h>





void testmanual(unsigned char commando)
{
	
	if(commando == 'w')
	{
		setESC(MANUAL_FORWARD);
		setServo(NEUTRAL);
	}
	else if (commando == 's')
	{
		setESC(MANUAL_REVERSE);
	}
	else if (commando == 'd')
	{
		setServo(MAXRIGHT);
	}
	else if (commando == 'a')
	{
		setServo(MAXLEFT);
	}
	else if (commando == 'q')
	{
		setESC(NEUTRAL);
	}
	
	
}

/* Current limits are defined in the servo.c-file
 * The first test provided the following:
 *
 * 1843 => 1 ms too much right
 * 2000 => 1.08 ms too much right
 * 2100 => 1.14 ms too much right
 *
 * APPROX MAX RIGHT: 2022 => 1.09 ms
 *
 *
 * 2690 => 1.459 ms STRAIGHT
 * 2764 => 1.5 ms slight left/center
 *
 * APPROX MAX LEFT: 3060 => 1.66 ms
 *
 * 3086 => 1.68 ms too much left
 * 3386 => 1.86 ms too much left
 * 3586 => 1.96 ms too much left
 * 3686 => 2.0 ms too much left
 */
void testSteering(void)
{

//
setServo(3135);



//setServo(STRAIGHT+160);
	
}

void testSpeed(void)
{
	
	// It was found that 2764 + 110 is a good MAXLIMIT for driving on the floor
	// and 2765 - 110 is a good MINLIMIT for driving on the floor
	
	setServo(STRAIGHT-300);
	_delay_ms(2000);
	
	setESC(NEUTRAL+90);
	
	
	// 
	_delay_ms(3000);
	
	setESC(NEUTRAL);
}


void testRoad(void)
{
	
	_delay_ms(2000);
	setESC(NEUTRAL+70);
	
	
	
}