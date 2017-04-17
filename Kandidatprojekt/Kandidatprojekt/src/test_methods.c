/*
 * test_methods.c
 *
 * Created: 4/17/2017 9:13:20 AM
 *  Author: hjaek237
 */ 

#include "test_methods.h"
#include "servo.h"






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


void testSteering(void)
{
	
	
	
}