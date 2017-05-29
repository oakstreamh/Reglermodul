/*
* stopLine.c
*
* Created: 5/12/2017 12:55:09 PM
*  Author: hjaek237
*/

#include "servo.h"
#include <string.h>
#include "fuzzySteering.h"
#include "general_FIS.h"
#include <stdio.h>
#include "counter16b.h"
#include <asf.h>
#include <avr/interrupt.h>
#include <avr/io.h>



void stop()
{
    
    if (step==0)
    {
        setESC(2836);
        setServo(MAXRIGHT);
    }
    else if (step==1)
    {
		OCR3A = 1000;
		setServo(STRAIGHT);
	}
	else if (step ==2)
	{
		OCR3A = 30000;
        setServo(MAXLEFT);
    }
    else if (step==3)
    {
        setESC(NEUTRAL);
        setServo(STRAIGHT);
		OCR3A = 14399;
    }
    else if (step==4)
    {
		OCR3A = 30000;
        setESC(2836);
        setServo(MAXLEFT);
    }
	else if (step==5)
	{
		OCR3A = 27000;
		setServo(MAXRIGHT);
	}
	else if (step==6)
	{
		setServo(STRAIGHT);
		OCR3A = 10000;
	}
	else if (step==7)
	{
		isParking = 0;
		resetCounter();
	}
	
}
