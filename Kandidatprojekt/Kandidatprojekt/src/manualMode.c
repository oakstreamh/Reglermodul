/*s
 * manualMode.c
 *
 * Created: 4/7/2017 11:44:01 AM
 *  Author: hjaek237
 */ 

#include "servo.h"
#include <avr/interrupt.h>
#include "manualMode.h"

int MANUAL_FORWARD = 2840;
int MANUAL_REVERSE = 2665;
int VELOCITY = 0;
int STEERING = 0;

/*
 * This method performs the manual mode of driving
 * The manual instructions are forward, left, right, reverse stored in array manualInstruction
 *
 */
void manualMode(char manualInstructions, int sF, int sB)
{
	cli(); //disable interrupts	
	
	if (manualInstructions && (1<<5))
	{
		VELOCITY += 5;
	} 
	if (manualInstructions && (1<<6))
	{
		VELOCITY -= 5;
	}
	if (manualInstructions && (1<<1))
	{
		STEERING += 15;
	}
	if (manualInstructions && (1<<2))
	{
		STEERING -= 15;
	}
	if (manualInstructions && (1<<3))
	{
		STEERING = 0;
	}
	if (manualInstructions && (1<<4))
	{
		STEERING = 0;
		VELOCITY = 0;
	}
	
	if (sF<30 || sB<30)
	{
		setESC(NEUTRAL);
	}
	else
	{
		setESC(NEUTRAL + VELOCITY);	
	}
	
	setServo(STRAIGHT + STEERING);
	
}