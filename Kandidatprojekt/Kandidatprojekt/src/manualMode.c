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
void manualMode(char manualInstructions, int sF, int sB, int* man_velocity, int* man_steering)
{
	cli(); //disable interrupts	
	
	if (manualInstructions & (1<<4))
	{
		*man_velocity = *man_velocity +  5;
	} 
	if (manualInstructions & (1<<5))
	{
		*man_velocity = *man_velocity - 5;
	}
	if (manualInstructions & (1<<0))
	{
		*man_steering = *man_steering - 100;
	}
	if (manualInstructions & (1<<1))
	{
		*man_steering = *man_steering + 100;
	}
	if (manualInstructions & (1<<2))
	{
		*man_steering = 0;
	}
	if (manualInstructions & (1<<3))
	{
		*man_steering = 0;
		*man_velocity = 0;
	}
	
	if (sF<30 && (*man_velocity > 0))
	{
		setESC(NEUTRAL);
		*man_velocity = 0;
	} 
	else if(sB<30 && (*man_velocity < 0))
	{
		setESC(NEUTRAL);
		*man_velocity = 0;
	}
	else
	{
		setESC(NEUTRAL + *man_velocity);	
	}
	
	setServo(STRAIGHT + *man_steering);
	
}