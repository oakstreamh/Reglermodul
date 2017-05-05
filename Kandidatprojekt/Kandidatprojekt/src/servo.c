<<<<<<< HEAD
﻿//////////////////////////////////////////////////////////////////////////////////
=======

//////////////////////////////////////////////////////////////////////////////////
>>>>>>> origin/master
// servo.c contains methods, variables and constants for the speed and          //
// steering servo signals                                                       //
// AUTHORS: MATHIAS DALSHAGEN & HJALMAR EKSTRÖM                                 //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

//////////////// DEFINITIONS /////////////////////////////////////////////////////


//////////////// INCLUSIONS //////////////////////////////////////////////////////

#include <asf.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "servo.h"

//////////////// CONSTANTS ///////////////////////////////////////////////////////


double Q1 = 0.8;
double Q2 = 0.2;

//////////////// METHODS /////////////////////////////////////////////////////////

/*
 * Configuration setup for PWM signals in Timer/Counter 1:
 * Pins 19(OC1A) and 18(OC1B) setup for output for ESC and steering servo respectively.
 * The formula of pwm length with current counter register settings:
 * pwm_length = counterValue/1843
*/
void pwmInit(void)
{
	DDRD = (1<<DDD5)|(1<<DDD4);
	TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
	ICR1 = 36863;
	TCCR1B = (1<<CS11)|(1<<WGM12)|(1<<WGM13);
	OCR1A = NEUTRAL;
	OCR1B =STRAIGHT;
}

/* This method sets the ESC pwm length. The extreme values are MINESC and MAXESC.
 * Input values outside of the valid interval are forced to the lower/upper limit
 * @param int counterEsc sets the upper limit to the 16-bit counter1 in the processor
*/
void setESC(int counterEsc){
	
	if(counterEsc <= MINESC)
	{
		OCR1A = MINESC;
	}
	else if  (counterEsc >= MAXESC)
	{
		OCR1A = MAXESC;
	} else {
		OCR1A = counterEsc;
	}
	
}


/* This method sets the PWM length for the servo signal that controls
 * the steering angle. The extreme values are defined as MAXRIGHT and MAXLEFT
 * Input values outside of the valid interval are forced to the lower/upper limit
 * @parameter int counterServo sets the upper limit to the 16-bit counter1 in the processor
*/
void setServo (int counterServo)
{
	if (counterServo > MAXRIGHT){
		
		OCR1B = MAXRIGHT;
		
	}	else if (counterServo < MAXLEFT){

		OCR1B = MAXLEFT;
		
	} else {
		OCR1B = counterServo;
	}
}