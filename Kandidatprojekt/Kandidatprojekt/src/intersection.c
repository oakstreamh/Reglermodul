//
//  intersection.c
//
//
//  Created by Mathias Dalshagen on 2017-05-03.
//
//

#include "intersection.h"
#include "servo.h"
#include <string.h>
#include "fuzzySteering.h"
#include "general_FIS.h"
#include <stdio.h>
#include "nFuzzySteering.h"
#include "counter16b.h"
#include <avr/io.h>

// DEFINITIONS
//////////////////////////////////////////////////////////////////////////////////////

#define maxGyro 125
#define minGyro 0

// PROTOTYPES
//////////////////////////////////////////////////////////////////////////////////////
void rightTurn(int gyro);
void leftTurn(int gyro);
void straightIntersection(int c1);



void straightIntersection(int c1)
{
	
	

	
	///// DECLARATION OF C INPUT VARIABLE ///////////////////////////////////
	//
	struct io_type cPosition; strcpy(cPosition.name, "cPosition");


	// set iErr's input value to measErr value
	if(c1<50)				// if sensor value is smaller than cPosition's input set's lower limit
	{
		cPosition.value = 50;  // force input value to lowest point in cPosition's input set
	}
	else if(c1>400)			// if sensor value is bigger than cPosition's input set's upper limit
	{
		cPosition.value = 400;  // force input value to lowest point in cPosition's input set
	}
	else
	{
		cPosition.value = c1;
	}
	struct mf_type cRight;
	MATLAB_MF(&cRight, "cRight", 49, 50, 80, 100); // Min_value = 160
	struct mf_type centre;
	MATLAB_MF(&centre, "centre", 80, 120, 120, 160);
	struct mf_type cLeft;
	MATLAB_MF(&cLeft, "cLeft", 140, 300, 400, 401); // Max_value = 370


	cPosition.membership_functions = &cRight;
	cRight.next = &centre;
	centre.next = &cLeft;
	cLeft.next = NULL;

	
	///// DECLARATION OF STEERING OUTPUT VARIABLE ///////////////////////////////////
	//
	struct io_type steering; strcpy(steering.name, "steering"); // All outputs downscaled by a factor 10

	struct mf_type left;
	MATLAB_MF(&left, "left", 223, 224, 228, 252);           // TODO: needs to be tuned
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 248, 259, 259, 270);
	struct mf_type right;
	MATLAB_MF(&right, "right", 266, 305, 309, 310);         // TODO: needs to be tuned


	steering.membership_functions = &right;
	right.next = &straight;
	straight.next = &left;
	left.next = NULL;



	// pointers to top of lists
	//
	System_Inputs = &cPosition;
	cPosition.next = NULL;
	System_Outputs = &steering;
	steering.next = NULL;


	struct rule_type rule1; Rule_Base = &rule1;
	struct rule_type rule2; rule1.next = &rule2;
	struct rule_type rule3; rule2.next = &rule3; rule3.next = NULL;


	// RULE SETUP
	//////////////////////////////////////////////////////////////////////////////

	////RULE 1 "if cPosition is cRight then steering is left"
	struct rule_element_type if11, then1;
	rule1.if_side = &if11; if11.next = NULL; rule1.then_side = &then1; then1.next = NULL;
	if11.value = &cRight.value; then1.value = &left.value;

	////RULE 2 "if cPosition is cLeft then steering is right"
	struct rule_element_type if21, then2;
	rule2.if_side = &if21; if21.next = NULL; rule2.then_side = &then2; then2.next = NULL;
	if21.value = &cLeft.value; then2.value = &right.value;

	////RULE 3 "if cPosition is centre and vOrientation is rightOriented then steering is left"
	struct rule_element_type if31, then3;
	rule3.if_side = &if31; if31.next = NULL; rule3.then_side = &then3; then3.next = NULL;
	if31.value = &centre.value; then3.value = &straight.value;


	// the methods performing the FLC
	fuzzification();
	rule_evaluation();
	defuzzification();
	
	steering.value = steering.value*10;
	if (steering.value < MAXLEFT)
	{
		setServo(MAXLEFT);
	}
	else if (steering.value >MAXRIGHT)
	{
		setServo(MAXRIGHT);
	}
	else
	{
		setServo(steering.value);
	}

}


void intersection(int gyro, unsigned char type, int c, int v)
{
	if (gyro<0)
	{
		gyro = -gyro;
	}
	else if (gyro > maxGyro)
	{
		gyro = maxGyro;
	}
	
	
	if (type == 'r')
	{
		// rightTurn(gyro);
		if (gyro<3)
		{
			setServo(MAXRIGHT-300);
		}
		else
		{
			setServo(MAXRIGHT-150); // maxright-180
		}
	}
	else if (type == 'l')
	{
		// leftTurn(gyro);               // original plan
		if (gyro < 20)                  // hard coded
		{
			setServo(STRAIGHT-200);
		}
		else
		{
			setServo(MAXLEFT-300);
		}
	}
	else if (type == 'F')
	{
		if (v==81 & c == 2)             // State "no left side"
		{
			setServo(STRAIGHT-350);
		}
		else
		{
			straightIntersection(c);
		}
		
	}
}
