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


void rightTurn(int gyro);
void leftTurn(int gyro);
void straightIntersection(int c1, int v1);




void rightTurn(int gyro)
{
	// DECLARATION OF GYRO INPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////
	
	struct io_type turn; strcpy(turn.name, "turn");
	
	// Set MFs
	struct mf_type start;
	MATLAB_MF(&start, "start", minGyro-1, minGyro, minGyro, 45);
	struct mf_type middle;
	MATLAB_MF(&middle, "middle", 30, 45, 45, 60);
	struct mf_type end;
	MATLAB_MF(&end, "end", 45, 45, maxGyro, maxGyro+1);

	// Linked list for MFs
	turn.membership_functions = &start;
	start.next = &middle;
	middle.next = &end;
	end.next = NULL;
	
	
	// DECLARATION OF SERVO OUTPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////
	
	struct io_type servo; strcpy(servo.name, "servo");
	
	// Set MFs
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 2739, 2740, 2740, 2815);
	struct mf_type right;
	MATLAB_MF(&right, "right", 2804, 2820, 2820, 2836);
	struct mf_type sharpRight;
	MATLAB_MF(&sharpRight, "sharpRight", MAXRIGHT-1, MAXRIGHT, MAXRIGHT, MAXRIGHT+1);

	// Linked list for MFs
	servo.membership_functions = &straight;
	straight.next = &right;
	right.next = &sharpRight;
	sharpRight.next = NULL;
	
	
	// SETTING I/O LINKED LISTS
	//////////////////////////////////////////////////////////////////////////////
	
	System_Inputs = &turn;
	turn.next = NULL;
	
	System_Outputs = &servo;
	servo.next = NULL;
	
	
	
	// DECLARATION OF RULES AND LISTS
	//////////////////////////////////////////////////////////////////////////////
	
	struct rule_type rule1; Rule_Base = &rule1;
	struct rule_type rule2; rule1.next = &rule2;
	struct rule_type rule3; rule2.next = &rule3; rule3.next = NULL;
	
	
	// RULE SETUP
	//////////////////////////////////////////////////////////////////////////////
	
	// if gyro is start then servo is noSpeed
	struct rule_element_type if11, then1;
	rule1.if_side = &if11; if11.next = NULL; rule1.then_side = &then1; then1.next = NULL;
	if11.value = &start.value; then1.value = &sharpRight.value;
	
	// if speed is low and distance is oneM then speed is slow    struct rule_element_type if21, if22, then2;
	struct rule_element_type if21, then2;
	rule2.if_side = &if21; if21.next = NULL; rule2.then_side = &then2; then2.next = NULL;
	if21.value = &middle.value; then2.value = &right.value;
	
	// if speed is medium and distance is oneM then speed is cruise
	struct rule_element_type if31, then3;
	rule3.if_side = &if31; if31.next = NULL; rule3.then_side = &then3; then3.next = NULL;
	if31.value = &end.value; then3.value = &straight.value;

	
	
	// EXECUTING FUZZY LOGIC & OUTPUT VALUE
	//////////////////////////////////////////////////////////////////////////////
	
	fuzzification();
	rule_evaluation();
	defuzzification();
	if (servo.value > MAXLEFT)
	{
		setServo(MAXLEFT);
	}
	else if (servo.value < MAXRIGHT)
	{
		setServo(MAXRIGHT);
	}
	else
	{
		setServo(servo.value);
	}
}

/*
* This function deals with left turns
*/
void leftTurn(int gyro)
{
	
	// DECLARATION OF GYRO INPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////
	
	struct io_type turn; strcpy(turn.name, "turn");
	
	// Set MFs
	struct mf_type start;
	MATLAB_MF(&start, "start", minGyro-1, minGyro, minGyro, 45);
	struct mf_type middle;
	MATLAB_MF(&middle, "middle", 30, 45, 45, 60);
	struct mf_type end;
	MATLAB_MF(&end, "end", 45, 45, maxGyro, maxGyro+1);

	// Linked list for MFs
	turn.membership_functions = &start;
	start.next = &middle;
	middle.next = &end;
	end.next = NULL;
	
	
	// DECLARATION OF SERVO OUTPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////
	
	struct io_type servo; strcpy(servo.name, "servo");
	
	// Set MFs
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 2460, 2560, 2560, 2660);
	struct mf_type left;
	MATLAB_MF(&left, "left", 2240, 2240, 2400, 2560);
	struct mf_type sharpLeft;
	MATLAB_MF(&sharpLeft, "sharpLeft", MAXLEFT-1, MAXLEFT, MAXLEFT, MAXLEFT+1);

	// Linked list for MFs
	servo.membership_functions = &straight;
	straight.next = &left;
	left.next = &sharpLeft;
	sharpLeft.next = NULL;
	
	
	// SETTING I/O LINKED LISTS
	//////////////////////////////////////////////////////////////////////////////
	
	System_Inputs = &turn;
	turn.next = NULL;
	
	System_Outputs = &servo;
	servo.next = NULL;
	
	
	
	// DECLARATION OF RULES AND LISTS
	//////////////////////////////////////////////////////////////////////////////
	
	struct rule_type rule1; Rule_Base = &rule1;
	struct rule_type rule2; rule1.next = &rule2;
	struct rule_type rule3; rule2.next = &rule3; rule3.next = NULL;
	
	
	// RULE SETUP
	//////////////////////////////////////////////////////////////////////////////
	
	// if gyro is start then servo is noSpeed
	struct rule_element_type if11, then1;
	rule1.if_side = &if11; if11.next = NULL; rule1.then_side = &then1; then1.next = NULL;
	if11.value = &start.value; then1.value = &left.value;
	
	// if speed is low and distance is oneM then speed is slow    struct rule_element_type if21, if22, then2;
	struct rule_element_type if21, then2;
	rule2.if_side = &if21; if21.next = NULL; rule2.then_side = &then2; then2.next = NULL;
	if21.value = &middle.value; then2.value = &sharpLeft.value;
	
	// if speed is medium and distance is oneM then speed is cruise
	struct rule_element_type if31, then3;
	rule3.if_side = &if31; if31.next = NULL; rule3.then_side = &then3; then3.next = NULL;
	if31.value = &end.value; then3.value = &sharpLeft.value;

	
	
	// EXECUTING FUZZY LOGIC & OUTPUT VALUE
	//////////////////////////////////////////////////////////////////////////////
	
	fuzzification();
	rule_evaluation();
	defuzzification();
	if (servo.value > MAXLEFT)
	{
		setServo(MAXLEFT);
	}
	else if (servo.value < MAXRIGHT)
	{
		setServo(MAXRIGHT);
	}
	else
	{
		setServo(servo.value);
	}
	
}


void straightIntersection(int c1, int v1)
{
	///// DECLARATION OF C INPUT VARIABLE ///////////////////////////////////

	struct io_type cPosition; strcpy(cPosition.name, "cPosition");

	struct mf_type cRight;
	MATLAB_MF(&cRight, "cRight", 159, 160, 170, 220); // Min_value = 160
	struct mf_type centre;
	MATLAB_MF(&centre, "centre", 170, 220, 250, 360);
	struct mf_type cLeft;
	MATLAB_MF(&cLeft, "cLeft", 250, 360, 370, 371); // Max_value = 370


	cPosition.membership_functions = &cRight;
	cRight.next = &centre;
	centre.next = &cLeft;
	cLeft.next = NULL;

	// set iErr's input value to measErr value
	if(c1<100)				// if sensor value is smaller than cPosition's input set's lower limit
	{
		cPosition.value = 160;  // force input value to lowest point in cPosition's input set
	}
	else if(c1>200)			// if sensor value is bigger than cPosition's input set's upper limit
	{
		cPosition.value = 371;  // force input value to lowest point in cPosition's input set
	}
	else
	{
		cPosition.value = c1;
	}

	///// DECLARATION OF V INPUT VARIABLE ///////////////////////////////////

	struct io_type vOrientation; strcpy(vOrientation.name, "vOrienta");

	struct mf_type leftOriented;
	MATLAB_MF(&leftOriented, "leftOri", 44, 45, 46, 48); // min V is 45

	struct mf_type straightOriented;
	MATLAB_MF(&straightOriented, "straOri", 46, 48, 50, 56);

	struct mf_type rightOriented;
	MATLAB_MF(&rightOriented, "rightOri", 50, 56, 57, 58); // max V is 57

	vOrientation.membership_functions = &leftOriented;
	leftOriented.next = &straightOriented;
	straightOriented.next = &rightOriented;
	rightOriented.next = NULL;

	// set V's input value to V´s value
	if(v1<=45)				// if sensor value is smaller than error's input set lower limit
	{
		vOrientation.value = 45; // force input value to lowest point in vOrientation's input set
	}
	else if(v1>=57)			// if sensor value is bigger than error's input set's upper limit
	{
		vOrientation.value = 57;  // force input value to lowest point in error's input set
	}
	else
	{
		vOrientation.value = v1;
	}

	///// DECLARATION OF STEERING OUTPUT VARIABLE ///////////////////////////////////

	struct io_type steering; strcpy(steering.name, "steering"); // All outputs downscaled by a factor 10

	struct mf_type left;
	MATLAB_MF(&left, "left", 223, 224, 228, 262);
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 228, 262, 270, 305);
	struct mf_type right;
	MATLAB_MF(&right, "right", 270, 305, 309, 310);


	steering.membership_functions = &right;
	right.next = &straight;
	straight.next = &left;
	left.next = NULL;



	// pointers to top of lists

	System_Inputs = &cPosition;
	cPosition.next = &vOrientation;
	vOrientation.next = NULL;
	System_Outputs = &steering;
	steering.next = NULL;


	struct rule_type rule1; Rule_Base = &rule1;
	struct rule_type rule2; rule1.next = &rule2;
	struct rule_type rule3; rule2.next = &rule3;
	struct rule_type rule4; rule3.next = &rule4;
	struct rule_type rule5; rule4.next = &rule5;
	rule5.next = NULL;


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

	////RULE 5 "if cPosition is centre and vOrientation is rightOriented then steering is left"
	struct rule_element_type if31, if32, then3;
	rule3.if_side = &if31; if31.next = &if32; if32.next = NULL; rule3.then_side = &then3; then3.next = NULL;
	if31.value = &centre.value; if32.value = &rightOriented.value; then3.value = &left.value;

	////RULE 6 "if cPosition is centre and vOrientation is straightOriented then steering is straight"
	struct rule_element_type if41, if42, then4;
	rule4.if_side = &if41; if41.next = &if42; if42.next = NULL; rule4.then_side = &then4; then4.next = NULL;
	if41.value = &centre.value; if42.value = &straightOriented.value; then4.value = &straight.value;

	////RULE 5 "if cPosition is centre and vPosition is leftOriented then steering is right"
	struct rule_element_type if51, if52, then5;
	rule5.if_side = &if51; if51.next = &if52; if52.next = NULL; rule5.then_side = &then5; then5.next = NULL;
	if51.value = &centre.value; if52.value = &leftOriented.value; then5.value = &right.value;




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
		setServo(MAXRIGHT-150);
	}
	else if (type == 'l')
	{
		leftTurn(gyro);
	}
	else if (type == 'F')
	{
		if (v==81)
		{
			setServo(MAXLEFT+200);
		}
		else
		{
			straightIntersection(c,v);	
		}
		
	}
}
