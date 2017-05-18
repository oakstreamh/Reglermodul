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

// DEFINITIONS
//////////////////////////////////////////////////////////////////////////////////////

#define maxGyro 125
#define minGyro 0


void rightTurn(int gyro);
void leftTurn(int gyro);



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
    if11.value = &start.value; then1.value = &sharpLeft.value;
    
    // if speed is low and distance is oneM then speed is slow    struct rule_element_type if21, if22, then2;
    struct rule_element_type if21, then2;
    rule2.if_side = &if21; if21.next = NULL; rule2.then_side = &then2; then2.next = NULL;
    if21.value = &middle.value; then2.value = &left.value;
    
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


void intersection(int gyro, int type, int c, int v)
{
	
	if (gyro<0)
	{
		gyro = -gyro; 
	}
	
	
    // INPUT CHECK
    //////////////////////////////////////////////////////////////////////////////////
    
    if (gyro > maxGyro)
    {
        gyro = maxGyro;
    }
    
    
    if (type == 'r')
    {
		setServo(MAXRIGHT);
    }
	else if (type == 'l')
	{
		if (checkCount(500) == 0) // if count less than 500 ms then, keep straight
		{
			nFuzzySteering(c,v);
		}
		else
		{
		count(0);
		leftTurn(gyro);	
		}
	}
}
