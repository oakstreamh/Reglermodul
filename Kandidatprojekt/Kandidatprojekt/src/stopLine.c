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




void stop(int k_value)
{
	

	
	// DECLARATION OF GYRO INPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////
	
	struct io_type orientation; strcpy(orientation.name, "orientatio");
	
	if (k_value < -20)
	{
		orientation.value = -20;
	}
	else if (k_value > 20)
	{
		orientation.value = 20;
	}
	else
	{
		orientation.value = k_value;
	}
	
	// Set MFs
	struct mf_type off_right;
	MATLAB_MF(&off_right, "off_right", -21, -20, -20, -5);
	struct mf_type middle;
	MATLAB_MF(&middle, "middle", -10, 0, 0, 10);
	struct mf_type off_left;
	MATLAB_MF(&off_left, "off_left", 5, 20, 20, 21);

	// Linked list for MFs
	orientation.membership_functions = &off_right;
	off_right.next = &middle;
	middle.next = &off_left;
	off_left.next = NULL;
	
	
	// DECLARATION OF SERVO OUTPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////
	
	struct io_type servo; strcpy(servo.name, "servo");
	
	// Set MFs
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 2600, 2660, 2660, 2720);
	struct mf_type right;
	MATLAB_MF(&right, "right", 2800, 2820, 2820, 2840);
	struct mf_type left;
	MATLAB_MF(&left, "left", 2480, 2500, 2500, 2520);

	// Linked list for MFs
	servo.membership_functions = &straight;
	straight.next = &left;
	left.next = &right;
	right.next = NULL;
	
	
	// SETTING I/O LINKED LISTS
	//////////////////////////////////////////////////////////////////////////////
	
	System_Inputs = &orientation;
	orientation.next = NULL;
	
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
	if11.value = &middle.value; then1.value = &straight.value;
	
	// if speed is low and distance is oneM then speed is slow    struct rule_element_type if21, if22, then2;
	struct rule_element_type if21, then2;
	rule2.if_side = &if21; if21.next = NULL; rule2.then_side = &then2; then2.next = NULL;
	if21.value = &off_right.value; then2.value = &left.value;
	
	// if speed is medium and distance is oneM then speed is cruise
	struct rule_element_type if31, then3;
	rule3.if_side = &if31; if31.next = NULL; rule3.then_side = &then3; then3.next = NULL;
	if31.value = &off_left.value; then3.value = &right.value;

	
	
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