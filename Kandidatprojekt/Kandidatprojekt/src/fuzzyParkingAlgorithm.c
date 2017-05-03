//
//  parkingAlgorithm.c
//  hemmajobb
//
//  Created by Mathias Dalshagen on 2017-04-27.
//  Copyright © 2017 Mathias Dalshagen. All rights reserved.
//

#include "fuzzyParkingAlgorithm.h"
#include "general_FIS.h"
#include "string.h"
#include "servo.h"


#define MIN_DIST 0
#define MAX_FRONT 100

#define MAX_RIGHT 100

#define PARK_SPEED_FORWARD 2730
#define PARK_SPEED_REVERSE 2790


int readyToPark = 0;
int parkingFinished = 0;


// prototypes


void step1(int sonicL, int sonicF)
{
	
	////////////////////////////////////////////////////////////////////////////////
	///// SETUP FOR INPUT VARIABLE DISTF ///////////////////////////////////////////
	
	// 1. Declaration
	
	struct io_type distF; strcpy(distF.name, "distF");
	
	// 2. Set MFs
	
	struct mf_type smallF;
	MATLAB_MF(&smallF, "smallF", MIN_DIST-1, MIN_DIST, 40, 60);
	struct mf_type mediumF;
	MATLAB_MF(&mediumF, "mediumF", 50, 80, 80, 90);
	struct mf_type bigF;
	MATLAB_MF(&bigF, "bigF", 80, 90, MAX_FRONT, MAX_FRONT+1);
	
	// 3. Linked list for MFs
	
	distF.membership_functions = &smallF;
	smallF.next = &mediumF;
	mediumF.next = &bigF;
	bigF.next = NULL;
	
	// 4. Variable assigned its reference value
	
	if(sonicF < MIN_DIST)
	{
		distF.value = MIN_DIST;
	}
	else if (sonicF > MAX_FRONT)
	{
		distF.value = MAX_FRONT;
	}
	else
	{
		distF.value = sonicF;
	}
	
	
	
	////////////////////////////////////////////////////////////////////////////////
	///// SETUP FOR INPUT VARIABLE distL ///////////////////////////////////////////
	
	// 1. Declaration
	
	struct io_type distL; strcpy(distL.name, "distL");
	
	// 2. Set MFs
	
	struct mf_type smallL;
	MATLAB_MF(&smallL, "smallL", MIN_DIST-1, MIN_DIST, 55, 75);
	struct mf_type mediumL;
	MATLAB_MF(&mediumL, "mediumL", 55, 75, 75, 95);
	struct mf_type bigL;
	MATLAB_MF(&bigL, "bigL", 75, 95, MAX_RIGHT, MAX_RIGHT+1);
	
	// 3. Linked list for MFs
	
	distL.membership_functions = &smallL;
	smallL.next = &mediumL;
	mediumL.next = &bigL;
	bigL.next = NULL;
	
	// 4. Variable assigned its reference value
	
	if(sonicL < MIN_DIST)
	{
		distL.value = MIN_DIST;
	}
	else if (sonicL > MAX_RIGHT)
	{
		distL.value = MAX_RIGHT;
	}
	else
	{
		distL.value = sonicL;
	}
	
	
	////////////////////////////////////////////////////////////////////////////////
	////// SETUP FOR OUTPUT VARIABLE  SERVO ////////////////////////////////////////
	
	// 1. Declaration
	
	struct io_type servo; strcpy(servo.name, "servo");
	
	// 2. Set MFs
	struct mf_type sharpRight;
	MATLAB_MF(&sharpRight, "sharpRight", MAXRIGHT-1, MAXRIGHT, MAXRIGHT, MAXRIGHT+1);
	struct mf_type right;
	MATLAB_MF(&right, "right", 2185, 2250, 2250, 2400);
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 2532, 2660, 2660, 2788);
	struct mf_type left;
	MATLAB_MF(&left, "left", 3000, 3100, 3100, MAXLEFT);
	
	
	// 3. Linked list for MFs
	servo.membership_functions = &sharpRight;
	sharpRight.next = &right;
	right.next = &straight;
	straight.next = &left;
	left.next = NULL;
	
	
	
	////////////////////////////////////////////////////////////////////////////////
	////// LINKED LISTS FOR VARIABLES //////////////////////////////////////////////
	
	System_Inputs = &distL;
	distL.next = &distF;
	distF.next = NULL;
	
	System_Outputs = &servo;
	servo.next = NULL;
	
	
	////////////////////////////////////////////////////////////////////////////////
	////// RULES ///////////////////////////////////////////////////////////////////
	
	// 1. Declaration of rule_types
	
	struct rule_type rule1;
	struct rule_type rule2;
	struct rule_type rule3;
	struct rule_type rule4;
	struct rule_type rule5;

	
	
	// 2. Linked lists for rule_types
	
	Rule_Base = &rule1;
	rule1.next = &rule2;
	rule2.next = &rule3;
	rule3.next = &rule4;
	rule4.next = &rule5;
	rule5.next = NULL;
	
	// 3. Declaration of rule_element_types
	
	struct rule_element_type if11, if12, then1;
	rule1.if_side = &if11; if11.next = &if12; if12.next = NULL; rule1.then_side = &then1; then1.next = NULL;
	if11.value = &bigF.value; if12.value = &smallL.value; then1.value = &right.value;
	
	struct rule_element_type if21, if22, then2;
	rule2.if_side = &if21; if21.next = &if22; if22.next = NULL; rule2.then_side = &then2; then2.next = NULL;
	if21.value = &bigF.value; if22.value = &bigL.value; then2.value = &left.value;
	
	struct rule_element_type if31, if32, then3;
	rule3.if_side = &if31; if31.next = &if32; if32.next = NULL; rule3.then_side = &then3; then3.next = NULL;
	if31.value = &bigF.value; if32.value = &mediumL.value; then3.value = &straight.value;
	
	struct rule_element_type if41, then4;
	rule4.if_side = &if41; if41.next = NULL; rule4.then_side = &then4; then4.next = NULL;
	if41.value = &mediumF.value;  then4.value = &sharpRight.value;
	
	struct rule_element_type if51, then5;
	rule5.if_side = &if51; if51.next = NULL; rule5.then_side = &then5; then5.next = NULL;
	if51.value = &smallF.value;  then5.value = &right.value;
	

	// the methods performing the FLC
	fuzzification();
	rule_evaluation();
	defuzzification();
	setServo(servo.value);

}

void step2(int sonicL, int sonicF)

{
	////////////////////////////////////////////////////////////////////////////////
	///// SETUP FOR INPUT VARIABLE DISTF ///////////////////////////////////////////
	
	// 1. Declaration
	
	struct io_type distF; strcpy(distF.name, "distF");
	
	// 2. Set MFs
	
	struct mf_type smallF;
	MATLAB_MF(&smallF, "smallF", MIN_DIST-1, MIN_DIST, 40, 60);
	struct mf_type mediumF;
	MATLAB_MF(&mediumF, "mediumF", 50, 80, 80, 90);
	struct mf_type bigF;
	MATLAB_MF(&bigF, "bigF", 80, 90, MAX_FRONT, MAX_FRONT+1);
	
	// 3. Linked list for MFs
	
	distF.membership_functions = &smallF;
	smallF.next = &mediumF;
	mediumF.next = &bigF;
	bigF.next = NULL;
	
	// 4. Variable assigned its reference value
	
	if(sonicF < MIN_DIST)
	{
		distF.value = MIN_DIST;
	}
	else if (sonicF > MAX_FRONT)
	{
		distF.value = MAX_FRONT;
	}
	else
	{
		distF.value = sonicF;
	}
	
	
	
	////////////////////////////////////////////////////////////////////////////////
	///// SETUP FOR INPUT VARIABLE distL ///////////////////////////////////////////
	
	// 1. Declaration
	
	struct io_type distL; strcpy(distL.name, "distL");
	
	// 2. Set MFs
	
	struct mf_type smallL;
	MATLAB_MF(&smallL, "smallL", MIN_DIST-1, MIN_DIST, 5, 10);
	struct mf_type mediumL;
	MATLAB_MF(&mediumL, "mediumL", 5, 10, 10, 20);
	struct mf_type bigL;
	MATLAB_MF(&bigL, "bigL", 15, 25, MAX_RIGHT, MAX_RIGHT+1);
	
	// 3. Linked list for MFs
	
	distL.membership_functions = &smallL;
	smallL.next = &mediumL;
	mediumL.next = &bigL;
	bigL.next = NULL;
	
	// 4. Variable assigned its reference value
	
	if(sonicL < MIN_DIST)
	{
		distL.value = MIN_DIST;
	}
	else if (sonicL > MAX_RIGHT)
	{
		distL.value = MAX_RIGHT;
	}
	else
	{
		distL.value = sonicL;
	}
	
	
	////////////////////////////////////////////////////////////////////////////////
	////// SETUP FOR OUTPUT VARIABLE  SERVO ////////////////////////////////////////
	
	// 1. Declaration
	
	struct io_type servo; strcpy(servo.name, "servo");
	
	// 2. Set MFs
	struct mf_type sharpRight;
	MATLAB_MF(&sharpRight, "sharpRight", MAXRIGHT-1, MAXRIGHT, MAXRIGHT, MAXRIGHT+1);
	struct mf_type right;
	MATLAB_MF(&right, "right", 2185, 2250, 2250, 2400);
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 2532, 2660, 2660, 2788);
	struct mf_type left;
	MATLAB_MF(&left, "left", 3000, 3100, 3100, MAXLEFT);
	
	
	// 3. Linked list for MFs
	servo.membership_functions = &sharpRight;
	sharpRight.next = &right;
	right.next = &straight;
	straight.next = &left;
	left.next = NULL;
	
	
	
	////////////////////////////////////////////////////////////////////////////////
	////// LINKED LISTS FOR VARIABLES //////////////////////////////////////////////
	
	System_Inputs = &distL;
	distL.next = &distF;
	distF.next = NULL;
	
	System_Outputs = &servo;
	servo.next = NULL;
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////
	////// RULES ///////////////////////////////////////////////////////////////////
	
	// 1. Declaration of rule_types
	
	struct rule_type rule1;
	struct rule_type rule2;
	struct rule_type rule3;
	struct rule_type rule4;
	struct rule_type rule5;

	
	
	// 2. Linked lists for rule_types
	
	Rule_Base = &rule1;
	rule1.next = &rule2;
	rule2.next = &rule3;
	rule3.next = &rule4;
	rule4.next = &rule5;
	rule5.next = NULL;
	
	// 3. Declaration of rule_element_types
	
	struct rule_element_type if11, if12, then1;
	rule1.if_side = &if11; if11.next = &if12; if12.next = NULL; rule1.then_side = &then1; then1.next = NULL;
	if11.value = &bigF.value; if12.value = &smallL.value; then1.value = &right.value;
	
	struct rule_element_type if21, if22, then2;
	rule2.if_side = &if21; if21.next = &if22; if22.next = NULL; rule2.then_side = &then2; then2.next = NULL;
	if21.value = &bigF.value; if22.value = &bigL.value; then2.value = &left.value;
	
	struct rule_element_type if31, if32, then3;
	rule3.if_side = &if31; if31.next = &if32; if32.next = NULL; rule3.then_side = &then3; then3.next = NULL;
	if31.value = &bigF.value; if32.value = &mediumL.value; then3.value = &straight.value;
	
	struct rule_element_type if41, then4;
	rule4.if_side = &if41; if41.next = NULL; rule4.then_side = &then4; then4.next = NULL;
	if41.value = &mediumF.value;  then4.value = &sharpRight.value;
	
	struct rule_element_type if51, then5;
	rule5.if_side = &if51; if51.next = NULL; rule5.then_side = &then5; then5.next = NULL;
	if51.value = &smallF.value;  then5.value = &right.value;
	

	// the methods performing the FLC
	fuzzification();
	rule_evaluation();
	defuzzification();
	setServo(servo.value);
	
	if (sonicF < 20)
	{
		readyToPark = 1;
	}
	
	
}







void fuzzyParking(int sonicL, int sonicF, int escCount)
{
	
	if (sonicF >50)
	{
		setESC(2840);
	}
	if (sonicF<10)
	{
		setESC(2640);
	}
	
	if (!readyToPark)
	{
		step1(sonicL, sonicL);
	}
	
	if (readyToPark)
	{
		setESC(NEUTRAL);
		// do parking routine
		
		
	}
	
	
	
}

