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




void gettingReady(int sonicL, int sonicF)
{
    
    
    ////////////////////////////////////////////////////////////////////////////////
    ///// SETUP FOR INPUT VARIABLE DISTF ///////////////////////////////////////////
    
    // 1. Declaration
    
    struct io_type distF; strcpy(distF.name, "distF");
    
    // 2. Set MFs
    
    struct mf_type smallF;
    MATLAB_MF(&smallF, "smallF", MIN_DIST-1, MIN_DIST, 0, 20);
    struct mf_type mediumF;
    MATLAB_MF(&mediumF, "mediumF", 10, 40, 40, 60);
    struct mf_type bigF;
    MATLAB_MF(&bigF, "bigF", 40, 70, MAX_FRONT, MAX_FRONT+1);
    
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
    MATLAB_MF(&smallL, "smallL", MIN_DIST-1, MIN_DIST, MIN_DIST, 20);
    struct mf_type mediumL;
    MATLAB_MF(&mediumL, "mediumL", 8, 12, 14, 18);
    struct mf_type bigL;
    MATLAB_MF(&bigL, "bigL", 40, 60, MAX_RIGHT, MAX_RIGHT+1);
    
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
    struct mf_type right;
    MATLAB_MF(&right, "right", 2185, 2250, 2250, 2400);
    struct mf_type straight;
    MATLAB_MF(&straight, "straight", 2532, 2660, 2660, 2788);
    struct mf_type left;
    MATLAB_MF(&left, "left", 3000, 3100, 3100, MAXLEFT);
  
    
    // 3. Linked list for MFs
    servo.membership_functions = &right;
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
    struct rule_type rule6;
    struct rule_type rule7;
    struct rule_type rule8;
    struct rule_type rule9;
    
    
    // 2. Linked lists for rule_types
    
    Rule_Base = &rule1;
    rule1.next = &rule2;
    rule2.next = &rule3;
    rule3.next = &rule4;
    rule4.next = &rule5;
    rule5.next = &rule6;
    rule6.next = &rule7;
    rule7.next = &rule8;
    rule8.next = &rule9;
    rule9.next = NULL;
    
    // 3. Declaration of rule_element_types
    
    struct rule_element_type if11, if12, then1;
    rule1.if_side = &if11; if11.next = &if12; if12.next = NULL; rule1.then_side = &then1; then1.next = NULL;
    if11.value = &smallL.value; if12.value = &smallF.value; then1.value = &right.value;
    
    struct rule_element_type if21, if22, then2;
    rule2.if_side = &if21; if21.next = &if22; if22.next = NULL; rule2.then_side = &then2; then2.next = NULL;
    if21.value = &smallL.value; if22.value = &mediumF.value; then2.value = &right.value;
    
    struct rule_element_type if31, if32, then3;
    rule3.if_side = &if31; if31.next = &if32; if32.next = NULL; rule3.then_side = &then3; then3.next = NULL;
    if31.value = &smallL.value; if32.value = &bigF.value; then3.value = &straight.value;
    
    struct rule_element_type if41, if42, then4;
    rule4.if_side = &if41; if41.next = &if42; if42.next = NULL; rule4.then_side = &then4; then4.next = NULL;
    if41.value = &mediumL.value; if42.value = &smallF.value; then4.value = &left.value;
    
    struct rule_element_type if51, if52, then5;
    rule5.if_side = &if51; if51.next = &if52; if52.next = NULL; rule5.then_side = &then5; then5.next = NULL;
    if51.value = &mediumL.value; if52.value = &mediumF.value; then5.value = &right.value;
    
    struct rule_element_type if61, if62, then6;
    rule6.if_side = &if61; if61.next = &if62; if62.next = NULL; rule6.then_side = &then6; then6.next = NULL;
    if61.value = &mediumL.value; if62.value = &bigF.value; then6.value = &straight.value;
    
    struct rule_element_type if71, if72, then7;
    rule7.if_side = &if71; if71.next = &if72; if72.next = NULL; rule7.then_side = &then7; then7.next = NULL;
    if71.value = &bigL.value; if72.value = &smallF.value; then7.value = &left.value;
    
    struct rule_element_type if81, if82, then8;
    rule8.if_side = &if81; if81.next = &if82; if82.next = NULL; rule8.then_side = &then8; then8.next = NULL;
    if81.value = &bigL.value; if82.value = &mediumF.value; then8.value = &right.value;
    
    struct rule_element_type if91, if92, then9;
    rule9.if_side = &if91; if91.next = &if92; if92.next = NULL; rule9.then_side = &then9; then9.next = NULL;
    if91.value = &bigL.value; if92.value = &bigF.value; then9.value = &right.value;
    
    
    // the methods performing the FLC
    fuzzification();
    rule_evaluation();
    defuzzification();
    setServo(servo.value);
	
	if (sonicL < 15 & sonicF < 40)
	{
		readyToPark = 1;
	}
    
    
    
    
    
    
    
    
    
    
    
}






void fuzzy_parking(int sonicL, int sonicF, int speedCount)
{
    
    if (sonicF >50)
    {
		setESC(2840);
    }
	if (sonicF<10)
	{
		setESC(2650);
	}
    
	if (readyToPark == 0)
    {
		gettingReady(sonicL, sonicL);
    }
    
    if (readyToPark == 1)
    {
		setESC(NEUTRAL);
        // do parking routine
        
        
    }
    
    
    
}

