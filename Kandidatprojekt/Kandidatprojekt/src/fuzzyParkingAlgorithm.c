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


#define MIN_DISTANCE 0
#define MAX_DISTANCE 250
#define PARK_SPEED_FORWARD 2730
#define PARK_SPEED_REVERSE 2790

int readyToPark = 0;
int parkingFinished = 0;


// prototypes

void gettingReady(int sonicR, int sonicF);



void gettingReady(int sonicR, int sonicF)
{
    
    
    ////////////////////////////////////////////////////////////////////////////////
    ///// SETUP FOR INPUT VARIABLE DISTF ///////////////////////////////////////////
    
    // 1. Declaration
    
    struct io_type distF; strcpy(distF.name, "distF");
    
    // 2. Set MFs
    
    struct mf_type smallF;
    MATLAB_MF(&smallF, "smallF", -1, 0, 0, 20);
    struct mf_type mediumF;
    MATLAB_MF(&mediumF, "mediumF", 10, 30, 30, 50);
    struct mf_type bigF;
    MATLAB_MF(&bigF, "bigF", 40, 60, 60, 80);
    
    // 3. Linked list for MFs
    
    distF.membership_functions = &smallF;
    smallF.next = &mediumF;
    mediumF.next = &bigF;
    bigF.next = NULL;
    
    // 4. Variable assigned its reference value
    
    if(sonicF < MIN_DISTANCE)
    {
        distF.value = MIN_DISTANCE;
    }
    else if (sonicF > MAX_DISTANCE)
    {
        distF.value = MAX_DISTANCE;
    }
    else
    {
        distF.value = sonicF;
    }
    
    
    
    ////////////////////////////////////////////////////////////////////////////////
    ///// SETUP FOR INPUT VARIABLE DISTR ///////////////////////////////////////////
    
    // 1. Declaration
    
    struct io_type distR; strcpy(distR.name, "distR");
    
    // 2. Set MFs
    
    struct mf_type smallR;
    MATLAB_MF(&smallR, "smallR", -1, 0, 0, 20);
    struct mf_type mediumR;
    MATLAB_MF(&mediumR, "mediumR", 10, 30, 30, 50);
    struct mf_type bigR;
    MATLAB_MF(&bigR, "bigR", 40, 60, 60, 80);
    
    // 3. Linked list for MFs
    
    distR.membership_functions = &smallR;
    smallR.next = &mediumR;
    mediumR.next = &bigR;
    bigR.next = NULL;
    
    // 4. Variable assigned its reference value
    
    if(sonicR < MIN_DISTANCE)
    {
        distR.value = MIN_DISTANCE;
    }
    else if (sonicR > MAX_DISTANCE)
    {
        distR.value = MAX_DISTANCE;
    }
    else
    {
        distR.value = sonicR;
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////
    ////// SETUP FOR OUTPUT VARIABLE  SERVO ////////////////////////////////////////
    
    // 1. Declaration
    
    struct io_type servo; strcpy(servo.name, "servo");
    
    // 2. Set MFs
    struct mf_type right;
    MATLAB_MF(&right, "right", 2739, 2740, 2740, 2815);
    struct mf_type straight;
    MATLAB_MF(&straight, "straight", 2804, 2820, 2820, 2836);
    struct mf_type left;
    MATLAB_MF(&left, "left", 2827, 2843, 2843, 2859);
    
    
    // 3. Linked list for MFs
    servo.membership_functions = &right;
    right.next = &straight;
    straight.next = &left;
    left.next = NULL;
    
    
    
    ////////////////////////////////////////////////////////////////////////////////
    ////// LINKED LISTS FOR VARIABLES //////////////////////////////////////////////
    
    System_Inputs = &distR;
    distR.next = &distF;
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
    if11.value = &smallR.value; if12.value = &smallF.value; then1.value = &right.value;
    
    struct rule_element_type if21, if22, then2;
    rule2.if_side = &if21; if21.next = &if22; if22.next = NULL; rule2.then_side = &then2; then2.next = NULL;
    if21.value = &smallR.value; if22.value = &mediumF.value; then2.value = &right.value;
    
    struct rule_element_type if31, if32, then3;
    rule3.if_side = &if31; if31.next = &if32; if32.next = NULL; rule3.then_side = &then3; then3.next = NULL;
    if31.value = &smallR.value; if32.value = &bigF.value; then3.value = &straight.value;
    
    struct rule_element_type if41, if42, then4;
    rule4.if_side = &if41; if41.next = &if42; if42.next = NULL; rule4.then_side = &then4; then4.next = NULL;
    if41.value = &mediumR.value; if42.value = &smallF.value; then4.value = &left.value;
    
    struct rule_element_type if51, if52, then5;
    rule5.if_side = &if51; if51.next = &if52; if52.next = NULL; rule5.then_side = &then5; then5.next = NULL;
    if51.value = &mediumR.value; if52.value = &mediumF.value; then5.value = &right.value;
    
    struct rule_element_type if61, if62, then6;
    rule6.if_side = &if61; if61.next = &if62; if62.next = NULL; rule6.then_side = &then6; then6.next = NULL;
    if61.value = &mediumR.value; if62.value = &bigF.value; then6.value = &straight.value;
    
    struct rule_element_type if71, if72, then7;
    rule7.if_side = &if71; if71.next = &if72; if72.next = NULL; rule7.then_side = &then7; then7.next = NULL;
    if71.value = &bigR.value; if72.value = &smallF.value; then7.value = &left.value;
    
    struct rule_element_type if81, if82, then8;
    rule8.if_side = &if81; if81.next = &if82; if82.next = NULL; rule8.then_side = &then8; then8.next = NULL;
    if81.value = &bigR.value; if82.value = &mediumF.value; then8.value = &right.value;
    
    struct rule_element_type if91, if92, then9;
    rule9.if_side = &if91; if91.next = &if92; if92.next = NULL; rule9.then_side = &then9; then9.next = NULL;
    if91.value = &bigR.value; if92.value = &bigF.value; then9.value = &right.value;
    
    
    // the methods performing the FLC
    fuzzification();
    rule_evaluation();
    defuzzification();
    //  setServo(servo.value);
    
    
    
    
    
    
    
    
    
    
    
}






void fuzzy_parking(int sonicRight, int sonicForward, int speedCount)
{
    
    if (sonicForward >50)
    {
		setESC(2830);
    }
	if (sonicForward<10)
	{
		setESC(2680);
	}
    
    if (readyToPark == 0)
    {
        gettingReady(sonicRight, sonicForward);
    }
    
    if (readyToPark == 1)
    {
        // do parking routine
        
        
    }
    
    
    
}

