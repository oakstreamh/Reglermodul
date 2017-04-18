﻿//////////////////////////////////////////////////////////////////////////////////
// FUZZY SPEED CONTROLLER                                                       //
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
// INCLUDES                                                                     //
//////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include "fuzzySteering.h"
#include "general_FIS.h"
#include <stdio.h>
#include "servo.h"


//////////////////////////////////////////////////////////////////////////////////
// DEFINITIONS OF I/O AND POINTERS TO TOP OF LISTS                              //
//////////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////////
// METHODS                                                                      //
//////////////////////////////////////////////////////////////////////////////////


/* Initialization and execution of fuzzy speed controller.
 * Only top of linked lists are global variables. The controller can
 * be tuned in the MATLAB FLC-tool. The file name is "fuzzy_speed_controller.fis".
 *
 * In the fuzzy logic tool used in this application, trapezoidal functions are defined
 * by the two end points  of membership, [a, d], and two slopes.
 * In MATLAB trapezoidal membership functions (MFs) are defined by four points
 * [a,b,c,d] according to:
 *
 *        b__________c
 *        /          \
 * _____a/............\d________
 *
 * Inputs: measurement of speed, v (PWM counter), and distance, d, from sonic sensors
 * Output: speed, (PWM counter)
 */
void FLC_steering(a, e)
int a;
int e;
{
 
 
    
	// Declaration of I/Os
	struct io_type error;		// input 1
	struct io_type angle;       // input 2
	struct io_type servo;         // output
	struct rule_type rule1;		// first rule in rule base
    strcpy(error.name, "error");
    strcpy(angle.name, "angle");
    strcpy(servo.name, "servo");
	
	error.value = e;
	angle.value = a;
	
	
    /* MFS FOR THE ANGLE INPUT VARIABLE
     *
     * From MATLAB-file
     * anlge variable interval: [2260 3060]
     *
	 * MF shRight:				[2259 2260 2270 2370]
	 * MF right					[2270 2370 2380 2480]
     * MF slRight:				[2400 2500 2550 2650]
     * MF straight:				[2540 2640 2680 2780]
     * MF SlLeft:				[2670 2770 2820 2920]
     * MF left:				    [2840 2940 2950 3050]
	 * MF shLeft:				[2950 3050 3060 3061]
     */
    struct mf_type shLeft;
    strcpy(shLeft.name, "shLeft");
    shLeft.value = 0;
    shLeft.point1 = 2950;
    shLeft.point2 = 3061;
    shLeft.slope1 = 1;
    shLeft.slope2 = 100;
    shLeft.next = NULL;
    
    struct mf_type left;
    strcpy(left.name, "left");
    left.value = 0;
    left.point1 = 2840;
    left.point2 = 3050;
    left.slope1 = 1;
    left.slope2 = 1;
    left.next = &shLeft;
    
    
    struct mf_type slLeft;
    strcpy(slLeft.name, "slLeft");
    slLeft.value = 0;
    slLeft.point1 = 2670;
    slLeft.point2 = 2920;
    slLeft.slope1 = 1;
    slLeft.slope2 = 1;
    slLeft.next = &left;
    
    
    struct mf_type straight;
    strcpy(straight.name, "straight");
    straight.value = 0;
    straight.point1 = 2540;
    straight.point2 = 2780;
    straight.slope1 = 1;
    straight.slope2 = 1;
    straight.next = &slLeft;
	
	struct mf_type slRight;
	strcpy(slRight.name, "slRight");
	slRight.value = 0;
	slRight.point1 = 2400;
	slRight.point2 = 2650;
	slRight.slope1 = 1;
	slRight.slope2 = 1;
	slRight.next = &straight;
	
	struct mf_type right;
	strcpy(right.name, "right");
	right.value = 0;
	right.point1 = 2270;
	right.point2 = 2480;
	right.slope1 = 1;
	right.slope2 = 1;
	right.next = &slRight;
	
	struct mf_type shRight;
	strcpy(shRight.name, "shRight");
	shRight.value = 0;
	shRight.point1 = 2259;
	shRight.point2 = 2370;
	shRight.slope1 = 100;
	shRight.slope2 = 1;
	shRight.next = &right;
    
    
    angle.membership_functions = &shRight;
    angle.next = NULL;
    
    
    /* MFS FOR THE ERROR INPUT VARIABLE:
     *
     * From MATLAB-file
     * speed variable interval: [2750 2930]
     *
	 * MF farLeft:	[215 50 2765 2970]
     * MF low:		[2765 2790 2810 2835]
	 * MF cruising:	[2800 2825 2855 2880]
     * MF medium:	[2845 2878 2892 2925]
     * MF high:		[2890 2900 2930 2931]
     *
     */
    struct mf_type farLeft;
    strcpy(farLeft.name, "farLeft");
    farLeft.value = 0;
    farLeft.point1 = 215;
    farLeft.point2 = 251;
    farLeft.slope1 = 4;
    farLeft.slope2 = 100;
    farLeft.next = NULL;
    
    struct mf_type eLeft;
    strcpy(eLeft.name, "eLeft");
    eLeft.value = 0;
    eLeft.point1 = 130;
    eLeft.point2 = 240;
    eLeft.slope1 = 2;
    eLeft.slope2 = 2;
    eLeft.next = &farLeft;
    
	struct mf_type center;
	strcpy(center.name, "center");
	center.value = 0;
	center.point1 = 65;
	center.point2= 185;
	center.slope1 = 2;
	center.slope2 = 2;
	center.next = &eLeft;
	
	struct mf_type eRight;
	strcpy(eRight.name, "eRight");
	eRight.value = 0;
	eRight.point1 = 10;
	eRight.point2= 120;
	eRight.slope1 = 2;
	eRight.slope2 = 2;
	eRight.next = &center;
	
	
    struct mf_type farRight;
    strcpy(farRight.name, "farRight");
    farRight.value = 0;
    farRight.point1 = -1;
    farRight.point2= 35;
    farRight.slope1 = 100;
    farRight.slope2 = 4;
    farRight.next = &eRight;
    
    error.membership_functions = &farRight;
    error.next = &angle;
    
    
    /* MFS FOR THE SPEED OUTPUT VARIABLE:
     *
     * From MATLAB
     * speed interval [2750 2930]
     *
     * noSpeed		[2749 2750 2765 2790]
	 * slow			[2765 2790 2811 2836] 
     * cruise		[2800 2833 2847 2880]
     * medHigh		[2844 2880 2890 2926]
     * max			[2890 2926 2934 2970]
	 
     *
     */
       struct mf_type oShLeft;
       strcpy(oShLeft.name, "oShLeft");
       oShLeft.value = 0;
       oShLeft.point1 = 2950;
       oShLeft.point2 = 3061;
       oShLeft.slope1 = 1;
       oShLeft.slope2 = 100;
       oShLeft.next = NULL;
       
       struct mf_type oLeft;
       strcpy(oLeft.name, "oLeft");
       oLeft.value = 0;
       oLeft.point1 = 2840;
       oLeft.point2 = 3050;
       oLeft.slope1 = 1;
       oLeft.slope2 = 1;
       oLeft.next = &oShLeft;
       
       
       struct mf_type oSlLeft;
       strcpy(oSlLeft.name, "oSlLeft");
       oSlLeft.value = 0;
       oSlLeft.point1 = 2670;
       oSlLeft.point2 = 2920;
       oSlLeft.slope1 = 1;
       oSlLeft.slope2 = 1;
       oSlLeft.next = &oLeft;
       
       
       struct mf_type oStraight;
       strcpy(oStraight.name, "oStraight");
       oStraight.value = 0;
       oStraight.point1 = 2540;
       oStraight.point2 = 2780;
       oStraight.slope1 = 1;
       oStraight.slope2 = 1;
       oStraight.next = &oSlLeft;
       
       struct mf_type oSlRight;
       strcpy(oSlRight.name, "oSlRight");
       oSlRight.value = 0;
       oSlRight.point1 = 2400;
       oSlRight.point2 = 2650;
       oSlRight.slope1 = 1;
       oSlRight.slope2 = 1;
       oSlRight.next = &oStraight;
       
       struct mf_type oRight;
       strcpy(oRight.name, "oRight");
       oRight.value = 0;
       oRight.point1 = 2270;
       oRight.point2 = 2480;
       oRight.slope1 = 1;
       oRight.slope2 = 1;
       oRight.next = &oSlRight;
       
       struct mf_type oShright;
       strcpy(oShright.name, "oShright");
       oShright.value = 0;
       oShright.point1 = 2259;
       oShright.point2 = 2370;
       oShright.slope1 = 100;
       oShright.slope2 = 1;
       oShright.next = &oRight;
	   
	   angle.membership_functions = &oShright;
	   angle.next = NULL;
    
    /* THE RULE BASE
     *
     * From MATLAB:
     *
     * #1 IF distance is "stopDist" THEN speed is "noSpeed"
     * #2 IF speed is "low" AND distance is "oneM" THEN speed is "slow"
     * #3 IF speed is "cruising" AND distance is "oneM" THEN speed is "slow"
     * #4 IF speed is "medium" AND distance is "oneM" THEN speed is "cruise"
     * #5 IF speed is "high" AND distance is "oneM" THEN speed is "medHigh"
     * #6 IF speed is "low" AND distance is "twoM" THEN speed is "cruise"
     * #7 IF speed is "cruising" AND distance is "twoM" THEN speed is "medHigh"
	 * #8 IF speed is "high" AND distance is "twoM" THEN speed is "medHigh"
	 * #9 IF distance is "threeM" THEN speed is "max"
	 * #10 IF speed is "low" AND distance is "twoM" THEN speed is "cruise"
	 * #11 IF speed is "still" AND distance is "oneM" THEN speed is "slow"
     *
     */
	
	/* rule # 11: if eLeft and straight then slRight */
	struct rule_element_type then11;
	then11.value = &slRight.value;
	then11.next = NULL;
	
	struct rule_element_type if112;
	if112.value = &straight.value;
	if112.next = NULL;
	
	struct rule_element_type if111;
	if111.value = &eLeft.value;
	if111.next = &if112;
	
	struct rule_type rule11;
	rule11.if_side = &if111;
	rule11.then_side = &then11;
	rule11.next = NULL;
	
	/* rule # 10: if eLeft and slRight then oslRight */
	struct rule_element_type then10;
	then10.value = &oSlRight.value;
	then10.next = NULL;
	
	struct rule_element_type if102;
	if102.value = &slRight.value;
	if102.next = NULL;
	
	struct rule_element_type if101;
	if101.value = &eLeft.value;
	if101.next = &if102;
	
	struct rule_type rule10;
	rule10.if_side = &if101;
	rule10.then_side = &then10;
	rule10.next = &rule11;
	
	/* rule # 9: if eLeft and Right then oSlRight */
	struct rule_element_type then9;
	then9.value = &oSlRight.value;
	then9.next = NULL;
	
	struct rule_element_type if92;
	if92.value = &right.value;
	if92.next = NULL;
	
	struct rule_element_type if91;
	if91.value = &eLeft.value;
	if91.next = &if92;
	
	struct rule_type rule9;
	rule9.if_side = &if91;
	rule9.then_side = &then9;
	rule9.next = &rule10;
 
	/* rule # 8: if eLeft and shRight then oRight */
	struct rule_element_type then8;
	then8.value = &oRight.value;
	then8.next = NULL;
	
	struct rule_element_type if82;
	if82.value = &shRight.value;
	if82.next = NULL;
	
	struct rule_element_type if81;
	if81.value = &eLeft.value;
	if81.next = &if82;
	
	struct rule_type rule8;
	rule8.if_side = &if81;
	rule8.then_side = &then8;
	rule8.next = &rule9;
 
	/* rule # 7: if center then oStraight */
	struct rule_element_type then7;
	then7.value = &oStraight.value;
	then7.next = NULL;

 
	struct rule_element_type if71;
	if71.value = &center.value;
	if71.next = NULL;
 
	struct rule_type rule7;
	rule7.if_side = &if71;
	rule7.then_side = &then7;
	rule7.next = &rule8;
 
 
     /* rule # 6: if eRight and straight then oSlLeft */
    struct rule_element_type then6;
    then6.value = &oSlLeft.value;
    then6.next = NULL;
    
	struct rule_element_type if62;
	if62.value = &straight.value;
	if62.next = NULL;
	
    struct rule_element_type if61;
    if61.value = &eRight.value;
	if61.next = &if62;
    
    struct rule_type rule6;
    rule6.if_side = &if61;
    rule6.then_side = &then6;
    rule6.next = &rule7;
    
    /* rule # 5: if eRight and slLeft then oSlLeft */
    struct rule_element_type then5;
    then5.value = &oSlLeft.value;
    then5.next = NULL;
    
    struct rule_element_type if52;
	if52.value = &slLeft.value;
	if52.next = NULL;
	
    struct rule_element_type if51;
    if51.value = &eRight.value;
    if51.next = &if52;
    
    struct rule_type rule5;
    rule5.if_side = &if51;
    rule5.then_side = &then5;
    rule5.next = &rule6;
    
    /* rule # 4: if eRight and left then oSlLeft */  
    struct rule_element_type then4;
    then4.value = &oSlLeft.value;
    then4.next = NULL;
    
    struct rule_element_type if42;
    if42.value = &left.value;
    if42.next = NULL;
    
    struct rule_element_type if41;
    if41.value = &eRight.value;
    if41.next = &if42;
    
    struct rule_type rule4;
    rule4.if_side = &if41;
    rule4.then_side = &then4;
    rule4.next = &rule5;
	  
    /* rule # 3: if eRight and sharpLeft then left */
    struct rule_element_type then3;
    then3.value = &oLeft.value;
    then3.next = NULL;
    
    struct rule_element_type if32;
    if32.value = &shLeft.value;
    if32.next = NULL;
    
    struct rule_element_type if31;
    if31.value = &eRight.value;
    if31.next = &if32;
    
    struct rule_type rule3;
    rule3.if_side = &if31;
    rule3.then_side = &then3;
    rule3.next = &rule4;
    
    /* rule # 2: if farLeft then right */
    struct rule_element_type if21;
    if21.value = &farLeft.value;
    if21.next = NULL;
    
    struct rule_element_type then2;
    then2.value = &oRight.value;
    then2.next = NULL;
    
    struct rule_type rule2;
    rule2.if_side = &if21;
    rule2.then_side = &then2;
    rule2.next = &rule3;
    
    /* rule # 1: if farRight then oLeft */
    struct rule_element_type if11;
    if11.value = &farRight.value;
    if11.next = NULL;
    
    struct rule_element_type then1;
    then1.value = &oLeft.value;
    then1.next = NULL;
    
    rule1.if_side = &if11;
    rule1.then_side = &then1;
    rule1.next = &rule2;
    
	// pointers to top of lists
    Rule_Base = &rule1;
    System_Inputs = &error;
    System_Outputs = &servo;
    
	// the methods performing the FLC
    fuzzification();
    rule_evaluation();
    defuzzification();
	setServo(servo.value);
}

