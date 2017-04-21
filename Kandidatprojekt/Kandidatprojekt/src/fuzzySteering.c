//////////////////////////////////////////////////////////////////////////////////
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
void FLC_steering(c, s, v)
int c;
int s;
int v;
{
	// DECLARATION OF C INPUT
	struct io_type delta_C;
	strcpy(delta_C.name, "delta_C");
	//MFs
	struct mf_type negative;
	strcpy(negative.name, "negative");
	negative.value = 0;
	negative.point1 = 44;
	negative.point2= 110;
	negative.slope1 = 100;
	negative.slope2 = 2;
	struct mf_type zer;
	strcpy(zer.name, "zer");
	zer.value = 0;
	zer.point1 = 75;
	zer.point2= 175;
	zer.slope1 = 50;
	zer.slope2 = 50;
	struct mf_type positive;
	strcpy(positive.name, "positive");
	positive.value = 0;
	positive.point1 = 140;
	positive.point2= 206;
	positive.slope1 = 2;
	positive.slope2 = 100;

	delta_C.membership_functions = &negative;
	negative.next = &zer;
	zer.next = &positive;
	positive.next = NULL;
	
	// DECLARATION OF STEERING INPUT
	struct io_type steering;
	strcpy(steering.name, "steering");
	
	//MFs
	struct mf_type inShRight;
	strcpy(inShRight.name, "inShRight");
	inShRight.value = 0;
	inShRight.point1 = 2259;
	inShRight.point2= 2400;
	inShRight.slope1 = 100;
	inShRight.slope2 = 1;
	
	struct mf_type right;
	strcpy(right.name, "right");
	right.value = 0;
	right.point1 = 2300;
	right.point2= 2580;
	right.slope1 = 1;
	right.slope2 = 1;
	
	struct mf_type inStraight;
	strcpy(inStraight.name, "inStraight");
	inStraight.value = 0;
	inStraight.point1 = 2480;
	inStraight.point2= 2840;
	inStraight.slope1 = 1;
	inStraight.slope2 = 1;

	struct mf_type inLeft;
	strcpy(inLeft.name, "inLeft");
	inLeft.value = 0;
	inLeft.point1 = 2740;
	inLeft.point2= 3020;
	inLeft.slope1 = 1;
	inLeft.slope2 = 1;
	
	struct mf_type inShLeft;
	strcpy(inShLeft.name, "inShLeft");
	inShLeft.value = 0;
	inShLeft.point1 = 2920;
	inShLeft.point2= 3061;
	inShLeft.slope1 = 1;
	inShLeft.slope2 = 100;

	steering.membership_functions = &inShRight;
	inShRight.next = &right;
	right.next = &inStraight;
	inStraight.next = &inLeft;
	inLeft.next = &inShLeft;
	inShLeft.next = NULL;
	
	
	// DECLARATION OF V INPUT
	struct io_type delta_V;
	strcpy(delta_V.name, "delta_V");

	//MFs
	struct mf_type small;
	strcpy(small.name, "small");
	small.value = 0;
	small.point1 = -1;
	small.point2= 30;
	small.slope1 = 100;
	small.slope2 = 5;
	struct mf_type medium;
	strcpy(medium.name, "medium");
	medium.value = 0;
	medium.point1 = 20;
	medium.point2= 60;
	medium.slope1 = 5;
	medium.slope2 = 5;
	struct mf_type high;
	strcpy(high.name, "high");
	high.value = 0;
	high.point1 = 50;
	high.point2= 81;
	high.slope1 = 5;
	high.slope2 = 100;

	delta_V.membership_functions = &small;
	small.next = &medium;
	medium.next = &high;
	high.next = NULL;
	
	// DECLARATION OF STEERING SERVO OUTPUT

	struct io_type servo;
	strcpy(servo.name, "servo");

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
	
	servo.membership_functions = &oShright;
	servo.next = NULL;
		
	
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

	
	/* rule # 9: if C is zero and V is high then servo is slightLeft */
		struct rule_element_type then9;
		then9.value = &oSlLeft.value;
		then9.next = NULL;
		
		struct rule_element_type if92;
		if92.value = &high.value;
		if92.next = NULL;
		
		struct rule_element_type if91;
		if91.value = &zer.value;
		if91.next = &if92;
		
		struct rule_type rule9;
		rule9.if_side = &if91;
		rule9.then_side = &then9;
		rule9.next = NULL;
		
		/* rule # 8: if positive and V is high then servo is slightRight */
		struct rule_element_type then8;
		then8.value = &oSlRight.value;
		then8.next = NULL;
		
		struct rule_element_type if82;
		if82.value = &high.value;
		if82.next = NULL;
		
		struct rule_element_type if81;
		if81.value = &positive.value;
		if81.next = &if82;
		
		struct rule_type rule8;
		rule8.if_side = &if81;
		rule8.then_side = &then8;
		rule8.next = &rule9;
		
		/* rule # 7: if C is positive and V is medium then servo is sharpRight */
		struct rule_element_type then7;
		then7.value = &oShright.value;
		then7.next = NULL;
		
		struct rule_element_type if72;
		if72.value = &medium.value;
		if72.next = NULL;
		
		struct rule_element_type if71;
		if71.value = &positive.value;
		if71.next = &if72;
		
		struct rule_type rule7;
		rule7.if_side = &if71;
		rule7.then_side = &then7;
		rule7.next = &rule8;
		
		/* rule # 6: if C is positive and V is medium then servo is sharpRight */
		struct rule_element_type then6;
		then6.value = &oShright.value;
		then6.next = NULL;
		
		struct rule_element_type if62;
		if62.value = &medium.value;
		if62.next = NULL;
		
		struct rule_element_type if61;
		if61.value = &positive.value;
		if61.next = &if62;
		
		struct rule_type rule6;
		rule6.if_side = &if61;
		rule6.then_side = &then6;
		rule6.next = &rule7;
		
		/* rule # 5: if C is negative and V is medium then servo is sharpLeft */
		struct rule_element_type then5;
		then5.value = &oShLeft.value;
		then5.next = NULL;
		
		struct rule_element_type if52;
		if52.value = &medium.value;
		if52.next = NULL;
		
		struct rule_element_type if51;
		if51.value = &negative.value;
		if51.next = &if52;
		
		struct rule_type rule5;
		rule5.if_side = &if51;
		rule5.then_side = &then5;
		rule5.next = &rule6;
		
		/* rule # 4: if C is zero and V is medium then servo is straight */
		struct rule_element_type then4;
		then4.value = &oStraight.value;
		then4.next = NULL;
		
		struct rule_element_type if42;
		if42.value = &medium.value;
		if42.next = NULL;
		
		struct rule_element_type if41;
		if41.value = &zer.value;
		if41.next = &if42;
		
		struct rule_type rule4;
		rule4.if_side = &if41;
		rule4.then_side = &then4;
		rule4.next = &rule5;
		
		
		/* rule # 3: if C is negative and V is medium then servo is sharpLeft */
		struct rule_element_type then3;
		then3.value = &oShLeft.value;
		then3.next = NULL;
		
		struct rule_element_type if32;
		if32.value = &medium.value;
		if32.next = NULL;
		
		struct rule_element_type if31;
		if31.value = &negative.value;
		if31.next = &if32;
		
		struct rule_type rule3;
		rule3.if_side = &if31;
		rule3.then_side = &then3;
		rule3.next = &rule4;
		
		/* rule # 2: if C is negative and V is small then servo is slightLeft */
		struct rule_element_type then2;
		then2.value = &oSlLeft.value;
		then2.next = NULL;
		
		struct rule_element_type if22;
		if22.value = &small.value;
		if22.next = NULL;
		
		struct rule_element_type if21;
		if21.value = &negative.value;
		if21.next = &if22;
		
		struct rule_type rule2;
		rule2.if_side = &if21;
		rule2.then_side = &then2;
		rule2.next = &rule3;
		
		/* rule # 1: if C is zero and V is small then servo is slightRight */
		struct rule_element_type then1;
		then1.value = &oSlRight.value;
		then1.next = NULL;
		
		struct rule_element_type if12;
		if12.value = &small.value;
		if12.next = NULL;
		
		struct rule_element_type if11;
		if11.value = &negative.value;
		if11.next = &if12;
		
		struct rule_type rule1;
		rule1.if_side = &if11;
		rule1.then_side = &then1;
		rule1.next = &rule2;


	
	// pointers to top of lists
	Rule_Base = &rule1;
	System_Inputs = &delta_C;
	System_Outputs = &servo;
	
	
	
	
	// set iErr's input value to measErr value
	if(c<0)				// if sensor value is smaller than delta_C's input set's lower limit
	{
		delta_C.value = 0;  // force input value to lowest point in delta_C's input set
	}
	else if(c>250)			// if sensor value is bigger than delta_C's input set's upper limit
	{
		delta_C.value = 250;  // force input value to lowest point in delta_C's input set
	}
	else
	{
		delta_C.value = c;
	}
	
	// set iAng's input value to measAng value
	if(s<2260)				// if sensor value is smaller than error's input set lower limit
	{
		steering.value = 2260;  // force input value to lowest point in delta_C's input set
	}
	else if(s>3060)			// if sensor value is bigger than error's input set's upper limit
	{
		steering.value = 3060;  // force input value to lowest point in error's input set
	}
	else
	{
		steering.value = s;
	}
	
	// set V's input value to V´s value
	if(v<0)				// if sensor value is smaller than error's input set lower limit
	{
		delta_V.value = 0;  // force input value to lowest point in delta_V's input set
	}
	else if(v>80)			// if sensor value is bigger than error's input set's upper limit
	{
		delta_V.value = 80;  // force input value to lowest point in error's input set
	}
	else
	{
		delta_V.value = v;
	}
	
	// the methods performing the FLC
	fuzzification();
	rule_evaluation();
	defuzzification();
	setServo(servo.value);
}

