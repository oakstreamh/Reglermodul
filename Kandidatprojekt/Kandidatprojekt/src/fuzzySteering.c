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
void FLC_steering(int c, int s,	int v)

{
	
	
	
	// DECLARATION OF C INPUT
	struct io_type delta_C;	
	strcpy(delta_C.name, "delta_C");
	delta_C.value = c;
	//MFs
	struct mf_type negative;
	strcpy(negative.name, "negative");
	negative.value = 0;
	negative.point1 = 44;
	negative.point2= 110;
	negative.slope1 = 100;
	negative.slope2 = 2;
	struct mf_type zero;
	strcpy(zero.name, "zero");
	zero.value = 0;
	zero.point1 = 75;
	zero.point2= 175;
	zero.slope1 = 50;
	zero.slope2 = 50;
	struct mf_type positive;
	strcpy(positive.name, "positive");
	positive.value = 0;
	positive.point1 = 140;
	positive.point2= 206;
	positive.slope1 = 2;
	positive.slope2 = 100;

	delta_C.membership_functions = &negative;
	negative.next = &zero;
	zero.next = &positive;
	positive.next = NULL;
	
	// DECLARATION OF STEERING INPUT
	struct io_type steering;
	strcpy(steering.name, "steering");
	steering.value = s;
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
	delta_V.value = v;
	//MFs
	struct mf_type small;
	strcpy(small.name, "small");
	small.value = 0;
	small.point1 = 39;
	small.point2= 60;
	small.slope1 = 100;
	small.slope2 = 5;
	struct mf_type medium;
	strcpy(medium.name, "medium");
	medium.value = 0;
	medium.point1 = 50;
	medium.point2= 70;
	medium.slope1 = 10;
	medium.slope2 = 10;
	struct mf_type high;
	strcpy(high.name, "high");
	high.value = 0;
	high.point1 = 60;
	high.point2= 101;
	high.slope1 = 5;
	high.slope2 = 100;

	delta_V.membership_functions = &small;
	small.next = &medium;
	medium.next = &high;
	high.next = NULL;
	   
	   

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
	
	
	/* THE NEW RULE BASE
	* #1 IF 
	*/
	
	
	
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
	struct rule_type rule1;		// first rule in rule base
	
	/* rule # 15: if eRight and slRight then oSlLeft */
	struct rule_element_type then15;
	then15.value = &oSlLeft.value;
	then15.next = NULL;
	
	struct rule_element_type if152;
	if152.value = &slRight.value;
	if152.next = NULL;
	
	struct rule_element_type if151;
	if151.value = &eRight.value;
	if151.next = &if152;
	
	struct rule_type rule15;
	rule15.if_side = &if151;
	rule15.then_side = &then15;
	rule15.next = NULL;
	
	/* rule # 14: if eRight and right then oSlLeft */
	struct rule_element_type then14;
	then14.value = &oSlLeft.value;
	then14.next = NULL;
	
	struct rule_element_type if142;
	if142.value = &right.value;
	if142.next = NULL;
	
	struct rule_element_type if141;
	if141.value = &eRight.value;
	if141.next = &if142;
	
	struct rule_type rule14;
	rule14.if_side = &if141;
	rule14.then_side = &then14;
	rule14.next = &rule15;
	
	/* rule # 13: if eLeft and slLeft then oSlRight */
	struct rule_element_type then13;
	then13.value = &oSlRight.value;
	then13.next = NULL;
	
	struct rule_element_type if132;
	if132.value = &slLeft.value;
	if132.next = NULL;
	
	struct rule_element_type if131;
	if131.value = &eLeft.value;
	if131.next = &if132;
	
	struct rule_type rule13;
	rule13.if_side = &if131;
	rule13.then_side = &then13;
	rule13.next = &rule14;
	
	/* rule # 12: if eLeft and left then oSlRight */
	struct rule_element_type then12;
	then12.value = &oSlRight.value;
	then12.next = NULL;
	
	struct rule_element_type if122;
	if122.value = &left.value;
	if122.next = NULL;
	
	struct rule_element_type if121;
	if121.value = &eLeft.value;
	if121.next = &if122;
	
	struct rule_type rule12;
	rule12.if_side = &if121;
	rule12.then_side = &then12;
	rule12.next = &rule13;
	
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
	rule11.next = &rule12;
	
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
	System_Inputs = &delta_C;
	System_Outputs = &servo;
	
	
	// set iErr's input value to measErr value
	if(measErr<0)				// if sensor value is smaller than delta_C's input set's lower limit
	{
		delta_C.value = 0;  // force input value to lowest point in delta_C's input set
	}
	else if(measErr>250)			// if sensor value is bigger than delta_C's input set's upper limit
	{
		delta_C.value = 250;  // force input value to lowest point in delta_C's input set
	}
	else
	{
		delta_C.value = measErr;
	}
	
	// set iAng's input value to measAng value
	if(measAng<2260)				// if sensor value is smaller than error's input set lower limit
	{
		angle.value = 2260;  // force input value to lowest point in delta_C's input set
	}
	else if(measAng>3060)			// if sensor value is bigger than error's input set's upper limit
	{
		angle.value = 3060;  // force input value to lowest point in error's input set
	}
	else
	{
		angle.value = measAng;
	}
	
	// the methods performing the FLC
	fuzzification();
	rule_evaluation();
	defuzzification();
	setServo(servo.value);
}

