
//////////////////////////////////////////////////////////////////////////////////
// FUZZY SPEED CONTROLLER                                                       //
//////////////////////////////////////////////////////////////////////////////////

#define F_CPU 14745600


//////////////////////////////////////////////////////////////////////////////////
// INCLUDES                                                                     //
//////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include "fuzzySteering.h"
#include "general_FIS.h"
#include <stdio.h>
#include "servo.h"
#include <util/delay.h>

//////////////////////////////////////////////////////////////////////////////////
// DEFINITIONS OF I/O AND POINTERS TO TOP OF LISTS                              //
//////////////////////////////////////////////////////////////////////////////////


struct io_type delta_C;
struct io_type delta_V;

//////////////////////////////////////////////////////////////////////////////////
// METHODS                                                                      //
//////////////////////////////////////////////////////////////////////////////////

void newDoFuzzy(int c, int v)
{
	///// DECLARATION OF C INPUT VARIABLE ///////////////////////////////////
	
	struct io_type delta_C; strcpy(delta_C.name, "delta_C");
	
	struct mf_type rightSide;
	MATLAB_MF(&rightSide, "rightSide", 99, 100, 120, 135); // Min_value = 100
	struct mf_type centre;
	MATLAB_MF(&centre, "centre", 125, 145, 155, 175);
	struct mf_type leftSide;
	MATLAB_MF(&leftSide, "leftSide", 165, 180, 199, 200);  // Max_value = 199
	
	delta_C.membership_functions = &rightSide;
	rightSide.next = &centre;
	centre.next = &leftSide;
	leftSide.next = NULL;
	
	// set iErr's input value to measErr value
	if(c<100)				// if sensor value is smaller than delta_C's input set's lower limit
	{
		delta_C.value = 100;  // force input value to lowest point in delta_C's input set
	}
	else if(c>199)			// if sensor value is bigger than delta_C's input set's upper limit
	{
		delta_C.value = 199;  // force input value to lowest point in delta_C's input set
	}
	else
	{
		delta_C.value = c;
	}
	
	///// DECLARATION OF V INPUT VARIABLE ///////////////////////////////////
	
	struct io_type delta_V; strcpy(delta_V.name, "delta_V");
	
	struct mf_type inMinus;
	MATLAB_MF(&inMinus, "inMinus", 0, 1, 20, 35); // min V is 1
	struct mf_type inNyll;
	MATLAB_MF(&inNyll, "inNyll", 15, 35, 40, 55);
	struct mf_type inPlus;
	MATLAB_MF(&inPlus, "inPlus", 40, 50, 74 , 75); // max V is 74
	
	delta_V.membership_functions = &inMinus;
	inMinus.next = &inNyll;
	inNyll.next = &inPlus;
	inPlus.next = NULL;
	
	// set V's input value to V´s value
	if(v<1)				// if sensor value is smaller than error's input set lower limit
	{
		delta_V.value = 1;  // force input value to lowest point in delta_V's input set
	}
	else if(v>74)			// if sensor value is bigger than error's input set's upper limit
	{
		delta_V.value = 74;  // force input value to lowest point in error's input set
	}
	else
	{
		delta_V.value = v;
	}
	
	///// DECLARATION OF STEERING OUTPUT VARIABLE ///////////////////////////////////

	struct io_type steering; strcpy(steering.name, "steering");
	
	struct mf_type sharpLeft;
	MATLAB_MF(&sharpLeft, "sharpLeft", 2359, 2360, 2360, 2460);
	struct mf_type left;
	MATLAB_MF(&left, "left", 2400, 2460, 2460, 2560);
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 2520, 2660, 2660, 2720);
	struct mf_type right;
	MATLAB_MF(&right, "right", 2700, 2800, 2800, 2900);
	struct mf_type sharpRight;
	MATLAB_MF(&sharpRight, "sharpRight", 2860, 2960, 2960, 2961);
	
	steering.membership_functions = &sharpRight;
	sharpRight.next = &right;
	right.next = &straight;
	straight.next = &left;
	left.next = &sharpLeft;
	sharpLeft.next = NULL;
	
	
	
	// pointers to top of lists

	System_Inputs = &delta_C;
	delta_C.next = &delta_V;
	delta_V.next = NULL;
	System_Outputs = &steering;
	steering.next = NULL;
	
	
	
	//RULE 9 "if C is centre AND V is rightOriented then servo is left"
	struct rule_element_type then9;
	then9.value = &left.value;
	then9.next = NULL;

	struct rule_element_type if92;
	if92.value = &inPlus.value;
	if92.next = NULL;

	struct rule_element_type if91;
	if91.value = &centre.value;
	if91.next = &if92;

	struct rule_type rule9;
	rule9.if_side = &if91;
	rule9.then_side = &then9;
	rule9.next = NULL;

	
	//RULE 8 "if C is positive AND V is inPlus then servo is right"
	struct rule_element_type then8;
	then8.value = &right.value;
	then8.next = NULL;

	struct rule_element_type if82;
	if82.value = &inPlus.value;
	if82.next = NULL;

	struct rule_element_type if81;
	if81.value = &leftSide.value;
	if81.next = &if82;

	struct rule_type rule8;
	rule8.if_side = &if81;
	rule8.then_side = &then8;
	rule8.next = &rule9;
	
	//RULE 7 "if C is positive AND V is inNyll then servo is sharpRight"
	struct rule_element_type then7;
	then7.value = &sharpRight.value;
	then7.next = NULL;

	struct rule_element_type if72;
	if72.value = &inNyll.value;
	if72.next = NULL;

	struct rule_element_type if71;
	if71.value = &leftSide.value;
	if71.next = &if72;

	struct rule_type rule7;
	rule7.if_side = &if71;
	rule7.then_side = &then7;
	rule7.next = &rule8;
	
	
	//RULE 6 "if C is positive AND V is minus then servo is straight"
	struct rule_element_type then6;
	then6.value = &straight.value;
	then6.next = NULL;

	struct rule_element_type if62;
	if62.value = &inMinus.value;
	if62.next = NULL;

	struct rule_element_type if61;
	if61.value = &leftSide.value;
	if61.next = &if62;

	struct rule_type rule6;
	rule6.if_side = &if61;
	rule6.then_side = &then6;
	rule6.next = &rule7;

	

	//RULE 5 "if C is negative  AND V is nyll then servo is sharpLeft"
	struct rule_element_type then5;
	then5.value = &sharpLeft.value;
	then5.next = NULL;

	struct rule_element_type if52;
	if52.value = &inNyll.value;
	if52.next = NULL;

	struct rule_element_type if51;
	if51.value = &rightSide.value;
	if51.next = &if52;

	struct rule_type rule5;
	rule5.if_side = &if51;
	rule5.then_side = &then5;
	rule5.next = &rule6;

	

	//RULE 4 "if C is zero t AND V is nyll then servo is Straight"
	struct rule_element_type then4;
	then4.value = &straight.value;
	then4.next = NULL;
	
	struct rule_element_type if42;
	if42.value = &inNyll.value;
	if42.next = NULL;
	
	struct rule_element_type if41;
	if41.value = &centre.value;
	if41.next = &if42;

	struct rule_type rule4;
	rule4.if_side = &if41;
	rule4.then_side = &then4;
	rule4.next = &rule5;
	
	//RULE 3 "if C is negative AND V is nyll then servo is SharpLeft"
	struct rule_element_type then3;
	then3.value = &left.value;
	then3.next = NULL;
	
	struct rule_element_type if32;
	if32.value = &inPlus.value;
	if32.next = NULL;
	
	struct rule_element_type if31;
	if31.value = &rightSide.value;
	if31.next = &if32;

	struct rule_type rule3;
	rule3.if_side = &if31;
	rule3.then_side = &then3;
	rule3.next = &rule4;
	
	
	//RULE 2 "if C is negative AND V is minus then servo is Left"
	struct rule_element_type then2;
	then2.value = &left.value;
	then2.next = NULL;

	struct rule_element_type if22;
	if22.value = &inMinus.value;
	if22.next = NULL;
	
	struct rule_element_type if21;
	if21.value = &rightSide.value;
	if21.next = &if22;

	struct rule_type rule2;
	rule2.if_side = &if21;
	rule2.then_side = &then2;
	rule2.next = &rule3;

	//RULE 1 "if C is zero AND V is minus then steering is right"
	struct rule_element_type then1;
	then1.value = &right.value;
	then1.next = NULL;

	struct rule_element_type if12;
	if12.value = &inMinus.value;
	if12.next = NULL;

	struct rule_element_type if11;
	if11.value = &centre.value;
	if11.next = &if12;

	struct rule_type rule1;
	rule1.if_side = &if11;
	rule1.then_side = &then1;
	rule1.next = &rule2;
	Rule_Base = &rule1;
	
	
	// the methods performing the FLC
	fuzzification();
	rule_evaluation();
	defuzzification();
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

/* FLC_steering is a fuzzy logic controller to perform lane following.
 * Input values are unitless reference values derived from image processing.
 * The image processing detects straight lines through the Hough transform.
 * To manage sharp curvatures, the image processing application implements a state 
 * machine.
 * Input parameters are c and v. Three reserved pairs defines states.
 * c = 227 & v = 45 corresponds to no lines detected
 * c = 20 & v = 45 defines the state, sharp right curvature
 * c = 210 & v = 45 defines the state, sharp left curvature
 * c in [45, 205] and v in [0, 80] corresponds to the fourth state, straight rode
 *
 * The fuzzy logic controller is designed to manage the fourth state
 */
int fuzzySteering(int c, int v)
{
    
    
    
	if ((c == 1) & (v == 81))        // right curvature, turn right
	{
		setServo(MAXRIGHT);
	}
	else if ((c == 2) & (v == 81))       // left curvature, turn left
	{
		setServo(MAXLEFT);
	}
	else                                   // straight road, do fuzzy
	{
		newDoFuzzy(c,v);
	}
	
	
	return 1;
}