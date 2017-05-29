
//////////////////////////////////////////////////////////////////////////////////
// FUZZY SPEED CONTROLLER                                                       //
//////////////////////////////////////////////////////////////////////////////////

#define F_CPU 14745600


//////////////////////////////////////////////////////////////////////////////////
// INCLUDES                                                                     //
//////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include "nfuzzySteering.h"
#include "general_FIS.h"
#include <stdio.h>
#include "servo.h"

//////////////////////////////////////////////////////////////////////////////////
// DEFINITIONS OF I/O AND POINTERS TO TOP OF LISTS                              //
//////////////////////////////////////////////////////////////////////////////////

void nDoFuzzy(int c, int v);
struct io_type delta_C;
struct io_type delta_V;

//////////////////////////////////////////////////////////////////////////////////
// METHODS                                                                      //
//////////////////////////////////////////////////////////////////////////////////

void nDoFuzzy(int c, int v)
{      
	///// DECLARATION OF C INPUT VARIABLE ///////////////////////////////////
	
	
	int adjustC = 10;
	
	struct io_type delta_C; strcpy(delta_C.name, "delta_C");
	
	struct mf_type farRight;
	MATLAB_MF(&farRight, "farRight", 99, 100, 120+adjustC, 130+adjustC); // Min_value = 100
	struct mf_type smallRight;
	MATLAB_MF(&smallRight, "smallRig", 120+adjustC, 130+adjustC, 140+adjustC, 145+adjustC);
	struct mf_type centre;
	MATLAB_MF(&centre, "centre", 135+adjustC, 150+adjustC, 150+adjustC, 165+adjustC);
	struct mf_type smallLeft;
	MATLAB_MF(&smallLeft, "smallLe", 155+adjustC, 160+adjustC, 170+adjustC, 180+adjustC);
	struct mf_type farLeft;
	MATLAB_MF(&farLeft, "farLeft", 170+adjustC, 180+adjustC, 200+adjustC, 201+adjustC);  // Max_value = 200
	
	delta_C.membership_functions = &farRight;
	farRight.next = &smallRight;
	smallRight.next = &centre;
	centre.next = &smallLeft;
	smallLeft.next = &farLeft;
	farLeft.next = NULL;
	
	// set iErr's input value to measErr value
	if(c<100)				// if sensor value is smaller than delta_C's input set's lower limit
	{
		delta_C.value = 100;  // force input value to lowest point in delta_C's input set
	}
	else if(c>200+adjustC)			// if sensor value is bigger than delta_C's input set's upper limit
	{
		delta_C.value = 200+adjustC;  // force input value to lowest point in delta_C's input set
	}
	else
	{
		delta_C.value = c;
	}
	
	///// DECLARATION OF V INPUT VARIABLE ///////////////////////////////////
	
	struct io_type delta_V; strcpy(delta_V.name, "delta_V");
	
	struct mf_type leftOriented;
	MATLAB_MF(&leftOriented, "leftOri", -1, 0, 0, 60); // min V is 0
	
	struct mf_type straightOriented;
	MATLAB_MF(&straightOriented, "straOri", 10, 40, 40, 70);
		
	struct mf_type rightOriented;
	MATLAB_MF(&rightOriented, "rightOri", 20, 80, 80, 81); // max V is 80

	delta_V.membership_functions = &leftOriented;
	leftOriented.next = &straightOriented;
	straightOriented.next = &rightOriented;
	rightOriented.next = NULL;
	
	// set V's input value to V´s value
	if(v<=0)				// if sensor value is smaller than error's input set lower limit
	{
		delta_V.value = 1;  // force input value to lowest point in delta_V's input set
	}
	else if(v>=80)			// if sensor value is bigger than error's input set's upper limit
	{
		delta_V.value = 79;  // force input value to lowest point in error's input set
	}
	else
	{
		delta_V.value = v;
	}
	
	///// DECLARATION OF STEERING OUTPUT VARIABLE ///////////////////////////////////

	struct io_type steering; strcpy(steering.name, "steering"); // All outputs downscaled by a factor 10
	
	struct mf_type sharpLeft;
	MATLAB_MF(&sharpLeft, "sharpLe", 216, 217, 217, 235);
	struct mf_type left;
	MATLAB_MF(&left, "left", 224, 239, 239, 254);
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 244, 259, 259, 274);
	struct mf_type right;
	MATLAB_MF(&right, "right", 264, 279, 279, 294);
	struct mf_type sharpRight;
	MATLAB_MF(&sharpRight, "sharpRi", 285, 302, 302, 303);
	
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


	struct rule_type rule1; Rule_Base = &rule1;
	struct rule_type rule2; rule1.next = &rule2;
	struct rule_type rule3; rule2.next = &rule3;
	struct rule_type rule4; rule3.next = &rule4;
	struct rule_type rule5; rule4.next = &rule5;
	struct rule_type rule6; rule5.next = &rule6;
	struct rule_type rule7; rule6.next = &rule7;
	rule7.next = NULL;


	// RULE SETUP
	//////////////////////////////////////////////////////////////////////////////

	////RULE 1 "if deltaC is farLeft then steering is SharpRight"
	struct rule_element_type if11, then1;
	rule1.if_side = &if11; if11.next = NULL; rule1.then_side = &then1; then1.next = NULL;
	if11.value = &farLeft.value; then1.value = &sharpRight.value;

	////RULE 2 "if deltaC is smallLeft then steering is right"
	struct rule_element_type if21, then2;
	rule2.if_side = &if21; if21.next = NULL; rule2.then_side = &then2; then2.next = NULL;
	if21.value = &smallLeft.value; then2.value = &right.value;

	////RULE 5 "if deltaC is centre and deltaV is rightOriented then steering is left"
	struct rule_element_type if31, if32, then3;
	rule3.if_side = &if31; if31.next = &if32; if32.next = NULL; rule3.then_side = &then3; then3.next = NULL;
	if31.value = &centre.value; if32.value = &rightOriented.value; then3.value = &left.value;
	
	////RULE 6 "if deltaC is centre and deltaV is straightOriented then steering is straight"
	struct rule_element_type if41, if42, then4;
	rule4.if_side = &if41; if41.next = &if42; if42.next = NULL; rule4.then_side = &then4; then4.next = NULL;
	if41.value = &centre.value; if42.value = &straightOriented.value; then4.value = &straight.value;
	
	////RULE 5 "if deltaC is centre and deltaV is leftOriented then steering is right"
	struct rule_element_type if51, if52, then5;
	rule5.if_side = &if51; if51.next = &if52; if52.next = NULL; rule5.then_side = &then5; then5.next = NULL;
	if51.value = &centre.value; if52.value = &leftOriented.value; then5.value = &right.value;
	
	////RULE 6 "if deltaC is smallRight then steering is Left"
	struct rule_element_type if61, then6;
	rule6.if_side = &if61; if61.next = NULL; rule6.then_side = &then6; then6.next = NULL;
	if61.value = &smallRight.value; then6.value = &left.value;
	
	////RULE 7 "if deltaC is farRight then steering is sharpLeft"
	struct rule_element_type if71, then7;
	rule7.if_side = &if71; if71.next = NULL; rule7.then_side = &then7; then7.next = NULL;
	if71.value = &farRight.value;  then7.value = &sharpLeft.value;
	

	
	
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

/* FLC_steering is a fuzzy logic controller to perform lane following.
* Input values are unitless reference values derived from image processing.
* The image processing detects straight lines through Hough transform.
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
void nFuzzySteering(int c, int v)
{
	if ((c == 1) && (v == 81))        // right curvature, turn right
	{
		setServo(MAXRIGHT-159);
	}
	else if ((c == 2) && (v == 81))       // left curvature, turn left
	{
		setServo(MAXLEFT);
	}
	else                                   // straight road, do fuzzy
	{
		nDoFuzzy(c,v);
	}
}
