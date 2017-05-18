
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

//////////////////////////////////////////////////////////////////////////////////
// DEFINITIONS OF I/O AND POINTERS TO TOP OF LISTS                              //
//////////////////////////////////////////////////////////////////////////////////


struct io_type delta_C;
struct io_type delta_V;

//////////////////////////////////////////////////////////////////////////////////
// METHODS                                                                      //
//////////////////////////////////////////////////////////////////////////////////

void nDoFuzzy(int c, int v)
{
	///// DECLARATION OF C INPUT VARIABLE ///////////////////////////////////
	
	struct io_type delta_C; strcpy(delta_C.name, "delta_C");
	
	struct mf_type farRight;
	MATLAB_MF(&farRight, "farRight", 99, 100, 110, 125); // Min_value = 100
	struct mf_type smallRight;
	MATLAB_MF(&smallRight, "smallRig", 115, 130, 130, 145);
	struct mf_type centre;
	MATLAB_MF(&centre, "centre", 135, 150, 150, 165);
	struct mf_type smallLeft;
	MATLAB_MF(&smallLeft, "smallLe", 155, 170, 170, 185);
	struct mf_type farLeft;
	MATLAB_MF(&farLeft, "farLeft", 175, 190, 200, 201);  // Max_value = 200
	
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
	else if(c>200)			// if sensor value is bigger than delta_C's input set's upper limit
	{
		delta_C.value = 200;  // force input value to lowest point in delta_C's input set
	}
	else
	{
		delta_C.value = c;
	}
	
	///// DECLARATION OF V INPUT VARIABLE ///////////////////////////////////
	
	struct io_type delta_V; strcpy(delta_V.name, "delta_V");
	
	struct mf_type leftOriented;
	MATLAB_MF(&leftOriented, "leftOrien", -1, 0, 30, 60); // min V is 0
	
	struct mf_type straightOriented;
	MATLAB_MF(&straightOriented, "straOri", 10, 40, 40, 70);
	
	struct mf_type rightOriented;
	MATLAB_MF(&rightOriented, "righOri", 20, 50, 80, 81); // max V is 80
	
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

	struct io_type steering; strcpy(steering.name, "steering");
	
	struct mf_type sharpLeft;
	MATLAB_MF(&sharpLeft, "sharpLe", 2209, 2210, 2210, 2390);
	struct mf_type left;
	MATLAB_MF(&left, "left", 2310, 2450, 2450, 2590);
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 2510, 2660, 2660, 2810);
	struct mf_type right;
	MATLAB_MF(&right, "right", 2730, 2870, 2870, 3010);
	struct mf_type sharpRight;
	MATLAB_MF(&sharpRight, "sharpRi", 2930, 3110, 3110, 3111);
	
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

	//RULE 15 "if deltaC is farRight and deltaV is leftOriented then steering is straight"
	struct rule_element_type then15;
	then15.value = &straight.value;
	then15.next = NULL;

	struct rule_element_type if152;
	if152.value = &leftOriented.value;
	if152.next = NULL;

	struct rule_element_type if151;
	if151.value = &farRight.value;
	if151.next = &if152;

	struct rule_type rule15;
	rule15.if_side = &if151;
	rule15.then_side = &then15;
	rule15.next = NULL;
	
	
	//RULE 14 "if deltaC is farRight and deltaV is straightOriented then steering is left"
	struct rule_element_type then14;
	then14.value = &left.value;
	then14.next = NULL;

	struct rule_element_type if142;
	if142.value = &straightOriented.value;
	if142.next = NULL;

	struct rule_element_type if141;
	if141.value = &farRight.value;
	if141.next = &if142;

	struct rule_type rule14;
	rule14.if_side = &if141;
	rule14.then_side = &then14;
	rule14.next = &rule15;

	//RULE 13 "if deltaC is farRight and deltaV is rightOriented then steering is sharpLeft"
	struct rule_element_type then13;
	then13.value = &sharpLeft.value;
	then13.next = NULL;

	struct rule_element_type if132;
	if132.value = &rightOriented.value;
	if132.next = NULL;

	struct rule_element_type if131;
	if131.value = &farRight.value;
	if131.next = &if132;

	struct rule_type rule13;
	rule13.if_side = &if131;
	rule13.then_side = &then13;
	rule13.next = &rule14;

	//RULE 12 "if deltaC is smallRight and deltaV is leftOriented then steering is right"
	struct rule_element_type then12;
	then12.value = &right.value;
	then12.next = NULL;

	struct rule_element_type if122;
	if122.value = &leftOriented.value;
	if122.next = NULL;

	struct rule_element_type if121;
	if121.value = &smallRight.value;
	if121.next = &if122;

	struct rule_type rule12;
	rule12.if_side = &if121;
	rule12.then_side = &then12;
	rule12.next = &rule13;


	//RULE 11 "if deltaC is smallRight and deltaV is straightOriented then steering is left"
	struct rule_element_type then11;
	then11.value = &left.value;
	then11.next = NULL;

	struct rule_element_type if112;
	if112.value = &straightOriented.value;
	if112.next = NULL;

	struct rule_element_type if111;
	if111.value = &smallRight.value;
	if111.next = &if112;

	struct rule_type rule11;
	rule11.if_side = &if111;
	rule11.then_side = &then11;
	rule11.next = &rule12;


	//RULE 10 "if deltaC is smallRight and deltaV is rightOriented then steering is sharpLeft"
	struct rule_element_type then10;
	then10.value = &sharpLeft.value;
	then10.next = NULL;

	struct rule_element_type if102;
	if102.value = &rightOriented.value;
	if102.next = NULL;

	struct rule_element_type if101;
	if101.value = &smallRight.value;
	if101.next = &if102;

	struct rule_type rule10;
	rule10.if_side = &if101;
	rule10.then_side = &then10;
	rule10.next = &rule11;
	
	//RULE 9 "if deltaC is centre and deltaV is leftOriented then steering is right"
	struct rule_element_type then9;
	then9.value = &right.value;
	then9.next = NULL;

	struct rule_element_type if92;
	if92.value = &leftOriented.value;
	if92.next = NULL;

	struct rule_element_type if91;
	if91.value = &centre.value;
	if91.next = &if92;

	struct rule_type rule9;
	rule9.if_side = &if91;
	rule9.then_side = &then9;
	rule9.next = &rule10;

	
	//RULE 8 "if deltaC is centre and deltaV is straightOriented then steering is straight"
	struct rule_element_type then8;
	then8.value = &straight.value;
	then8.next = NULL;

	struct rule_element_type if82;
	if82.value = &straightOriented.value;
	if82.next = NULL;

	struct rule_element_type if81;
	if81.value = &centre.value;
	if81.next = &if82;

	struct rule_type rule8;
	rule8.if_side = &if81;
	rule8.then_side = &then8;
	rule8.next = &rule9;
	
	//RULE 7 "if deltaC is centre and deltaV is rightOriented then steering is left"
	struct rule_element_type then7;
	then7.value = &left.value;
	then7.next = NULL;

	struct rule_element_type if72;
	if72.value = &rightOriented.value;
	if72.next = NULL;

	struct rule_element_type if71;
	if71.value = &centre.value;
	if71.next = &if72;

	struct rule_type rule7;
	rule7.if_side = &if71;
	rule7.then_side = &then7;
	rule7.next = &rule8;
	
	
	//RULE 6 "if deltaC is smallLeft and deltaV is leftOriented then steering is sharpRight"
	struct rule_element_type then6;
	then6.value = &sharpRight.value;
	then6.next = NULL;

	struct rule_element_type if62;
	if62.value = &leftOriented.value;
	if62.next = NULL;

	struct rule_element_type if61;
	if61.value = &smallLeft.value;
	if61.next = &if62;

	struct rule_type rule6;
	rule6.if_side = &if61;
	rule6.then_side = &then6;
	rule6.next = &rule7;

	

	//RULE 5 "if deltaC is smallLeft and deltaV is straightOriented then steering is right"
	struct rule_element_type then5;
	then5.value = &right.value;
	then5.next = NULL;

	struct rule_element_type if52;
	if52.value = &straightOriented.value;
	if52.next = NULL;

	struct rule_element_type if51;
	if51.value = &smallLeft.value;
	if51.next = &if52;

	struct rule_type rule5;
	rule5.if_side = &if51;
	rule5.then_side = &then5;
	rule5.next = &rule6;

	

	//RULE 4 "if deltaC is smallLeft and deltaV is rightOriented then steering is left"
	struct rule_element_type then4;
	then4.value = &left.value;
	then4.next = NULL;
	
	struct rule_element_type if42;
	if42.value = &rightOriented.value;
	if42.next = NULL;
	
	struct rule_element_type if41;
	if41.value = &smallLeft.value;
	if41.next = &if42;

	struct rule_type rule4;
	rule4.if_side = &if41;
	rule4.then_side = &then4;
	rule4.next = &rule5;
	
	//RULE 3 "if deltaC is farLeft and deltaV is leftOriented then steering is sharpRight"
	struct rule_element_type then3;
	then3.value = &sharpRight.value;
	then3.next = NULL;
	
	struct rule_element_type if32;
	if32.value = &leftOriented.value;
	if32.next = NULL;
	
	struct rule_element_type if31;
	if31.value = &farLeft.value;
	if31.next = &if32;

	struct rule_type rule3;
	rule3.if_side = &if31;
	rule3.then_side = &then3;
	rule3.next = &rule4;
	
	
	//RULE 2 "if deltaC is farLeft and deltaV is straightOriented then steering is right"
	struct rule_element_type then2;
	then2.value = &right.value;
	then2.next = NULL;

	struct rule_element_type if22;
	if22.value = &straightOriented.value;
	if22.next = NULL;
	
	struct rule_element_type if21;
	if21.value = &farLeft.value;
	if21.next = &if22;

	struct rule_type rule2;
	rule2.if_side = &if21;
	rule2.then_side = &then2;
	rule2.next = &rule3;

	//RULE 1 "if deltaC is farLeft and deltaV is rightOriented then steering is straight"
	struct rule_element_type then1;
	then1.value = &straight.value;
	then1.next = NULL;

	struct rule_element_type if12;
	if12.value = &rightOriented.value;
	if12.next = NULL;

	struct rule_element_type if11;
	if11.value = &farLeft.value;
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
		nDoFuzzy(c,v);
	}
	
	
}
