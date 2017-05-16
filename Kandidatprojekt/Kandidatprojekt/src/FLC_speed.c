/*
* FLC_speed.c
*
* Created: 5/15/2017 1:51:12 PM
*  Author: hjaek237
*/

//////////////////////////////////////////////////////////////////////////////////
// FUZZY SPEED CONTROLLER                                                       //
//////////////////////////////////////////////////////////////////////////////////


// INCLUDES
//////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include "fuzzy_speed_controller.h"
#include "general_FIS.h"
#include "servo.h"
#include <stdio.h>
#include "FLC_speed.h"


// DEFINITIONS
//////////////////////////////////////////////////////////////////////////////////

#define MIN_DISTANCE 0          // lower limit of distance variable
#define MAX_DISTANCE 75        // upper limit of distance variable
#define MIN_SPEED 2740          // lower limit of speed input variable



// FLC OBSTACLE AVOIDER
//////////////////////////////////////////////////////////////////////////////////


void FLC_speed(int currentServo, int midSonicRange, int currentEsc)
{
	// Inputs
	struct io_type distance; strcpy(distance.name, "distance");
	struct io_type steering; strcpy(steering.name, "steering");
	struct io_type esc; strcpy(esc.name, "esc");

	// Output
	struct io_type speed; strcpy(speed.name, "speed");


	// Variable assigned its reference value
	if (currentServo<=MAXLEFT) {
		steering.value = 202;
	}
	else if (currentServo>=MAXRIGHT)
	{
		steering.value = 330;
	}
	else
	{
		steering.value = (int) currentServo / 10;
	}
	
	// Variable assigned its reference value
	if (currentEsc<MINESC)
	{
		esc.value = MINESC;
	}
	else if (currentEsc>MAXESC)
	{
		esc.value = MAXESC;
	}
	else
	{
		esc.value = currentEsc;
	}

	// Variable assigned its reference value
	if(midSonicRange < MIN_DISTANCE)
	{
		distance.value = MIN_DISTANCE;
	}
	else if (midSonicRange > MAX_DISTANCE)
	{
		distance.value = MAX_DISTANCE;
	}
	else
	{
		distance.value = midSonicRange;
	}


	// DECLARATION OF STEERING INPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////


	// Set MFs
	struct mf_type maxLeft;
	MATLAB_MF(&maxLeft, "maxLeft", 201, 202, 205, 230);
	struct mf_type left;
	MATLAB_MF(&left, "left", 205, 230, 237, 262);
	struct mf_type straight;
	MATLAB_MF(&straight, "straight", 236, 266, 266, 296);
	struct mf_type right;
	MATLAB_MF(&right, "right", 267, 295, 302, 325);
	struct mf_type maxRight;
	MATLAB_MF(&maxRight, "maxRight", 301, 325, 330, 331);
	

	// Linked list for MFs
	steering.membership_functions = &maxLeft;
	maxLeft.next = &left;
	left.next = &straight;
	straight.next = &right;
	right.next = & maxRight;
	maxRight.next = NULL;


	// DECLARATION OF DISTANCE INPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////

	// Set MFs
	struct mf_type stopDist;
	MATLAB_MF(&stopDist, "stopDist", MIN_DISTANCE-1, MIN_DISTANCE, 10, 20);
	struct mf_type close;
	MATLAB_MF(&close, "close", 10, 30, 30, 50);
	struct mf_type distant;
	MATLAB_MF(&distant, "distant", 30, 50, 50, 70);
	struct mf_type faar;
	MATLAB_MF(&faar, "faar", 50, 70, 75, MAX_DISTANCE+1);


	// Linked list for MFs
	distance.membership_functions = &stopDist;
	stopDist.next = &close;
	close.next = &distant;
	distant.next = &faar;
	faar.next = NULL;

	// DECLARATION OF ESC INPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////


	// Set MFs
	struct mf_type neutral;
	MATLAB_MF(&neutral, "neutral", 2779, 2780, 2802, 2830);
	struct mf_type low;
	MATLAB_MF(&low, "low", 2825, 2835, 2835, 2845);
	struct mf_type medium;
	MATLAB_MF(&medium, "medium", 2830, 2840, 2840, 2850);
	struct mf_type high;
	MATLAB_MF(&high, "high", 2835, 2845, 2845, 2855);

	// Linked list for MFs
	esc.membership_functions = &neutral;
	neutral.next = &low;
	low.next = &medium;
	medium.next = &high;
	high.next = NULL;


	// SETTING I/O LINKED LISTS
	//////////////////////////////////////////////////////////////////////////////

	System_Inputs = &distance;
	distance.next = &steering;
	steering.next = &esc;
	esc.next = NULL;

	System_Outputs = &speed;
	speed.next = NULL;


	// DECLARATION OF SPEED OUTPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////


	// Set MFs
	struct mf_type zero;
	MATLAB_MF(&zero, "zero", 2779, 2780, 2802, 2830);
	struct mf_type slow;
	MATLAB_MF(&slow, "slow", 2825, 2835, 2835, 2845);
	struct mf_type average;
	MATLAB_MF(&average, "average", 2830, 2840, 2840, 2850);
	struct mf_type fast;
	MATLAB_MF(&fast, "fast", 2835, 2845, 2845, 2855);

	// Linked list for MFs
	speed.membership_functions = &zero;
	zero.next = &slow;
	slow.next = &average;
	average.next = &fast;
	fast.next = NULL;


	// SETTING I/O LINKED LISTS
	//////////////////////////////////////////////////////////////////////////////

	System_Inputs = &distance;
	distance.next = &steering;
	steering.next = &esc;
	esc.next = NULL;

	System_Outputs = &speed;
	speed.next = NULL;


	// DECLARATION OF RULES AND LISTS
	//////////////////////////////////////////////////////////////////////////////

	struct rule_type rule1; Rule_Base = &rule1;
	struct rule_type rule2; rule1.next = &rule2;
	struct rule_type rule3; rule2.next = &rule3;
	struct rule_type rule4; rule3.next = &rule4;
	struct rule_type rule5; rule4.next = &rule5; 
	struct rule_type rule6; rule5.next = &rule6;
	struct rule_type rule7; rule6.next = &rule7;
	struct rule_type rule8; rule7.next = &rule8;
	struct rule_type rule9; rule8.next = &rule9;
	struct rule_type rule10; rule9.next = &rule10;
	struct rule_type rule11; rule10.next = &rule11;
	struct rule_type rule12; rule11.next = &rule12;
	struct rule_type rule13; rule12.next = &rule13;
	struct rule_type rule14; rule13.next = &rule14;
	rule14.next = NULL;


	// RULE SETUP
	//////////////////////////////////////////////////////////////////////////////

	// if distance is stopDist then speed is neutral
	struct rule_element_type if11, then1;
	rule1.if_side = &if11; if11.next = NULL; rule1.then_side = &then1; then1.next = NULL;
	if11.value = &stopDist.value; then1.value = &zero.value;

	// if dist is close then speed is slow
	struct rule_element_type if21, then2;
	rule2.if_side = &if21; if21.next = NULL; rule2.then_side = &then2; then2.next = NULL;
	if21.value = &close.value; then2.value = &low.value;

	// if dist is faar and steering is right then speed is average
	struct rule_element_type if31, if32, then3;
	rule3.if_side = &if31; if31.next = &if32; if32.next = NULL; rule3.then_side = &then3; then3.next = NULL;
	if31.value = &faar.value; if32.value = &right.value; then3.value = &average.value;

	// if dist is faar and steering is left then speed is average
	struct rule_element_type if41, if42, then4;
	rule4.if_side = &if41; if41.next = &if42; if42.next = NULL; rule4.then_side = &then4; then4.next = NULL;
	if41.value = &faar.value; if42.value = &left.value; then4.value = &average.value;

	// if dist is faar and steering is maxRight then speed is slow
	struct rule_element_type if51, if52, then5;
	rule5.if_side = &if51; if51.next = &if52; if52.next = NULL; rule5.then_side = &then5; then5.next = NULL;
	if51.value = &faar.value; if52.value = &maxRight.value; then5.value = &slow.value;
	
	// if dist is faar and steering is maxLeft then speed is slow
	struct rule_element_type if61, if62, then6;
	rule6.if_side = &if61; if61.next = &if62; if62.next = NULL; rule6.then_side = &then6; then6.next = NULL;
	if61.value = &faar.value; if62.value = &maxLeft.value; then6.value = &slow.value;
	
	// if dist is distant and currentEsc is neutral then speed is slow
	struct rule_element_type if71, if72, then7;
	rule7.if_side = &if71; if71.next = &if72; if72.next = NULL; rule7.then_side = &then7; then7.next = NULL;
	if71.value = &distant.value; if72.value = &neutral.value; then7.value = &slow.value;
	
	// if dist is distant and currentEsc is low then speed is average
	struct rule_element_type if81, if82, then8;
	rule8.if_side = &if81; if81.next = &if82; if82.next = NULL; rule8.then_side = &then8; then8.next = NULL;
	if81.value = &distant.value; if82.value = &low.value; then8.value = &average.value;
	
	// if dist is distant and currentEsc is medium then speed is average
	struct rule_element_type if91, if92, then9;
	rule9.if_side = &if91; if91.next = &if92; if92.next = NULL; rule9.then_side = &then9; then9.next = NULL;
	if91.value = &distant.value; if92.value = &medium.value; then9.value = &average.value;
	
	// if dist is faar and steering is straight and currentEsc is neutral then speed is average
	struct rule_element_type if101, if102, if103, then10;
	rule10.if_side = &if101; if101.next = &if102; if102.next = &if103; if103.next = NULL; rule10.then_side = &then10; then10.next = NULL;
	if101.value = &faar.value; if102.value = &straight.value; if103.value = &neutral; then10.value = &slow.value;

	// if dist is faar and steering is straight and currentEsc is low then speed is average
	struct rule_element_type if111, if112, if113, then11;
	rule11.if_side = &if111; if111.next = &if112; if112.next = &if113; if113.next = NULL; rule11.then_side = &then11; then11.next = NULL;
	if111.value = &faar.value; if112.value = &straight.value; if113.value = &low.value; then11.value = &average.value;
	
	// if dist is faar and steering is straight and currentEsc is medium then speed is fast
	struct rule_element_type if121, if122, if123, then12;
	rule12.if_side = &if121; if121.next = &if122; if122.next = &if123; if123.next = NULL; rule12.then_side = &then12; then12.next = NULL;
	if121.value = &faar.value; if122.value = &straight.value; if123.value = &medium.value; then12.value = &fast.value;
	
	// if dist is faar and steering is straight and currentEsc is high then speed is fast
	struct rule_element_type if131, if132, if133, then13;
	rule13.if_side = &if131; if131.next = &if132; if132.next = &if133; if133.next = NULL; rule13.then_side = &then13; then13.next = NULL;
	if131.value = &faar.value; if132.value = &straight.value; if133.value = &high.value; then13.value = &fast.value;

	// if dist is faar and steering is straight then speed is fast
	struct rule_element_type if141, if142, then14;
	rule14.if_side = &if141; if141.next = &if142; if142.next = NULL; rule14.then_side = &then14; then14.next = NULL;
	if141.value = &faar.value; if142.value = &straight.value; then14.value = &fast.value;



	// EXECUTING FUZZY LOGIC & OUTPUT VALUE
	//////////////////////////////////////////////////////////////////////////////

	fuzzification();
	rule_evaluation();
	defuzzification();

	if (speed.value > MAXESC)
	{
		setESC(MAXESC);
	}
	else if (speed.value < NEUTRAL)
	{
		setESC(NEUTRAL);
	}
	else
	{
		setESC(speed.value);
	}
}
