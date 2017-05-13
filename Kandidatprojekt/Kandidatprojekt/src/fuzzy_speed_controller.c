
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


// DEFINITIONS
//////////////////////////////////////////////////////////////////////////////////

#define MIN_DISTANCE 0          // lower limit of distance variable
#define MAX_DISTANCE 250        // upper limit of distance variable
#define MIN_SPEED 2740          // lower limit of speed input variable
#define MAX_SPEED 2900          // upper limit of speed input variable

int adjustment = 0;

// FLC OBSTACLE AVOIDER
//////////////////////////////////////////////////////////////////////////////////


void doFuzzy2(int currentOCR1A, int midSonicRange)
{
	// DECLARATION OF DISTANCE INPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////
	
	struct io_type distance; strcpy(distance.name, "distance");
	struct io_type speed; strcpy(speed.name, "speed");
	
	// Variable assigned its reference value
	if (currentOCR1A<MIN_SPEED) {
		speed.value = MIN_SPEED;
	}
	else if (currentOCR1A>MAX_SPEED)
	{
		speed.value = MAX_SPEED;
	}
	else
	{
		speed.value = currentOCR1A;
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
	
	
	// Set MFs
	struct mf_type stopDist;
	MATLAB_MF(&stopDist, "stopDist", MIN_DISTANCE-1, 0, 12, 20);
	struct mf_type oneM;
	MATLAB_MF(&oneM, "oneM", 18, 60, 60, 95);
	struct mf_type twoM;
	MATLAB_MF(&twoM, "twoM", 75, 125, 125, 175);
	struct mf_type threeM;
	MATLAB_MF(&threeM, "threeM", 150, 250, 250, MAX_DISTANCE+1);
	
	// Linked list for MFs
	distance.membership_functions = &stopDist;
	stopDist.next = &oneM;
	oneM.next = &twoM;
	twoM.next = &threeM;
	threeM.next = NULL;
	

	
	// DECLARATION OF SPEED INPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////
	

	// Set MFs
	struct mf_type still;
	MATLAB_MF(&still, "still", 2739, 2740, 2740, 2815);
	struct mf_type low;
	MATLAB_MF(&low, "low", 2825, 2830, 2830, 2835);
	struct mf_type cruising;
	MATLAB_MF(&cruising, "cruising", 2830, 2835, 2835, 2840);
	struct mf_type medium;
	MATLAB_MF(&medium, "medium", 2830, 2840, 2840, 2850);
	struct mf_type high;
	MATLAB_MF(&high, "high", 2835, 2845, 28545, 2855);
	
	// Linked list for MFs
	speed.membership_functions = &still;
	still.next = &low;
	low.next = &cruising;
	cruising.next = &medium;
	medium.next = &high;
	high.next = NULL;
	

	
	// DECLARATION OF PWM OUTPUT VARIABLE
	//////////////////////////////////////////////////////////////////////////////
	
	struct io_type pwm; strcpy(pwm.name, "pwm");
	
	// Set MFs
	struct mf_type noSpeed;
	MATLAB_MF(&noSpeed, "noSpeed", 2739, 2740, 2740, 2815);
	struct mf_type slow;
	MATLAB_MF(&slow, "slow", 2825, 2830, 2830, 2835);
	struct mf_type cruise;
	MATLAB_MF(&cruise, "cruise", 2830, 2835, 2835, 2840);
	struct mf_type medHigh;
	MATLAB_MF(&medHigh, "medHigh", 2830, 2840, 2840, 2850);
	struct mf_type max;
	MATLAB_MF(&max, "max", 2835, 2845, 28545, 2855);
	
	// Linked list for MFs
	pwm.membership_functions = &noSpeed;
	noSpeed.next = &slow;
	slow.next = &cruise;
	cruise.next = &medHigh;
	medHigh.next = &max;
	max.next = NULL;
	
	
	// SETTING I/O LINKED LISTS
	//////////////////////////////////////////////////////////////////////////////
	
	System_Inputs = &distance;
	distance.next = &speed;
	speed.next = NULL;
	
	System_Outputs = &pwm;
	pwm.next = NULL;
	

	
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
	struct rule_type rule12; rule11.next = &rule12; rule12.next = NULL;
	
	
	// RULE SETUP
	//////////////////////////////////////////////////////////////////////////////
	
	// if distance is stopDist then pwm is noSpeed
	struct rule_element_type if11, then1;
	rule1.if_side = &if11; if11.next = NULL; rule1.then_side = &then1; then1.next = NULL;
	if11.value = &stopDist.value; then1.value = &noSpeed.value;
	
	// if speed is low and distance is oneM then speed is slow    struct rule_element_type if21, if22, then2;
	struct rule_element_type if21, if22, then2;
	rule2.if_side = &if21; if21.next = &if22; if22.next = NULL; rule2.then_side = &then2; then2.next = NULL;
	if21.value = &low.value; if22.value = &oneM.value; then2.value = &slow.value;
	
	// if speed is medium and distance is oneM then speed is cruise
	struct rule_element_type if31, if32, then3;
	rule3.if_side = &if31; if31.next = &if32; if32.next = NULL; rule3.then_side = &then3; then3.next = NULL;
	if31.value = &medium.value; if32.value = &oneM.value; then3.value = &cruise.value;
	
	// if speed is high and distance is oneM then speed is cruise
	struct rule_element_type if41, if42, then4;
	rule4.if_side = &if41; if41.next = &if42; if42.next = NULL; rule4.then_side = &then4; then4.next = NULL;
	if41.value = &high.value; if42.value = &oneM.value; then4.value = &cruise.value;
	
	// if speed is still and distance is oneM then speed is slow
	struct rule_element_type if51, if52, then5;
	rule5.if_side = &if51; if51.next = &if52; if52.next = NULL; rule5.then_side = &then5; then5.next = NULL;
	if51.value = &still.value; if52.value = &oneM.value; then5.value = &slow.value;
	
	// if speed is cruising and distance is oneM then speed is slow
	struct rule_element_type if61, if62, then6;
	rule6.if_side = &if61; if61.next = &if62; if62.next = NULL; rule6.then_side = &then6; then6.next = NULL;
	if61.value = &cruising.value; if62.value = &oneM.value; then6.value = &slow.value;
	
	// if speed is low and distance is twoM then speed is cruise
	struct rule_element_type if71, if72, then7;
	rule7.if_side = &if71; if71.next = &if72; if72.next = NULL; rule7.then_side = &then7; then7.next = NULL;
	if71.value = &low.value; if72.value = &twoM.value; then7.value = &cruise.value;
	
	// if speed is medium and distance is twoM then speed is medHigh
	struct rule_element_type if81, if82, then8;
	rule8.if_side = &if81; if81.next = &if82; if82.next = NULL; rule8.then_side = &then8; then8.next = NULL;
	if81.value = &medium.value; if82.value = &twoM.value; then8.value = &medHigh.value;
	
	// if speed is high and distance is twoM then speed is medHigh
	struct rule_element_type if91, if92, then9;
	rule9.if_side = &if91; if91.next = &if92; if92.next = NULL; rule9.then_side = &then9; then9.next = NULL;
	if91.value = &high.value; if92.value = &twoM.value; then9.value = &medHigh.value;
	
	// if speed is still and distance is twoM then speed is cruise
	struct rule_element_type if101, if102, then10;
	rule10.if_side = &if101; if101.next = &if102; if102.next = NULL; rule10.then_side = &then10; then10.next = NULL;
	if101.value = &still.value; if102.value = &twoM.value; then10.value = &cruise.value;
	
	// if speed is cruising and distance is twoM then speed is medHigh
	struct rule_element_type if111, if112, then11;
	rule11.if_side = &if111; if111.next = &if112; if112.next = NULL; rule11.then_side = &then11; then11.next = NULL;
	if111.value = &cruising.value; if112.value = &twoM.value; then11.value = &medHigh.value;
	
	// if distance is threeM then speed is max
	struct rule_element_type if121, then12;
	rule12.if_side = &if121; if121.next = NULL; rule12.then_side = &then12; then12.next = NULL;
	if121.value = &threeM.value; then12.value = &max.value;
	
	
	
	// EXECUTING FUZZY LOGIC & OUTPUT VALUE
	//////////////////////////////////////////////////////////////////////////////
	
	fuzzification();
	rule_evaluation();
	defuzzification();

	if (pwm.value > 2845)
	{
		setESC(2845+adjustment);
	}
	else if (pwm.value < 2750)
	{
		setESC(2750);
	}
	else
	{
		setESC(pwm.value+adjustment);
	}
	
	
	
}

void FLC_obstacle(int currentOCR1A, int midSonicRange, int v)
{
	if(v == 81)
	{
		adjustment = -5;
	}
	else
	{
		adjustment = 0;
	}

	doFuzzy2(currentOCR1A,midSonicRange);
	
}
