//////////////////////////////////////////////////////////////////////////////////
// FUZZY SPEED CONTROLLER                                                       //
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
// INCLUDES                                                                     //
//////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include "fuzzy_speed_controller.h"
#include "general_FIS.h"
#include "servo.h"
#include <stdio.h>


//////////////////////////////////////////////////////////////////////////////////
// DEFINITIONS OF I/O AND POINTERS TO TOP OF LISTS                              //
//////////////////////////////////////////////////////////////////////////////////




#define MIN_DISTANCE 0 
#define MAX_DISTANCE 250
#define MIN_SPEED 2740
#define MAX_SPEED 2900


struct io_type *speedP;		// pointer to top of inputs' linked list
struct io_type *pwmP;		// pointer to top of output's linked list
struct rule_type *rule1P;	// pointer to first rule in rule base

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
void FLC_obstacle(int currentOCR1A, int midSonicRange)
{
    
	
	
    
   ///// DECLARATION OF DISTANCE INPUT VARIABLE ///////////////////////////////////
   	struct io_type distance; strcpy(distance.name, "distance");
    
    // Set MFs
    struct mf_type stopDist;
    MATLAB_MF(&stopDist, "stopDist", -1, 0, 12, 52);
    struct mf_type oneM;
    MATLAB_MF(&oneM, "oneM", 30, 60, 60, 95);
    struct mf_type twoM;
    MATLAB_MF(&twoM, "twoM", 75, 125, 125, 175);
    struct mf_type threeM;
    MATLAB_MF(&threeM, "threeM", 150, 250, 250, 251);
    
    // Linked list for MFs
    distance.membership_functions = &stopDist;
    stopDist.next = &oneM;
    oneM.next = &twoM;
    twoM.next = &threeM;
    threeM.next = NULL;
    
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
    
    
    ////// DECLARATION OF SPEED INPUT VARIABLE ///////////////////////////////////
    struct io_type speed; strcpy(speed.name, "speed");
    
    // Set MFs
    struct mf_type still;
    MATLAB_MF(&still, "still", 2739, 2740, 2740, 2815);
    struct mf_type low;
    MATLAB_MF(&low, "low", 2804, 2820, 2820, 2836);
    struct mf_type cruising;
    MATLAB_MF(&cruising, "cruising", 2827, 2843, 2843, 2859);
    struct mf_type medium;
    MATLAB_MF(&medium, "medium", 2849, 2865, 2865, 2881);
    struct mf_type high;
    MATLAB_MF(&high, "high", 2870, 2900, 2900, 2901);
    
    // Linked list for MFs
    speed.membership_functions = &still;
    still.next = &low;
    low.next = &cruising;
    cruising.next = &medium;
    medium.next = &high;
    high.next = NULL;
    
    // Variable assigned it's reference value
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
    
    
    ////// DECLARATION OF PWM OUTPUT VARIABLE ///////////////////////////////////
   	struct io_type pwm; strcpy(pwm.name, "pwm");
    
    // Set MFs
    struct mf_type noSpeed;
    MATLAB_MF(&noSpeed, "noSpeed", 2739, 2740, 2740, 2815);
    struct mf_type slow;
    MATLAB_MF(&slow, "slow", 2804, 2820, 2820, 2836);
    struct mf_type cruise;
    MATLAB_MF(&cruise, "cruise", 2827, 2843, 2843, 2859);
    struct mf_type medHigh;
    MATLAB_MF(&medHigh, "medHigh", 2849, 2865, 2865, 2881);
    struct mf_type max;
    MATLAB_MF(&max, "max", 2870, 2900, 2900, 2901);
    
    // Linked list for MFs
    pwm.membership_functions = &noSpeed;
    noSpeed.next = &slow;
    slow.next = &cruise;
    cruise.next = &medHigh;
    medHigh.next = &max;
    max.next = NULL;
    
    
    
    
    
    
    ////// SETTING I/O LINKED LISTS ///////////////////////////////////
    
    System_Inputs = &distance;
    distance.next = &speed;
    speed.next = NULL;
    
    System_Outputs = &pwm;
    pwm.next = NULL;
    
   
    
    // Declaration of rules and lists
    
    struct rule_type rule1;
    struct rule_type rule2;
    struct rule_type rule3;
    struct rule_type rule4;
    struct rule_type rule5;
    struct rule_type rule6;
    struct rule_type rule7;
    struct rule_type rule8;
    struct rule_type rule9;
    struct rule_type rule10;
    struct rule_type rule11;
    struct rule_type rule12;
    struct rule_type rule13;
    
    
    Rule_Base = &rule1;
    rule1.next = &rule2;
    rule2.next = &rule3;
    rule3.next = &rule4;
    rule4.next = &rule5;
    rule5.next = &rule6;
    rule6.next = &rule7;
    rule7.next = &rule8;
    rule8.next = &rule9;
    rule9.next = &rule10;
    rule10.next = &rule11;
    rule11.next = &rule12;
    rule12.next = &rule13;
    rule13.next = NULL;
    
    // if distance is stopDist then pwm is noSpeed
    struct rule_element_type if11;
    if11.value = &stopDist.value;
    if11.next = NULL;
    struct rule_element_type then1;
    then1.next = NULL;
    then1.value = &noSpeed.value;
    
    rule1.if_side = &if11;
    rule1.then_side = &then1;
    
    // if speed is low and distance is oneM then speed is slow
    struct rule_element_type if22;
    if22.value = &oneM.value;
    if22.next = NULL;
    struct rule_element_type if21;
    if21.value = &low.value;
    if21.next = &if22;
    struct rule_element_type then2;
    then2.value = &slow.value;
    then2.next = NULL;
    
    rule2.if_side = &if21;
    rule2.then_side = &then2;
    
    // if speed is medium and distance is oneM then speed is cruise
    
    struct rule_element_type then3;
    then3.value = &cruise.value;
    then3.next = NULL;
    
    struct rule_element_type if32;
    if32.value = &oneM.value;
    if32.next = NULL;
    
    struct rule_element_type if31;
    if31.value = &medium.value;
    if31.next = &if32;
    
    rule3.if_side = &if31;
    rule3.then_side = &then3;
    
    // if speed is high and distance is oneM then speed is cruise
    struct rule_element_type then4;
    then4.value = &cruise.value;
    then4.next = NULL;
    
    struct rule_element_type if42;
    if42.value = &oneM.value;
    if42.next = NULL;
    
    struct rule_element_type if41;
    if41.value = &high.value;
    if41.next = &if42;
    
    rule4.if_side = &if41;
    rule4.then_side = &then4;
    
    // if speed is still and distance is oneM then speed is slow
    
    struct rule_element_type then5;
    then5.value = &slow.value;
    then5.next = NULL;
    
    struct rule_element_type if52;
    if52.value = &oneM.value;
    if52.next = NULL;
    
    struct rule_element_type if51;
    if51.value = &still.value;
    if51.next = &if52;
    
    rule5.if_side = &if51;
    rule5.then_side = &then5;
    
    // if speed is cruising and distance is oneM then speed is slow
    struct rule_element_type then6;
    then6.value = &slow.value;
    then6.next = NULL;
    
    struct rule_element_type if62;
    if62.value = &oneM.value;
    if62.next = NULL;
    
    struct rule_element_type if61;
    if61.value = &cruising.value;
    if61.next = &if62;
    
    rule6.if_side = &if61;
    rule6.then_side = &then6;
    
    // if speed is low and distance is twoM then speed is cruise
    struct rule_element_type then7;
    then7.value = &cruise.value;
    then7.next = NULL;
    
    struct rule_element_type if72;
    if72.value = &twoM.value;
    if72.next = NULL;
    
    struct rule_element_type if71;
    if71.value = &low.value;
    if71.next = &if72;
    
    rule7.if_side = &if71;
    rule7.then_side = &then7;
    
    // if speed is medium and distance is twoM then speed is medHigh
    
    struct rule_element_type then8;
    then8.value = &medHigh.value;
    then8.next = NULL;
    
    struct rule_element_type if82;
    if82.value = &twoM.value;
    if82.next = NULL;
    
    struct rule_element_type if81;
    if81.value = &medium.value;
    if81.next = &if82;
    
    rule8.if_side = &if81;
    rule8.then_side = &then8;
    
    // if speed is high and distance is twoM then speed is medHigh
    struct rule_element_type then9;
    then9.value = &medHigh.value;
    then9.next = NULL;
    
    struct rule_element_type if92;
    if92.value = &twoM.value;
    if92.next = NULL;
    
    struct rule_element_type if91;
    if91.value = &high.value;
    if91.next = &if92;
    
    rule9.if_side = &if91;
    rule9.then_side = &then9;
    
    // if speed is still and distance is twoM then speed is cruise
    struct rule_element_type then10;
    then10.value = &cruise.value;
    then10.next = NULL;
    
    struct rule_element_type if102;
    if102.value = &twoM.value;
    if102.next = NULL;
    
    struct rule_element_type if101;
    if101.value = &still.value;
    if101.next = &if102;
    
    rule10.if_side = &if101;
    rule10.then_side = &then10;
    
    // if speed is cruising and distance is twoM then speed is medHigh
    struct rule_element_type then11;
    then11.value = &medHigh.value;
    then11.next = NULL;
    
    struct rule_element_type if112;
    if112.value = &twoM.value;
    if112.next = NULL;
    
    struct rule_element_type if111;
    if111.value = &cruising.value;
    if111.next = &if112;
    
    rule11.if_side = &if111;
    rule11.then_side = &then11;
    
    // if distance is threeM then speed is max
    struct rule_element_type then12;
    then12.value = &max.value;
    then12.next = NULL;
    
    struct rule_element_type if121;
    if121.value = &threeM.value;
    if121.next = NULL;
    
    rule12.if_side = &if121;
    rule12.then_side = &then12;
    
    
    
    

    
    
    // the methods performing the FLC
    fuzzification();
    rule_evaluation();
    defuzzification();
    setESC(pwm.value);

} 


