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
#define MAX_DISTANCE 300
#define MIN_SPEED 2750
#define MAX_SPEED 2930


struct io_type *speedP;		// pointer to top of inputs' linked list
struct io_type *pwmP;		// pointer to top of output's linked list
struct rule_type *rule1P;	// pointer to first rule in rule base

//////////////////////////////////////////////////////////////////////////////////
// METHODS                                                                      //
//////////////////////////////////////////////////////////////////////////////////
void set_newMf(struct mf_type *newMf, char newname[MAXNAME], int p1, int p2, int p3, int p4)
{
    strcpy(newMf->name, newname);
    newMf->value = 0;
    newMf->point1 = p1;
    newMf->point2 = p4;
    newMf->slope1 = (int)100/(p2-p1);
    newMf->slope2 = (int)100/(p4-p3);
}


/* Input the four points given in MATLAB trapezoidal function */
void set_newRule(struct rule_type *newRule, int noOfArgs, int noOfCons, int args[4], int consequences[2])
{
    
    struct rule_element_type *pointer = NULL;
    newRule->if_side = pointer;
    struct rule_element_type *movPointer = NULL;
    
    for (int i = 1; i<noOfArgs ; i++)
    {
        struct rule_element_type temp;
        if (i == 1)
        {
            pointer = &temp;
        }
        else
        {
            movPointer = &temp;
        }
        temp.value = &args[i];
        
        
        
    }
    
    
}



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
void FLC_road(int currentOCR1A, int midSonicRange)
{
    
    
    ////// DECLARATION OF DISTANCE INPUT VARIABLE ///////////////////////////////////
   	struct io_type distance; strcpy(distance.name, "distance");
    
    // Set MFs
    struct mf_type stopDist;
    set_newMf(&stopDist, "stopDist", -1, 0, 12, 62);
    struct mf_type oneM;
    set_newMf(&oneM, "oneM", 22, 72, 100, 150);
    struct mf_type twoM;
    set_newMf(&twoM, "twoM", 110, 160, 172, 222);
    struct mf_type threeM;
    set_newMf(&threeM, "threeM", 150, 200, 300, 301);
    
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
    
    
    ////// DECLARATION OF DISTANCE INPUT VARIABLE ///////////////////////////////////
    struct io_type speed; strcpy(speed.name, "speed");
    
    // Set MFs
    struct mf_type still;
    set_newMf(&still, "still", 2749, 2750, 2765, 2970);
    struct mf_type low;
    set_newMf(&low, "low", 2765, 2790, 2810, 2835);
    struct mf_type cruising;
    set_newMf(&cruising, "cruising", 2800, 2825, 2855, 2880);
    struct mf_type medium;
    set_newMf(&medium, "medium", 2845, 2878, 2892, 2925);
    struct mf_type high;
    set_newMf(&high, "high", 2890, 2900, 2930, 2931);
    
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
    set_newMf(&noSpeed, "noSpeed2", 2749, 2750, 2765, 2790);
    struct mf_type slow;
    set_newMf(&slow, "slow", 2765, 2790, 2811, 2836);
    struct mf_type cruise;
    set_newMf(&cruise, "cruise", 2800, 2833, 2847, 2880);
    struct mf_type medHigh;
    set_newMf(&medHigh, "medHigh", 2844, 2880, 2890, 2926);
    struct mf_type max;
    set_newMf(&max, "max", 2890, 2926, 2934, 2970);
    
    // Linked list for MFs
    pwm.membership_functions = &noSpeed;
    noSpeed.next = &slow;
    slow.next = &cruise;
    cruise.next = &medHigh;
    medHigh.next = &high;
    high.next = NULL;
    
    
    
    
    
    
    ////// SETTING I/O LINKED LISTS ///////////////////////////////////
    
    System_Inputs = &distance;
    distance.next = &speed;
    speed.next = NULL;
    
    System_Outputs = &pwm;
    pwm.next = NULL;
    
    
    
    
    
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
    
    /* rule # 11: if still and oneMeter then slow */
    struct rule_element_type then11;
    then11.value = &slow.value;
    then11.next = NULL;
    
    struct rule_element_type if112;
    if112.value = &oneM.value;
    if112.next = NULL;
    
    struct rule_element_type if111;
    if111.value = &still.value;
    if111.next = &if112;
    
    struct rule_type rule11;
    rule11.if_side = &if111;
    rule11.then_side = &then11;
    rule11.next = NULL;
    
    
    /* rule # 10: if low and twoM then cruise */
    struct rule_element_type then10;
    then10.value = &cruise.value;
    then10.next = NULL;
    
    struct rule_element_type if102;
    if102.value = &twoM.value;
    if102.next = NULL;
    
    struct rule_element_type if101;
    if101.value = &low.value;
    if101.next = &if102;
    
    struct rule_type rule10;
    rule10.if_side = &if101;
    rule10.then_side = &then10;
    rule10.next = &rule11;
    
    /* rule # 9: if threeM then max */
    struct rule_element_type then9;
    then9.value = &max.value;
    then9.next = NULL;
    
    struct rule_element_type if91;
    if91.value = &threeM.value;
    if91.next = NULL;
    
    struct rule_type rule9;
    rule9.if_side = &if91;
    rule9.then_side = &then9;
    rule9.next = &rule10;
    
    /* rule # 8: if high and twoM then medHigh */
    struct rule_element_type then8;
    then8.value = &medHigh.value;
    then8.next = NULL;
    
    struct rule_element_type if82;
    if82.value = &twoM.value;
    if82.next = NULL;
    
    struct rule_element_type if81;
    if81.value = &high.value;
    if81.next = &if82;
    
    struct rule_type rule8;
    rule8.if_side = &if81;
    rule8.then_side = &then8;
    rule8.next = &rule9;
    
    /* rule # 7: if cruising and twoM then medHigh */
    struct rule_element_type then7;
    then7.value = &medHigh.value;
    then7.next = NULL;
    
    struct rule_element_type if72;
    if72.value = &twoM.value;
    if72.next = NULL;
    
    struct rule_element_type if71;
    if71.value = &cruising.value;
    if71.next = &if72;
    
    struct rule_type rule7;
    rule7.if_side = &if71;
    rule7.then_side = &then7;
    rule7.next = &rule8;
    
    
    /* rule # 6: if low and twoM then cruise */
    struct rule_element_type then6;
    then6.value = &cruise.value;
    then6.next = NULL;
    
    struct rule_element_type if62;
    if62.value = &twoM.value;
    if62.next = NULL;
    
    struct rule_element_type if61;
    if61.value = &low.value;
    if61.next = &if62;
    
    struct rule_type rule6;
    rule6.if_side = &if61;
    rule6.then_side = &then6;
    rule6.next = &rule7;
    
    /* rule # 5: if high and oneM then medHigh */
    struct rule_element_type then5;
    then5.value = &medHigh.value;
    then5.next = NULL;
    
    struct rule_element_type if52;
    if52.value = &oneM.value;
    if52.next = NULL;
    
    struct rule_element_type if51;
    if51.value = &high.value;
    if51.next = &if52;
    
    struct rule_type rule5;
    rule5.if_side = &if51;
    rule5.then_side = &then5;
    rule5.next = &rule6;
    
    /* rule # 4: if medium and oneM then cruise */
    struct rule_element_type then4;
    then4.value = &cruise.value;
    then4.next = NULL;
    
    struct rule_element_type if42;
    if42.value = &oneM.value;
    if42.next = NULL;
    
    struct rule_element_type if41;
    if41.value = &medium.value;
    if41.next = &if42;
    
    struct rule_type rule4;
    rule4.if_side = &if41;
    rule4.then_side = &then4;
    rule4.next = &rule5;
    
    /* rule # 3: if cruising and oneM then slow */
    struct rule_element_type then3;
    then3.value = &slow.value;
    then3.next = NULL;
    
    struct rule_element_type if32;
    if32.value = &oneM.value;
    if32.next = NULL;
    
    struct rule_element_type if31;
    if31.value = &cruising.value;
    if31.next = &if32;
    
    struct rule_type rule3;
    rule3.if_side = &if31;
    rule3.then_side = &then3;
    rule3.next = &rule4;
    
    /* rule # 2: if low and oneM then slow */
    struct rule_element_type if22;
    if22.value = &oneM.value;
    if22.next = NULL;
    
    struct rule_element_type if21;
    if21.value = &low.value;
    if21.next = &if22;
    
    struct rule_element_type then2;
    then2.value = &slow.value;
    then2.next = NULL;
    
    struct rule_type rule2;
    rule2.if_side = &if21;
    rule2.then_side = &then2;
    rule2.next = &rule3;
    
    /* rule # 1: if stopDist then noSpeed */
    struct rule_element_type if11;
    if11.value = &stopDist.value;
    if11.next = NULL;
    
    struct rule_element_type then1;
    then1.value = &noSpeed.value;
    then1.next = NULL;
    
    struct rule_type rule1;		// first rule in rule base
    
    rule1.if_side = &if11;
    rule1.then_side = &then1;
    rule1.next = &rule2;
    
    // pointers to top of lists
    Rule_Base = &rule1;
    
    
    // the methods performing the FLC
    fuzzification();
    rule_evaluation();
    defuzzification();
    setESC(pwm.value);

}

