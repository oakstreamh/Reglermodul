//////////////////////////////////////////////////////////////////////////////////
// FUZZY SPEED CONTROLLER                                                       //
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
// INCLUDES                                                                     //
//////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include "fuzzy_speed_controller.h"
#include "general_FIS.h"
#include <stdio.h>


//////////////////////////////////////////////////////////////////////////////////
// DEFINITIONS OF I/O AND POINTERS TO TOP OF LISTS                              //
//////////////////////////////////////////////////////////////////////////////////

struct io_type distance;    // input 1
struct io_type speed;       // input 2
struct io_type pwm;         // output
struct rule_type rule1;		// first rule in rule base
struct io_type *speedP;		// pointer to top of inputs' linked list
struct io_type *pwmP;		// pointer to top of output's linked list
struct rule_type *rule1P;	// pointer to first rule in rule base

//////////////////////////////////////////////////////////////////////////////////
// METHODS                                                                      //
//////////////////////////////////////////////////////////////////////////////////


/* set_fuzzySpeedInputs purpose is to initialize the input parameters
 * with the latest measurements from sensor unit before the control loop
 *
 */
void set_fuzzySpeedInputs(v, d)
int v;
int d;
{
    speed.value = v;
    distance.value = d;
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
void FLC_road(void)
{
    
	// Declaration of I/Os
    strcpy(speed.name, "speed");
    strcpy(distance.name, "distance");
    strcpy(pwm.name, "pwm");

    /* MFS FOR THE DISTANCE INPUT VARIABLE
     *
     * From MATLAB-file
     * speed variable interval: [0 300]
     *
     * MF threeM:				[150 200 200 301]
     * MF twoM:					[110 160 172 222]
     * MF oneM:					[22 72 100 150]
     * MF stopDist:				[-1, 0, 12, 62]
     */
    struct mf_type threeM;
    strcpy(threeM.name, "threeM");
    threeM.value = 0;
    threeM.point1 = 150;
    threeM.point2 = 301;
    threeM.slope1 = 2;
    threeM.slope2 = 100;
    threeM.next = NULL;
    
    struct mf_type twoM;
    strcpy(twoM.name, "twoM");
    twoM.value = 0;
    twoM.point1 = 110;
    twoM.point2 = 222;
    twoM.slope1 = 2;
    twoM.slope2 = 2;
    twoM.next = &threeM;
    
    
    struct mf_type oneM;
    strcpy(oneM.name, "oneM");
    oneM.value = 0;
    oneM.point1 = 22;
    oneM.point2 = 150;
    oneM.slope1 = 2;
    oneM.slope2 = 2;
    oneM.next = &twoM;
    
    
    struct mf_type stopDist;
    strcpy(stopDist.name, "stopDist");
    stopDist.value = 0;
    stopDist.point1 = -1;
    stopDist.point2 = 62;
    stopDist.slope1 = 100;
    stopDist.slope2 = 2;
    stopDist.next = &oneM;
    
    
    distance.membership_functions = &stopDist;
    distance.next = NULL;
    
    
    /* MFS FOR THE SPEED INPUT VARIABLE:
     *
     * From MATLAB-file
     * speed variable interval: [2750 2930]
     *
	 * MF still:	[2749 2750 2765 2970]
     * MF low:		[2765 2790 2810 2835]
	 * MF cruising:	[2800 2825 2855 2880]
     * MF medium:	[2845 2878 2892 2925]
     * MF high:		[2890 2900 2930 2931]
     *
     */
    struct mf_type high;
    strcpy(high.name, "high");
    high.value = 0;
    high.point1 = 2890;
    high.point2 = 2931;
    high.slope1 = 10;
    high.slope2 = 100;
    high.next = NULL;
    
    struct mf_type medium;
    strcpy(medium.name, "medium");
    medium.value = 0;
    medium.point1 = 2845;
    medium.point2 = 2926;
    medium.slope1 = 3;
    medium.slope2 = 3;
    medium.next = &high;
    
	struct mf_type cruising;
	strcpy(cruising.name, "cruising");
	cruising.value = 0;
	cruising.point1 = 2800;
	cruising.point2= 2880;
	cruising.slope1 = 4;
	cruising.slope2 = 4;
	cruising.next = &medium;
	
	struct mf_type low;
	strcpy(low.name, "low");
	low.value = 0;
	low.point1 = 2765;
	low.point2= 2835;
	low.slope1 = 4;
	low.slope2 = 4;
	low.next = &cruising;
	
	
    struct mf_type still;
    strcpy(still.name, "still");
    still.value = 0;
    still.point1 = 2749;
    still.point2= 2790;
    still.slope1 = 100;
    still.slope2 = 4;
    still.next = &low;
    
    speed.membership_functions = &still;
    speed.next = &distance;
    
    
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
    struct mf_type noSpeed;
    strcpy(noSpeed.name, "noSpeed");
    noSpeed.value = 0;
    noSpeed.point1 = 2749;
    noSpeed.point2 = 2790;
    noSpeed.slope1 = 100;
    noSpeed.slope2 = 4;
    noSpeed.next = NULL;
    
    struct mf_type slow;
    strcpy(slow.name, "slow");
    slow.value = 0;
    slow.point1 = 2765;
    slow.point2 = 2836;
    slow.slope1 = 4;
    slow.slope2 = 4;
    slow.next = &noSpeed;
    
    struct mf_type cruise;
    strcpy(cruise.name, "cruise");
    cruise.value = 0;
    cruise.point1 = 2800;
    cruise.point2 = 2880;
    cruise.slope1 = 3;
    cruise.slope2 = 3;
    cruise.next = &slow;
    
    struct mf_type medHigh;
    strcpy(medHigh.name, "medHigh");
    medHigh.value = 0;
    medHigh.point1 = 2844;
    medHigh.point2 = 2926;
    medHigh.slope1 = 3;
    medHigh.slope2 = 3;
    medHigh.next = &cruise;
    
    struct mf_type max;
    strcpy(high.name, "max");
    max.value = 0;
    max.point1 = 2890;
    max.point2 = 2931;
    max.slope1 = 4;
    max.slope2 = 100;
    max.next = &medHigh;
    
    pwm.membership_functions = &max;
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
    
    rule1.if_side = &if11;
    rule1.then_side = &then1;
    rule1.next = &rule2;
    
	// pointers to top of lists
    rule1P = & rule1;
    speedP = &speed;
    pwmP = &pwm;
    
	// the methods performing the FLC
    setPointers(speedP, pwmP, rule1P);
    fuzzification();
    rule_evaluation();
    defuzzification();
	setESC(pwm.value);
}

