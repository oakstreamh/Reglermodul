
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



// FLC OBSTACLE AVOIDER
//////////////////////////////////////////////////////////////////////////////////


void FLC_obstacle(int currentServo, int midSonicRange)
{
    // Inputs
    struct io_type distance; strcpy(distance.name, "distance");
    struct io_type steering; strcpy(steering.name, "steering");
    
    // Output
    struct io_type speed; strcpy(speed.name, "speed");

    
    // Variable assigned its reference value
    if (currentServo<MAXLEFT) {
        steering.value = (int) MAXLEFT / 10;
    }
    else if (currentServo>MAXRIGHT)
    {
        steering.value = (int) MAXRIGHT / 10;
    }
    else
    {
        steering.value = (int) currentServo / 10;
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
    struct mf_type right;
    MATLAB_MF(&right, "right", 276, 330, 330, 331);
    struct mf_type straight;
    MATLAB_MF(&straight, "straight", 246, 266, 266, 286);
    struct mf_type left;
    MATLAB_MF(&left, "left", 201, 202, 256, 256);
    
    // Linked list for MFs
    steering.membership_functions = &right;
    right.next = &straight;
    straight.next = &left;
    left.next = NULL;
    
    
    // DECLARATION OF DISTANCE INPUT VARIABLE
    //////////////////////////////////////////////////////////////////////////////
    
    // Set MFs
    struct mf_type stopDist;
    MATLAB_MF(&stopDist, "stopDist", MIN_DISTANCE-1, MIN_DISTANCE, 15, 30);
    struct mf_type close;
    MATLAB_MF(&close, "close", 5, 35, 35, 70);
    struct mf_type faar;
    MATLAB_MF(&faar, "faar", 50, 125, MAX_DISTANCE, MAX_DISTANCE+1);
    
    
    // Linked list for MFs
    distance.membership_functions = &stopDist;
    stopDist.next = &close;
    close.next = &faar;
    faar.next = NULL;
    
    
    // DECLARATION OF SPEED OUTPUT VARIABLE
    //////////////////////////////////////////////////////////////////////////////
    
    
    // Set MFs
    struct mf_type neutral;
    MATLAB_MF(&neutral, "neutral", 2764, 2765, 2765, 2826);
    struct mf_type low;
    MATLAB_MF(&low, "low", 2830, 2840, 2840, 28450);
    struct mf_type high;
    MATLAB_MF(&high, "high", MAXESC-20, MAXESC, MAXESC, MAXESC+20);
    
    // Linked list for MFs
    speed.membership_functions = &neutral;
    neutral.next = &low;
    low.next = &high;
    high.next = NULL;
    
    
    // SETTING I/O LINKED LISTS
    //////////////////////////////////////////////////////////////////////////////
    
    System_Inputs = &distance;
    distance.next = &steering;
    steering.next = NULL;
    
    System_Outputs = &speed;
    speed.next = NULL;
    
    
    // DECLARATION OF RULES AND LISTS
    //////////////////////////////////////////////////////////////////////////////
    
    struct rule_type rule1; Rule_Base = &rule1;
    struct rule_type rule2; rule1.next = &rule2;
    struct rule_type rule3; rule2.next = &rule3;
    struct rule_type rule4; rule3.next = &rule4;
    struct rule_type rule5; rule4.next = &rule5; rule5.next = NULL;
    
    
    // RULE SETUP
    //////////////////////////////////////////////////////////////////////////////
    
    // if distance is stopDist then speed is neutral
    struct rule_element_type if11, then1;
    rule1.if_side = &if11; if11.next = NULL; rule1.then_side = &then1; then1.next = NULL;
    if11.value = &stopDist.value; then1.value = &neutral.value;
    
    // if dist is close then speed is slow
    struct rule_element_type if21, then2;
    rule2.if_side = &if21; if21.next = NULL; rule2.then_side = &then2; then2.next = NULL;
    if21.value = &close.value; then2.value = &low.value;
    
    // if dist is faar and steering is right then speed is slow
    struct rule_element_type if31, if32, then3;
    rule3.if_side = &if31; if31.next = &if32; if32.next = NULL; rule3.then_side = &then3; then3.next = NULL;
    if31.value = &faar.value; if32.value = &right.value; then3.value = &low.value;
    
    // if dist is faar and steering is left then speed is slow
    struct rule_element_type if41, if42, then4;
    rule4.if_side = &if41; if41.next = &if42; if42.next = NULL; rule4.then_side = &then4; then4.next = NULL;
    if41.value = &faar.value; if42.value = &left.value; then4.value = &low.value;
    
    // if dist is faar and steering is straight then speed is high
    struct rule_element_type if51, if52, then5;
    rule5.if_side = &if51; if51.next = &if52; if52.next = NULL; rule5.then_side = &then5; then5.next = NULL;
    if51.value = &faar.value; if52.value = &straight.value; then5.value = &high.value;
    
    
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
