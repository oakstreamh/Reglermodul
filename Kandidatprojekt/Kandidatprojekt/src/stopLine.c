/*
* stopLine.c
*
* Created: 5/12/2017 12:55:09 PM
*  Author: hjaek237
*/

#include "servo.h"
#include <string.h>
#include "fuzzySteering.h"
#include "general_FIS.h"
#include <stdio.h>




void stop();
{
    
    if (overflow<1)
    {
        setESC(2835);
        setServo(MAXRIGHT);
    }
    else if (overflow<2)
    {
        setESC(2835);
        setServo(MAXLEFT);
    }
    else if (overflow<3)
    {
        setESC(NEUTRAL);
        setServo(STRAIGHT);
    }
    else if (overflow<4)
    {
        setESC(2835);
        setServo(MAXRIGHT);
    }
}
