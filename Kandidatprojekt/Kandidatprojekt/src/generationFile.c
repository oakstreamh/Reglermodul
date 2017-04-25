//
//  generationFile.c
//  homeWork
//
//  Created by Mathias Dalshagen on 2017-04-25.
//  Copyright © 2017 Mathias Dalshagen. All rights reserved.
//

#include "generationFile.h"


//////////////////////////////////////////////////////////////////////////////////
// MATLAB MF FUNCTIONS                                                          //
//////////////////////////////////////////////////////////////////////////////////

/* This function takes four parameters defining a trapezoid or triangular mf
 * function in MATLAB's tool for fuzzy logic design and returns a MF according
 * to the format in general_FIS.c with two end points and two slopes
 */
void MATLAB_MF(struct mf_type *newMf, char newname[MAXNAME], int p1, int p2, int p3, int p4)
{
    strcpy(newMf->name, newname);
    newMf->value = 0;
    newMf->point1 = p1;
    newMf->point2 = p4;
    newMf->slope1 = (int)UPPER_LIMIT/(p2-p1);
    newMf->slope2 = (int)UPPER_LIMIT/(p4-p3);
}

//////////////////////////////////////////////////////////////////////////////////
// RULE GENERATION                                                              //
//////////////////////////////////////////////////////////////////////////////////



void pushElement(struct rule_element_type **head, int *val){
    struct rule_element_type *new_element = (struct rule_element_type*) malloc (sizeof (struct rule_element_type));
    new_element->value = val;
    new_element->next = *head;
    *head = new_element;
}


/* Input the four points given in MATLAB trapezoidal function */
void set_newRule(struct rule_type *newRule, int* args[], int noOfArgs, int* cons[], int noOfCons)
{
    
    for (int i = 1; i<= noOfArgs ; i++)
    {
        pushElement(&newRule->if_side, args[i-1]);
    }
    
    
    for (int j = 1; j<= noOfCons ; j++)
    {
        pushElement(&newRule->then_side, cons[j-1]);
    }
    
    
}

