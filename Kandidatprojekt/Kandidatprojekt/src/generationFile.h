//
//  generationFile.h
//  homeWork
//
//  Created by Mathias Dalshagen on 2017-04-25.
//  Copyright © 2017 Mathias Dalshagen. All rights reserved.
//

#ifndef generationFile_h
#define generationFile_h

#include <stdio.h>


void MATLAB_MF(struct mf_type *newMf, char newname[MAXNAME], int p1, int p2, int p3, int p4);
void set_newRule(struct rule_type *newRule, int* args[], int noOfArgs, int* cons[], int noOfCons);


#endif /* generationFile_h */
