


//////////////////////////////////////////////////////////////////////////////////
// general_FIS.c contains the functions required for performing FLC.            //
// The code for the FIS was not written by any member of the project. It was    //
// fetched from: http://www.drdobbs.com/cpp/fuzzy-logic-in-c/184408940          //
//                                                                              //
// The code has been modified to fit our purposes.                              //
//                                                                              //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

//  To build a FLC. Use Matlabs fuzzy logic application for simple analysis
//  and adjustment of the controller. The FIS is composed of data structures
//  that are linked together as lists. The methods fuzzification(),
//  rule_evaluation() and defuzzification() runs through the linked list to
//  transform the input values to a corresponding output value.
//
//  In the beginning of every execution the pointers System_Inputs,
//  System_outputs and Rule_Base must be pointed to the top of the corresponding
//  linked list.
//
//  Visit the web-link for more detailed information.


#include <stdio.h>
#include "general_FIS.h"
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////
// VARIABLES & HEADERS                                                          //
//////////////////////////////////////////////////////////////////////////////////

volatile struct io_type *System_Outputs;
volatile struct io_type *System_Inputs;
volatile struct rule_type *Rule_Base;
int max(int arg1, int arg2);
int min(int arg1, int arg2);
void compute_degree_of_membership(mf,input);
int compute_area_of_trapezoid(mf);


//////////////////////////////////////////////////////////////////////////////////
// METHODS TO PERFORM MAX AND MIN CALCULATIONS                                  //
//////////////////////////////////////////////////////////////////////////////////

/* Returns the maximal value of two integer arguments 
 */
int max(int arg1, int arg2)
{
    if (arg1>arg2)
    {
        return(arg1);
    }
    else
    {
        return(arg2);
    }
}

/* Returns the minimal value of two integer arguments 
 */
int min(arg1,arg2)
int arg1;
int arg2;
{
    if (arg1<arg2)
    {
        return(arg1);
    }
    else
    {
        return(arg2);
    }
}


//////////////////////////////////////////////////////////////////////////////////
// COMPUTE DEGREE OF MEMBERSHIP                                                 //
//////////////////////////////////////////////////////////////////////////////////

/* Degree to which input is a member of mf is calculated as follows: 
 *
 * 1.     Compute delta terms to determine if input is inside or outside
 *        membership function.
 * 2.     If outside, then degree of membership is 0. Otherwise, smaller
 *        of delta_1 * slope1 and delta_2 * slope2 applies.
 * 3.     Enforce upper limit.
 *
 */
void compute_degree_of_membership(mf,input)
struct mf_type *mf;
int input;
{
    int delta_1;
    int delta_2;
    delta_1 = input - mf->point1;
    delta_2 = mf->point2 - input;
    if ((delta_1 <= 0) || (delta_2 <= 0))   /* input outside mem. function ?  */
        mf->value = 0;                           /* then degree of membership is 0 */
        else
            mf->value = min( (mf->slope1*delta_1),(mf->slope2*delta_2) );
            mf->value = min(mf->value,UPPER_LIMIT);  /* enforce upper limit */
            }


//////////////////////////////////////////////////////////////////////////////////
// COMPUTE AREA OF TRAPEZOID                                                    //
//////////////////////////////////////////////////////////////////////////////////

/* Each inference pass produces a new set of output strengths which affect 
 * the areas of trapezoidal membership functions used in center-of-gravity
 * defuzzification.
 *
 * Area values must be recalculated with each pass. Area of trapezoid is
 * h*(a+b)/2 where:
 *    h=height=output_strength=mf->value
 *    b=base=mf->point2-mf->point1
 *    a=f(h,b, mf->slopes1, mf->slope2)
 */
int compute_area_of_trapezoid(mf)
struct mf_type *mf;
{
    int run_1;
    int run_2;
    int base;
    int top;
    int area;
    base = mf->point2 - mf->point1;
    run_1 = mf->value/mf->slope1;
    run_2 = mf->value/mf->slope2;
    top = base - run_1 - run_2;
    area = mf->value * ( base + top)/2;
    return(area);
}


//////////////////////////////////////////////////////////////////////////////////
// FUZZIFICATION                                                                //
//////////////////////////////////////////////////////////////////////////////////

/* Degree of membership value is calculated for each membership function of 
 * each system input. Values correspond to antecedents in rules. 
 */
void fuzzification()
{
    struct io_type *si;    /* system input pointer        */
    struct mf_type *mf;    /* membership function pointer */
    for(si=System_Inputs; si != NULL; si=si->next)
        for(mf=si->membership_functions; mf != NULL; mf=mf->next)
            if (mf!=NULL) {
                compute_degree_of_membership(mf,si->value);
            }
}


//////////////////////////////////////////////////////////////////////////////////
// RULE EVALUATION                                                              //
//////////////////////////////////////////////////////////////////////////////////

/* Each rule consists of a list of pointers to antecedents (if side), list
 * of pointers to outputs (then side), and pointer to next rule
 * in rule base. 
 *
 * When a rule is evaluated, its antecedents are ANDed together, using a 
 * minimum function, to form strength of rule. Then strength is applied
 * to each of listed rule outputs. If an output has already been assigned a rule
 * strength, during current inference pass, a maximum function is used to
 * determine which strength should apply. 
 */
void rule_evaluation()
{
    struct rule_type *rule;
    struct rule_element_type *ip;       /* pointer of antecedents  (if-parts)   */
    struct rule_element_type *tp;       /* pointer to consequences (then-parts) */
    int strength;                /* strength of  rule currently being evaluated */
    for(rule=Rule_Base; rule != NULL; rule=rule->next){
        strength = UPPER_LIMIT;                       /* max rule strength allowed */
        /* process if-side of rule to determine strength */
        for(ip=rule->if_side; ip != NULL; ip=ip->next)
            strength = min(strength,*(ip->value));
        /* process then-side of rule to apply strength */
        for(tp=rule->then_side; tp != NULL; tp=tp->next)
            *(tp->value) = max(strength,*(tp->value));
    }
}


//////////////////////////////////////////////////////////////////////////////////
// DEFUZZIFICATION                                                              //
//////////////////////////////////////////////////////////////////////////////////

/* Crisp output values are calculated through the centroid method (center of 
 * gravity). For each system output the corresponding output membership function
 * strength are used to calculate the sum of areas that constitutes the basis
 * of centroid calculations
 */
void defuzzification()
{
    struct io_type *so;    /* system output pointer */
    struct mf_type *mf;    /* output membership function pointer */
    double sum_of_products;   /* sum of products of area & centroid */
    double sum_of_areas;  /* sum of shortend trapezoid area */
    double area;
    double centroid;
	double result;
    /* compute a defuzzified value for each system output */
    for(so=System_Outputs; so != NULL; so=so->next){
        sum_of_products = 0;
        sum_of_areas = 0;
		centroid = 0;
		result = 0;
        for(mf=so->membership_functions; mf != NULL; mf=mf->next){
            area = compute_area_of_trapezoid(mf);
			centroid = mf->point1;
			int intermed = (int) ((mf->point2 - mf->point1)/2);
            centroid = centroid + intermed;
            sum_of_products += area * centroid;
            sum_of_areas += area;
			result = (sum_of_products/sum_of_areas); /* weighted average */
        }
        so->value = (int) result;  
    }
}


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

/* Creates a linked list and assigns values according to inputs */
void setupRule(struct rule_type *rule, struct rule_element_type *elements[], int* args[], int* cons[])
{
	
	int noArgs = (int)(*(&args+1)-args);
	int noCons = (int)(*(&cons+1)-cons);
	
	rule->if_side = elements[0];
	
	for (int i = 1; i<= noArgs ; i++)
	{
		elements[i-1]->value = args[i-1];
		if (i==noArgs) {
			elements[i-1]->next = NULL;
		}
		else
		{
			elements[i-1]->next = elements[i];
		}
	}
	
	rule->then_side = elements[noArgs];
	
	for (int j = noArgs ; j <= noArgs+noCons-1 ; j++)
	{
		elements[j]->value = cons[j-noArgs];
		if (j==noCons+noArgs-1) {
			elements[j]->next = NULL;
		}
		else
		{
			elements[j]->next = elements[j+1];
		}
		
	}
}