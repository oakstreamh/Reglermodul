///*
 //* CFile1.c
 //*
 //* Created: 5/10/2017 2:16:24 PM
 //*  Author: hjaek237
 //*/ 
///* FLC_steering is a fuzzy logic controller to perform lane following.
 //* Input values are unitless reference values derived from image processing.
 //* The image processing detects straight lines through the Hough transform.
 //* To manage sharp curvatures, the image processing application implements a state 
 //* machine.
 //* Input parameters are c and v. Three reserved pairs defines states.
 //* c = 227 & v = 45 corresponds to no lines detected
 //* c = 20 & v = 45 defines the state, sharp right curvature
 //* c = 210 & v = 45 defines the state, sharp left curvature
 //* c in [45, 205] and v in [0, 80] corresponds to the fourth state, straight rode
 //*
 //* The fuzzy logic controller is designed to manage the fourth state
 //*/
//int FLC_steering(int c, int v)
//{
    //
    //
    //
	//if ((c == 1) & (v == 81))        // right curvature, turn right
	//{
		//setServo(MAXRIGHT);
	//}
	//else if ((c == 2) & (v == 81))       // left curvature, turn left
	//{
		//setServo(MAXLEFT);
	//}
	//else                                   // straight road, do fuzzy
	//{
		//
		//
		/////// DECLARATION OF C INPUT VARIABLE ///////////////////////////////////
		//
		//struct io_type delta_C; strcpy(delta_C.name, "delta_C");  
		//
		//struct mf_type rightSide;
		//MATLAB_MF(&rightSide, "rightSide", 49, 50, 110, 120); // Min_value = 50
		//struct mf_type centre;
		//MATLAB_MF(&centre, "centre", 110, 135, 165, 185);
		//struct mf_type leftSide;
		//MATLAB_MF(&leftSide, "leftSide", 175, 200, 230, 231);  // Max_value = 230
		//
		//delta_C.membership_functions = &rightSide;
		//rightSide.next = &centre;
		//centre.next = &leftSide;
		//leftSide.next = NULL;
		//
		//// set iErr's input value to measErr value
		//if(c<0)				// if sensor value is smaller than delta_C's input set's lower limit
		//{
			//delta_C.value = 0;  // force input value to lowest point in delta_C's input set
		//}
		//else if(c>205)			// if sensor value is bigger than delta_C's input set's upper limit
		//{
			//delta_C.value = 205;  // force input value to lowest point in delta_C's input set
		//}
		//else
		//{
			//delta_C.value = c;
		//}
		//
		/////// DECLARATION OF V INPUT VARIABLE ///////////////////////////////////
		//
		//struct io_type delta_V; strcpy(delta_V.name, "delta_V");
		//
		//struct mf_type inMinus;
		//MATLAB_MF(&inMinus, "inMinus", 0, 1, 15, 30); // min V is 1
		//struct mf_type inNyll;
		//MATLAB_MF(&inNyll, "inNyll", 15 , 30, 30, 60);
		//struct mf_type inPlus;
		//MATLAB_MF(&inPlus, "inPlus", 30, 60, 74, 75); // max V is 74
		//
		//delta_V.membership_functions = &inMinus;
		//inMinus.next = &inNyll;
		//inNyll.next = &inPlus;
		//inPlus.next = NULL;
		//
		//// set V's input value to V´s value
		//if(v<1)				// if sensor value is smaller than error's input set lower limit
		//{
			//delta_V.value = 1;  // force input value to lowest point in delta_V's input set
		//}
		//else if(v>74)			// if sensor value is bigger than error's input set's upper limit
		//{
			//delta_V.value = 74;  // force input value to lowest point in error's input set
		//}
		//else
		//{
			//delta_V.value = v;
		//}
		//
		/////// DECLARATION OF STEERING OUTPUT VARIABLE ///////////////////////////////////
//
		//struct io_type steering; strcpy(steering.name, "steering");
		//
		//struct mf_type sharpLeft;
		//MATLAB_MF(&sharpLeft, "sharpLeft", 2259, 2260, 2260, 2360);
		//struct mf_type left;
		//MATLAB_MF(&left, "left", 2360, 2460, 2460, 2560);
		//struct mf_type straight;
		//MATLAB_MF(&straight, "straight", 2560, 2660, 2660, 2760);
		//struct mf_type right;
		//MATLAB_MF(&right, "right", 2760, 2860, 2860, 2960);
		//struct mf_type sharpRight;
		//MATLAB_MF(&sharpRight, "sharpRight", 2960, 3060, 3060, 3061);
		//
		//steering.membership_functions = &sharpRight;
		//sharpRight.next = &right;
		//right.next = &straight;
		//straight.next = &left;
		//left.next = &sharpLeft;
		//sharpLeft.next = NULL;
		//
		//
		//
		//// pointers to top of lists
//
		//System_Inputs = &delta_C;
		//delta_C.next = &delta_V;
		//delta_V.next = NULL;
		//System_Outputs = &steering;
		//steering.next = NULL;
		//
		//
		//struct rule_type rule1; Rule_Base = &rule1;
		//struct rule_type rule2; rule1.next = &rule2;
		//volatile struct rule_type rule3; rule2.next = &rule3;
		//struct rule_type rule4; rule3.next = &rule4;
		//struct rule_type rule5; rule4.next = &rule5;
		//struct rule_type rule6; rule5.next = &rule6;
		//struct rule_type rule7; rule6.next = &rule7;
		//struct rule_type rule8; rule7.next = &rule8;
		//struct rule_type rule9; rule8.next = &rule9; rule9.next = NULL;
		//
		//struct rule_element_type if11, if12, then1;
		//rule1.if_side = &if11; if11.next = &if12; if12.next = NULL; rule1.then_side = &then1; then1.next = NULL;
		//if11.value = &centre.value; if12.value = &inMinus.value; then1.value = &right.value;
		//
		//struct rule_element_type if21, if22, then2;
		//rule2.if_side = &if21; if21.next = &if22; if22.next = NULL; rule2.then_side = &then2; then2.next = NULL;
		//if21.value = &rightSide.value; if22.value = &inMinus.value; then2.value = &left.value;
		//
		//volatile struct rule_element_type if31, if32;
		//rule3.if_side = &if31; if31.next = &if32; if32.next = NULL; rule3.then_side = &then3; then3.next = NULL;
		//if31.value = &rightSide.value; if32.value = &inPlus.value; then3.value = &straight.value;
		//
		//struct rule_element_type if41, if42, then4;
		//rule4.if_side = &if41; if41.next = &if42; if42.next = NULL; rule4.then_side = &then4; then4.next = NULL;
		//if41.value = &centre.value; if42.value = &inNyll.value; then4.value = &straight.value;
		//
		//struct rule_element_type if51, if52, then5;
		//rule5.if_side = &if51; if51.next = &if52; if52.next = NULL; rule5.then_side = &then5; then5.next = NULL;
		//if51.value = &rightSide.value; if52.value = &inNyll.value; then5.value = &sharpLeft.value;
		//
		//struct rule_element_type if61, if62, then6;
		//rule6.if_side = &if61; if61.next = &if62; if62.next = NULL; rule6.then_side = &then6; then6.next = NULL;
		//if61.value = &leftSide.value; if62.value = &inMinus.value; then6.value = &straight.value;
		//
		//struct rule_element_type if71, if72, then7;
		//rule7.if_side = &if71; if71.next = &if72; if72.next = NULL; rule7.then_side = &then7; then7.next = NULL;
		//if71.value = &leftSide.value; if72.value = &inNyll.value; then7.value = &sharpRight.value;
		//
		//struct rule_element_type if81, if82, then8;
		//rule8.if_side = &if81; if81.next = &if82; if82.next = NULL; rule8.then_side = &then8; then8.next = NULL;
		//if81.value = &leftSide.value; if82.value = &inPlus.value; then8.value = &right.value;
		//
		//struct rule_element_type if91, if92, then9;
		//rule9.if_side = &if91; if91.next = &if92; if92.next = NULL; rule9.then_side = &then9; then9.next = NULL;
		//if91.value = &centre.value; if92.value = &inPlus.value;	then9.value = &left.value;
		//
		//
		//// the methods performing the FLC
		//fuzzification();
		//rule_evaluation();
		//defuzzification();
		//if (steering.value < MAXLEFT)
		//{
			//setServo(MAXLEFT);
		//}
		//else if (steering.value >MAXRIGHT)
		//{
			//setServo(MAXRIGHT);
		//}
		//else
		//{
			//setServo(steering.value);
		//}
		//
	//}
	//return 0;
//}