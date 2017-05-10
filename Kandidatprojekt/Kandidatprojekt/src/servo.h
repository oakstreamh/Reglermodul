#ifndef SERVO_H_
#define SERVO_H_


#define MAXESC 2875		// MAX/MIN limit for driving on the floor
#define MINESC 2655

#define MAXLEFT 2022	// Limits before servo is exhausted
#define MAXRIGHT 3300

#define NEUTRAL 2765
#define STRAIGHT 2660

void setServo(int counterServo);
void setESC(int counterESC);
void pwmInit(void);

#endif

