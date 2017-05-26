/*
 * counter16b.h
 *
 * Created: 5/15/2017 10:35:25 AM
 *  Author: hjaek237
 */ 


#ifndef COUNTER16B_H_
#define COUNTER16B_H_

void countInit(int req_delay);
int checkCount(uint16_t req_delay);

volatile uint8_t step;


#endif /* COUNTER16B_H_ */
