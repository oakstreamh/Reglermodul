
//////////////////////////////////////////////////////////////////////////////////
// main.c contains the ISR, important signals and the control loop.             //
// AUTHORS: MATHIAS DALSHAGEN, HJALMAR EKSTRÖM & SIMON MÅRTENSSON                              //
//                                                                              //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////


//////////////// DEFINITIONS /////////////////////////////////////////////////////

#define F_CPU 14745600

//////////////// INCLUDING PROGRAMS //////////////////////////////////////////////

#include <asf.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "test_methods.h"
#include "servo.h"
#include "manualMode.h"
#include "pid.h"
#include <string.h>
#include "stdint.h"
#include "general_FIS.h"
#include "spi_buffer_slave.h"
#include "counter16b.h"
#include "nFuzzySteering.h"
#include "intersection.h"
#include "FLC_speed.h"
#include "stopLine.h"

//////////////// PROTOTYPES /////////////////////////////////////////////////////////

void carInit(void);

//////////////// VARIABLES ///////////////////////////////////////////////////////
volatile unsigned char spi_rx_not_empty_flag = 0;


/* This method initiates the car. PWM to servos are
* set initial values. There is a five second delay
* to allow the operator to turn on the ESC manually
* and thus set the neutral
*/
void carInit(void)
{
	pwmInit();
	spi_slave_init();
	OCR1A = NEUTRAL;
	OCR1B = STRAIGHT;
	_delay_ms(5000);
	OCR1A = 2900;
	_delay_ms(4000);
	setESC(NEUTRAL);
}



//////////////// MAIN /////////////////////////////////////////////////////////////
int main (void)
{
	straightIntersection(200);
	
	int man_velocity = 0;
	int man_steering = 0;
	carInit();
	
	volatile struct Sensor_information sensor_info;
	struct Sensor_information* sens_info_ptr;
	sens_info_ptr = &sensor_info;
	volatile unsigned char control_mode;
	unsigned char prev_control_mode;
	sei();
	
	isParking = 0;
	
	while (1)
	{
		if (is_package_recieved())
		{
			
			//Reading Information
			prev_control_mode = control_mode;
			read_sensor_info(&control_mode, sens_info_ptr);
			
			int sF = (int) sensor_info.dist_sonic_middle;
			int sB = (int) sensor_info.dist_sonic_back;
			int c = (int) sensor_info.dist_right_line;
			int v = (int) sensor_info.angular_diff;
			int gyro = (int) sensor_info.angle - 125;
			unsigned char type = (unsigned) (char) sensor_info.next_turn_decision;
			int manualInstruction = (int) sensor_info.dist_right_line;
			
			if(control_mode == 0x05 && prev_control_mode == 0x04){
				countInit(30000);
				isParking = 1;
			}
			
			cli();
			
			if(!isParking)
			{
				if (control_mode == 0)
				{
					FLC_speed(OCR1A, sF, OCR1B);
					nFuzzySteering(c,v);
				}
				else if (control_mode == 4)
				{
					setESC(NEUTRAL);
					setServo(STRAIGHT);
				}
				
				else if (control_mode == 1)
				{
					
					FLC_speed(OCR1A, sF, OCR1B);
					intersection(gyro, sensor_info.next_turn_decision, c, v);
				}
				else if (control_mode == 6)
				{
					manualMode(manualInstruction, sF, sB, &man_velocity, &man_steering);
				}
			}
			else if (isParking)
			{
				stop();
			}

			
			sei();
			
			//Sending back information
			unsigned int esc_value_to_send;
			esc_value_to_send = OCR1A;
			unsigned int steering_value_to_send;
			steering_value_to_send = OCR1B;
			//Big endian
			unsigned int temp_ESC;
			temp_ESC = (esc_value_to_send<<8) & 0xFF;
			unsigned int temp_steering;
			temp_steering = (steering_value_to_send<<8) & 0xFF;
			spi_send_byte(0x05);
			spi_send_byte(0x06);
			spi_send_byte(0x07);
			spi_send_byte(0x08);
			
			
		}
	}
}


