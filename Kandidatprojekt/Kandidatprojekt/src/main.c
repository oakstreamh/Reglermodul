
//////////////////////////////////////////////////////////////////////////////////
// main.c contains the ISR, important signals and the control loop.             //
// AUTHORS: MATHIAS DALSHAGEN & HJALMAR EKSTRÖM                                 //
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
//#include <uart_buffer.h>
#include "pid.h"
#include <string.h>
#include "stdint.h"
#include "fuzzy_speed_controller.h"
#include "general_FIS.h"
#include "fuzzySteering.h"
#include "spi_buffer_slave.h"
#include "fuzzyParkingAlgorithm.h"


//////////////// STRUCTS /////////////////////////////////////////////////////////

//struct Sensor_information{
	//unsigned char dist_right_line;
	//unsigned char angular_diff;
	//unsigned char dist_sonic_middle;
	//unsigned char dist_sonic_left;
	//unsigned char dist_sonic_right;
	//unsigned char dist_sonic_back;
	//unsigned char car_speed;
	//unsigned char angle;
	//unsigned char dist_to_stop_line;
	//unsigned char sign_type; //Not sure we gonna use this one. Depends if camera can detect signs
//};


//////////////// HEADERS /////////////////////////////////////////////////////////


void carInit(void);
void USART1_init(unsigned int baud_setting);
void Sens_info_read(struct Sensor_information*);
void Init(void);
int16_t Get_Measurement(void);
int16_t Get_Reference(void);



//////////////// VARIABLES ///////////////////////////////////////////////////////
volatile unsigned char spi_rx_not_empty_flag = 0;


int elapsedSeconds = 0;


// Declaration of input signals
int carInitialized = 0;
int drivingMode[6] = {0,0,0,0,0,0}; // if index i is set to 1 => driving mode i as specified by {courtyard,road,intersection,parkingLot,garage,manual}
int speedCounter = 0;
int speedInterrupt = 0;
double circleArch = 0.053;


//--------UART Variables (temp)------------
volatile unsigned char UART1_reciever_buffer[32];
volatile int counter_UART1_reciever;
//----------Ending of UART Variables (temp)-----------


ISR(USART1_RX_vect){
	/*
	*This buffer is specialized for sensorinformation.
	*May overwrite existing data in buffer
	*
	*/
	
	unsigned char in_value;
	
	//Disabling interrupts
	//cli();

	
	in_value = UDR1;
	
	//checking if header (0xFF)
	if (in_value ==  0xFF){ // to check if in_value==0xFF
		counter_UART1_reciever = 0;
	}
	else{
		//Write value to buffer
		UART1_reciever_buffer[counter_UART1_reciever] = in_value;
		counter_UART1_reciever++;
	}

	
	//Enabling interrupts
	//sei();

	
}



/*
* SPI port definitions
* PB7 SCK (Master clock)
* PB6 Slave output (MISO)
* PB5 Slave input (MOSI)
* PB4 Slave select (SS)
*/




/* This method initiates the car. PWM to servos are
* set initial values. There is a five second delay
* to allow the operator to turn on the ESC manually
* and set the neutral
*/
void carInit(void)
{
	pwmInit();
	spi_slave_init();
	setESC(NEUTRAL);
	setServo(STRAIGHT);
}





void USART1_init(unsigned int baud_setting)
{
	//UART enabling:
	UBRR1 = 0;
	//Enabling reciever and disabling transmitter interrupts
	UCSR1B = (1<<RXEN1) | (0<<TXEN1) | (1<<RXCIE1) | (0<<TXCIE1)| (0<<UDRIE1);
	//Set frame format: 8data, 2 stop bit
	UCSR1C = (1<<USBS1) | (3<<UCSZ10);
	//Setting baud rateS
	UBRR1 = baud_setting;
}


void Sens_info_read(struct Sensor_information* sens_info_ptr) //There is no check if the buffer is empty or not
{
	
	//Disable UART1 interrupts to prevent values from changing while reading
	UCSR1B &= ~(1<<RXCIE1);
	
	//Assigning values from buffer to sens_info
	sens_info_ptr->dist_right_line = (unsigned) (char) UART1_reciever_buffer[4];
	sens_info_ptr->angular_diff = (unsigned) (char) UART1_reciever_buffer[5];
	sens_info_ptr->dist_sonic_right = (unsigned) (char) UART1_reciever_buffer[2];
	sens_info_ptr->dist_sonic_middle = (unsigned) (char) UART1_reciever_buffer[0];
	sens_info_ptr->dist_sonic_left = (unsigned) (char) UART1_reciever_buffer[1];
	sens_info_ptr->dist_sonic_back = (unsigned) (char) UART1_reciever_buffer[3];
	
	
	//sens_info_ptr->dist_sonic_right = ((unsigned) (short) UART1_reciever_buffer[7] << 8) | (unsigned) (short) UART1_reciever_buffer[6];
	
	//sens_info_ptr->ang_acc = ((unsigned) (short) UART1_reciever_buffer[11] << 8) | (unsigned) (short) UART1_reciever_buffer[10];
	//sens_info_ptr->car_speed = ((unsigned) (short) UART1_reciever_buffer[13] << 8) | (unsigned) (short) UART1_reciever_buffer[12];
	//sens_info_ptr->dist_to_stop_line = ((unsigned) (short) UART1_reciever_buffer[15] << 8) | (unsigned) (short) UART1_reciever_buffer[14];
	//sens_info_ptr->sign_type = UART1_reciever_buffer[16];
	
	counter_UART1_reciever = 0; //To be able to use counter as if-thing for reading
	//Enable UART1 interrupts
	UCSR1B |= (1<<RXCIE1);
}





int main (void)
{
	// FOR TESTING
	//	FLC_obstacle(2800, 150);

	carInit();
	_delay_ms(5000);


	
	sei();
	
	
	
	//-----Variables and pointers for Sensor information
	//Er info finns i sensor_info.dist_right_line;
	//om counter_UART1_reciever true, finns info att hemta
	
	
	struct Sensor_information sensor_info;
	struct Sensor_information* sens_info_ptr;
	sens_info_ptr = &sensor_info;
	unsigned char control_mode;
	//--end of sensor information
	
	//Init for UART
	unsigned int baud_setting = 7;
	USART1_init(baud_setting);
	//End of init for UART
	
	//Setting for Testing
	//DDRA = 0xFF;
	//End of test setting
	
	while (1) {
		
		if (counter_UART1_reciever > 5) {
			
			PORTA |= (1<<PORTA1);
			
			//Reading Information
			//read_sensor_info(&control_mode, sens_info_ptr);
			Sens_info_read(sens_info_ptr);
			
			int sR = (int) sensor_info.dist_sonic_right;
			int sF = (int) sensor_info.dist_sonic_middle;
			int sL = (int) sensor_info.dist_sonic_left;
			int sB = (int) sensor_info.dist_sonic_back;
			
			int c = (int) sensor_info.dist_right_line;
			int v = (int) sensor_info.angular_diff;
			
			cli();
			
			FLC_obstacle(OCR1A, sF);
			FLC_steering(c,v);

			sei();
			
			//Sending back information
			//unsigned int esc_value_to_send;
			//esc_value_to_send = (unsigned) (short) OCR1A;
			//unsigned int steering_value_to_send;
			//steering_value_to_send = (unsigned) (short) OCR1B;
			//Big endian
			//spi_send_byte((unsigned) (char) (esc_value_to_send<<8));
			//spi_send_byte((unsigned) (char) (esc_value_to_send));
			//spi_send_byte((unsigned) (char) (steering_value_to_send<<8));
			//spi_send_byte((unsigned) (char) (steering_value_to_send));
		}
	}
}