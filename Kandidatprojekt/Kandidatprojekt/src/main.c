
/**
 * \file
 *
 * \Styrmodulen
 *
 */

#define F_CPU 14745600

#include <asf.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <testmanual.h>
#include "servo.h"
#include "manualMode.h"
//#include <uart_buffer.h>
#include "pid.h"
#include <string.h>
#include "stdint.h"
#include "fuzzy_speed_controller.h"
#include "general_FIS.h"

struct Sensor_information{
	unsigned char dist_right_line;
	unsigned short dist_sonic_middle;
	unsigned short dist_sonic_left;
	unsigned short dist_sonic_right;
	unsigned short dist_sonic_back;
	unsigned short ang_acc;
	unsigned short car_speed;
	unsigned short dist_to_stop_line;
	unsigned char sign_type; //Not sure we gonna use this one. Depends if camera can detect signs
};



void USART1_init(unsigned int baud_setting);

// Headers for methods
void SPI_slaveInit(void);

void carInit(void);

void Sens_info_read(struct Sensor_information*);



int elapsedSeconds = 0;
int SPI_FLAG = 0;

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
void SPI_slaveInit(void)
{
	DDRB = (1<<DDB6); // Set MISO output, all others input
	SPCR = (1<<SPE)|(1<<SPIE); // Enables SPI
}



void carInit(void)
{
	pwmInit();
	SPI_slaveInit();
	setESC(NEUTRAL);
	setServo(STRAIGHT);
	_delay_ms(5000);
}





void USART1_init(unsigned int baud_setting)
{
	//UART enabling:
	UBRR1 = 0;
	//Enabling reciever and disabling transmitter interrupts
	UCSR1B = (1<<RXEN1) | (0<<TXEN1) | (1<<RXCIE1) | (0<<TXCIE1)| (0<<UDRIE1);
	//Set frame format: 8data, 2 stop bit
	UCSR1C = (1<<USBS1) | (3<<UCSZ10);
	//Setting baud rate
	UBRR1 = baud_setting;
}


void Sens_info_read(struct Sensor_information* sens_info_ptr) //There is no check if the buffer is empty or not
{
	
	//Disable UART1 interrupts to prevent values from changing while reading
	UCSR1B &= ~(1<<RXCIE1);
	
	//Assigning values from buffer to sens_info
	sens_info_ptr->dist_right_line = (unsigned) (char) UART1_reciever_buffer[0];
	//sens_info_ptr->dist_sonic_middle = ((unsigned) (short) UART1_reciever_buffer[3] << 8) | (unsigned) (short) UART1_reciever_buffer[2];
	//sens_info_ptr->dist_sonic_left = ((unsigned) (short) UART1_reciever_buffer[5] << 8) | (unsigned) (short) UART1_reciever_buffer[4];
	//sens_info_ptr->dist_sonic_right = ((unsigned) (short) UART1_reciever_buffer[7] << 8) | (unsigned) (short) UART1_reciever_buffer[6];
	//sens_info_ptr->dist_sonic_back = ((unsigned) (short) UART1_reciever_buffer[9] << 8) | (unsigned) (short) UART1_reciever_buffer[8];
	//sens_info_ptr->ang_acc = ((unsigned) (short) UART1_reciever_buffer[11] << 8) | (unsigned) (short) UART1_reciever_buffer[10];
	//sens_info_ptr->car_speed = ((unsigned) (short) UART1_reciever_buffer[13] << 8) | (unsigned) (short) UART1_reciever_buffer[12];
	//sens_info_ptr->dist_to_stop_line = ((unsigned) (short) UART1_reciever_buffer[15] << 8) | (unsigned) (short) UART1_reciever_buffer[14];
	//sens_info_ptr->sign_type = UART1_reciever_buffer[16];
	
	counter_UART1_reciever = 0; //To be able to use counter as if-thing for reading
	//Enable UART1 interrupts
	UCSR1B |= (1<<RXCIE1);
}

#define K_P 1.00
#define K_D 0.00

struct GLOBAL_FLAGS {
	//! True when PID control loop should run one time
	uint8_t pidTimer : 1;
	uint8_t dummy : 7;
	} gFlags = {0, 0};
	
	struct PID_DATA pidData;

#define TIME_INTERVAL 38555 //EXAMPLE //TODO

ISR(TIMER0_OVF_vect)
{
	static uint16_t i = 0;

	if (i < TIME_INTERVAL) {
		i++;
		} else {
		gFlags.pidTimer = TRUE;
		i               = 0;
	}
}

void Init(void)
{
	pid_Init(K_P * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);

	// Set up timer, enable timer/counter 0 overflow interrupt
	TCCR0B = (1 << CS00); // clock source to be used by the Timer/Counter clkI/O
	TIMSK0 = (1 << TOIE0);
	TCNT0  = 0;
}

int16_t Get_Reference(void) //TODO
{
	return 125;
}

int16_t Get_Measurement(void) //TODO
{
	return 140;
}

int main (void)
{
	/*
		Init();
		setESC(MANUAL_FORWARD);
		uint8_t referenceValue, measurementValue = 0;
		int8_t inputValue = 0;
		
		referenceValue =   Get_Reference();
		measurementValue = Get_Measurement();
		
		inputValue = pid_Controller(referenceValue, measurementValue, &pidData);
		setServo(STRAIGHT+inputValue);
	
	*/
	
	
	
		carInit();

		
		uint16_t referenceValue, measurementValue, inputValue = 0;
		//TCNT0 =255;
		sei();
		DDRA = 0xFF;
				
		//-----Variables and pointers for Sensor information
		//Er info finns i sensor_info.dist_right_line;
		//om counter_UART1_reciever true, finns info att hemta
		
		struct Sensor_information sensor_info;
		struct Sensor_information* sens_info_ptr;
		sens_info_ptr = &sensor_info;
		//--end of sensor information
			
		//Init for UART
		unsigned int baud_setting = 7;
		USART1_init(baud_setting);
		//End of init for UART
		
		
		
		
		int distance;
		
		while (1) {
			
			if (counter_UART1_reciever) {			
				
			Sens_info_read(sens_info_ptr);
						
			PORTA = sensor_info.dist_right_line;			
			gFlags.pidTimer = 1;
			
			}
			if (gFlags.pidTimer == 1) {
				cli();
		
				referenceValue   = 125;
				distance = (int) sensor_info.dist_right_line;
				set_fuzzySpeedInputs(lastESC, distance);
				FLC_road();
				// inputValue = pid_Controller(referenceValue, measurementValue, &pidData);

				sei();
				
				
			}
		}
}


