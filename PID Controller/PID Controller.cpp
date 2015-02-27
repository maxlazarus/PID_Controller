/*
 * PID_Controller.cpp
 *
 * Created: 1/23/2015 9:28:17 AM
 *  Author: Max Prokopenko
 *  max@theprogrammingclub.com
 *
 *  NOPE
 *  Nano pinout:
 *	 D2:	Hall Effect input
 *   D6:	PWM0 out
 */ 

#include "F_CPU.h"

#define CONTROL_PORT PORTD
#define HCTL_BYTE_SELECT_BIT 2
#define HCTL_ENABLE_BIT 4
#define MOTOR_BIT_1 5 // soft serial
#define HCTL_CLK_BIT 6
#define DATA_HIGH_NIBBLE (PINC << 4)
#define DATA_LOW_NIBBLE PINB
// conversion constants
#define PI 3.1415926
#define TICKS_TO_RADIANS 0.0005236 // (2 * PI) / 12000
#define RADIANS_TO_TICKS 1910 // 12000 / (2 * PI)
#define ADC_TO_HALF_REV 5.859 // 6000 / 1024

#include <avr/io.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h> 
#include <avr/interrupt.h>
#include "usart.h"
#include "PWM.h"
#include "button.h"

//------------------PROTOTYPES

void velocityStep(double d);
void controlSequence();
uint16_t getPosition();
unsigned int TIM16_ReadTCNT1();
void setBitTo(uint8_t bit, uint8_t value, volatile uint8_t *reg);
void softSerial(uint8_t message);

//--------------CONTROLLER GAINS--------------//
	
double Kp, Ki, Kd;

//------------------VARIABLES

bool debug = false;
volatile uint8_t analogLow = 0, analogHigh = 0;
uint16_t desiredPosition = 3000;
uint16_t lastPosition = 0, position = 0, zero = 0;
int16_t velocity = 0;
int64_t I = 0;
int32_t P = 0, D = 0;
int32_t E = 0, lastE = 0;
double speed;
PWM externalClock(0);
PWM controlCycle(1);
Button panelButton(5, &PINC);
char str[20]; // debug output buffer
double path[] = {1};

//------------------CLASSES

class Motor {
	public:
		// accepts a decimal between -1 (full reverse) and 1 (full forward)
		static void setSpeed(double d) {
			if(d > 1)
				d = 1;
			else if(d < -1)
				d = -1;
			//USART_Sendbyte((uint8_t)(192 + (d * 63.5)));
			softSerial((uint8_t)(192 + (d * 63.5)));
		}
};

//------------------INTERRUPT SERVICE ROUTINES

ISR(TIMER1_OVF_vect) {USART_Send_string("TIMER1_OVF\n");}
ISR(TIMER1_COMPA_vect) {
	
	TCNT1 = 0; // restart control cycle
	controlSequence();
	// velocityStep(1);
}
ISR(INT0_vect) {}
ISR(TIMER0_COMPA_vect) {}
ISR(TIMER0_OVF_vect) {}
ISR(ADC_vect) {
	
	analogLow = ADCL;
	analogHigh = ADCH;
}

//------------------MAIN

int main(void) {
	
	// pwm0.setDuty(0.2); // pwm set to clock output at the moment
	
	DDRB = 0b11110000;	//B5 output: board LED
	DDRD = 0b11111111;
	PORTC = 0;
	DDRC = 0b00000000;
	PORTD = 0xff;
	
	//-Ulfuse:w:0x22:m // argument for AVRdude to get clock out
	
	//USART_Init(convertBaud(19200)); // motor serial speed
	USART_Init(convertBaud(57600)); // computer com speed
	
	ADMUX  = 0b01100110; // port A6 ADC selected
	ADCSRA = 0b10001011; // on, 2x clock
	ADCSRB = 0b00000000; // free running
	
	SREG = SREG | 0x80;
	sei(); // enable global interrupts
	externalClock.start();
	controlCycle.start();
	
	while(1) {

		desiredPosition = (uint16_t)(ADC_TO_HALF_REV * ((analogHigh << 2) | (analogLow >> 6)) + 6000);
		// desiredPosition = 6000;
		Kp = 0.37;
		Ki = 0;
		Kd = 0.04;
		
		if(!panelButton.isUp()) {
			Kp = 2;
			Ki = 0;
			Kd = 0;
			
			for(int i = 0; i < 1; i++) {
				cli();
				debug = true;
				desiredPosition = (uint16_t)(RADIANS_TO_TICKS * path[i]) + getPosition();
				sei();
				_delay_ms(1000);
			}
		}
	}
}

//------------------FUNCTIONS

uint16_t getPosition() {
	
	uint16_t count = 0;

	externalClock.stop();
	
	setBitTo(HCTL_BYTE_SELECT_BIT, 0, &CONTROL_PORT); // select MSB
	setBitTo(HCTL_ENABLE_BIT, 0, &CONTROL_PORT); // enable
	setBitTo(HCTL_CLK_BIT, 0, &CONTROL_PORT); // clock low
	
	count = (count << 8) | DATA_HIGH_NIBBLE | DATA_LOW_NIBBLE; // get byte
	
	setBitTo(HCTL_BYTE_SELECT_BIT, 1, &CONTROL_PORT); // select LSB

	count = count << 8 | DATA_HIGH_NIBBLE | DATA_LOW_NIBBLE; // get byte
	
	setBitTo(HCTL_ENABLE_BIT, 1, &CONTROL_PORT); // disable
	
	externalClock.start();
	return count;
}

unsigned int TIM16_ReadTCNT1( void ) {
	unsigned char sreg;
	unsigned int i;
	sreg = SREG; // global interrupt flags saved
	cli(); // disable interrupts
	i = TCNT1;
	SREG = sreg; // global interrupt flags restored
	sei();
	return i;
}

void setBitTo(uint8_t bit, uint8_t value, volatile uint8_t *reg) {
	*reg = (value << bit) | (*reg & (0xff - (1 << bit)));
}

void softSerial(uint8_t message) {
	
	uint8_t serialByte = message ^ 0xff;
	setBitTo(MOTOR_BIT_1, 0, &CONTROL_PORT);
	_delay_us(51);
	
	for(uint8_t i = 1; i < 8; i++) {
		setBitTo(MOTOR_BIT_1, serialByte & 1, &CONTROL_PORT);
		serialByte = serialByte >> 1;
		_delay_us(51);
	}
	
	setBitTo(MOTOR_BIT_1, 1, &CONTROL_PORT);
	_delay_us(52);
}

void velocityStep(double d) {
	
	if(!panelButton.isUp()) {		
		Motor::setSpeed(d);
		sprintf(str, "%lu,", (uint32_t)getPosition());
		USART_Send_string(str);
	} else {
		Motor::setSpeed(0);
	}
}

void controlSequence() {
	
	lastPosition = position;
	position = getPosition();
	velocity = position - lastPosition;
		
	lastE = E;
	E = (int32_t)desiredPosition - position;
		
	P = E;
	I = (int64_t)((I + E) * 0.986233); // decays to 0.5 * I after 0.1 s if no E
	D = E - lastE;
		
	speed = (double)(Kp * P + Ki * I + Kd * D) * TICKS_TO_RADIANS;
		
	Motor::setSpeed(speed);
		
	if(debug) {
		sprintf(str, "%u,", position);
		USART_Send_string(str);
	}
	
	setBitTo(ADSC, 1, &ADCSRA); // ADC read start
}
