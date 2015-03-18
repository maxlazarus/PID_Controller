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

#include "F_CPU.h" // clock speed

#define CONTROL_PORT PORTD
#define HCTL_BYTE_SELECT_BIT 2
#define HCTL_ENABLE_BIT 7
#define HCTL_SELECT 4
#define MOTOR1 2 // soft serial
#define MOTOR2 3 // PWM
#define HCTL_CLK_BIT 6
#define DATA_HIGH_NIBBLE (PINC << 4)
#define DATA_LOW_NIBBLE PINB
#define ADC10BIT ((analogHigh << 2) | (analogLow >> 6))
#define ADC_TO_DOUBLE 0.000976562
// conversion constants
#define PI 3.1415926
#define TICKS_TO_RADIANS 0.0005236 // (2 * PI) / 12000
#define RADIANS_TO_TICKS 1910 // 12000 / (2 * PI)
#define ADC_TO_HALF_REV 5.859 // 6000 / 1024
#define HALF_REV 6000
#define SOFT_SERIAL_BAUDRATE 19200
#define SOFT_SERIAL_DELAY (1000000 / SOFT_SERIAL_BAUDRATE)

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
void testServoController();

//--------------CONTROLLER GAINS--------------//
	
double Kp, Ki, Kd;

//------------------GLOBAL FLAGS

volatile bool debug = false;
volatile bool controllable = true;

//------------------VARIABLES

volatile uint8_t analogLow = 0, analogHigh = 0;
uint16_t desiredPosition = HALF_REV;
uint16_t lastPosition = 0, position = 0, zero = 0;
int16_t velocity = 0;
int32_t I = 0;
int32_t P = 0, D = 0;
int32_t E = 0, lastE = 0;
double speed;
PWM externalClock(0);
PWM controlCycle(1);
PWM motor2(2);
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
			softSerial((uint8_t)(192 + (d * 63.5)));
		}
};

//------------------INTERRUPT SERVICE ROUTINES

ISR(TIMER1_COMPA_vect) {
	
	TCNT1 = 0; // restart control cycle
	if(controllable)
		controlSequence();
	else
		velocityStep(1);
}

ISR(ADC_vect) {
	analogLow = ADCL;
	analogHigh = ADCH;
}

//------------------MAIN

int main(void) {
	
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

	Kp = 0.4;
	Ki = 0.04;
	Kd = 0.2;
	
	sei(); // enable global interrupts
	
	motor2.start();
	externalClock.start();
	
	while(1) {
		// both motors controlled by potentiometer
		motor2.setDuty(ADC10BIT * ADC_TO_DOUBLE);
		setBitTo(ADSC, 1, &ADCSRA); // ADC read start
		Motor::setSpeed(ADC10BIT * ADC_TO_DOUBLE);
		_delay_ms(50);
	}
	
	controlCycle.start();
	
	while(1) {
		
		if(!panelButton.isUp()) {
			
			controllable = false;
			
			for(int i = 0; i < 1; i++) {
				cli();
				debug = true;
				desiredPosition = (uint16_t)(RADIANS_TO_TICKS * path[i]) + getPosition();
				sei();
				_delay_ms(1000);
			}
		} else {
			controllable = true;
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
	setBitTo(MOTOR1, 0, &CONTROL_PORT);
	_delay_us(SOFT_SERIAL_DELAY);
	
	for(uint8_t i = 1; i < 8; i++) {
		setBitTo(MOTOR1, serialByte & 1, &CONTROL_PORT);
		serialByte = serialByte >> 1;
		_delay_us(SOFT_SERIAL_DELAY - 1);
	}
	
	setBitTo(MOTOR1, 1, &CONTROL_PORT);
	_delay_us(SOFT_SERIAL_DELAY);
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
	
	if(controllable)
		desiredPosition = (uint16_t)(ADC_TO_HALF_REV * ADC10BIT + HALF_REV);
	lastPosition = position;
	position = getPosition();
	velocity = position - lastPosition;	
		
	lastE = E;
	E = (int32_t)desiredPosition - position;
		
	P = E;
	I = I + E;
	// limiting I to prevent instability
	if(I > RADIANS_TO_TICKS)
		I = RADIANS_TO_TICKS;
	else if(I < -RADIANS_TO_TICKS)
		I = -RADIANS_TO_TICKS;
	D = E - lastE;
		
	speed = (double)(Kp * P + Ki * I + Kd * D) * TICKS_TO_RADIANS;
		
	Motor::setSpeed(speed);
		
	if(debug) {
		sprintf(str, "%u,", position);
		USART_Send_string(str);
	}
	
	setBitTo(ADSC, 1, &ADCSRA); // ADC read start
}

void testServoController() {
	uint16_t pulseWidth = 0;
		
	while(1) {
		pulseWidth = 400 + 2 * ADC10BIT;
		setBitTo(ADSC, 1, &ADCSRA); // ADC read start
			
		setBitTo(5, 1, &PORTD);
		for(uint16_t i = 0; i < pulseWidth; i++){
			_delay_us(1);
		}
		setBitTo(5, 0, &PORTD);
		_delay_ms(10);
			
		sprintf(str, "%u,", pulseWidth);
		USART_Send_string(str);
	}
}