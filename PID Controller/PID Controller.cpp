/*
 * PID_Controller.cpp
 *
 * Created: 1/23/2015 9:28:17 AM
 *  Author: Max Prokopenko
 *  max@theprogrammingclub.com
 */ 

// COMMAND LINE OPTIONS FOR FUSE CONTROL
// -U lfuse:r:-:i -U hfuse:r:-:i -U efuse:r:-:i

//--------------INCLUDES AND DEFINES-----------//

#include "F_CPU.h"				// clock speed
#define COM_SPEED				57600
#define CONTROL_PORT			PORTD

#define MOTOR1					2 // soft serial
#define MOTOR2					3 // PWM
#define HCTL_SELECT				4
#define HCTL_BYTE				5
#define HCTL_CLK				6
#define HCTL_ENABLE				7

#define ALT_PORT				PORTB
#define ALT1					4
#define ALT2					5

#define ADC_LEFT				6
#define ADC_RIGHT				7

#define ENCODERX				0
#define ENCODERY				1
#define DATA_HIGH_NIBBLE		(PINC << 4)
#define DATA_LOW_NIBBLE			PINB
#define ADC10BIT				((analogHigh << 2) | (analogLow >> 6))

// conversion constants
#define ADC_TO_DOUBLE			(2 * ADC10BIT * 0.000976562) - 1 // -1.0 to 1.0
#define PI						3.1415926
#define WHEEL_HALF_REVOLUTION	1024 // 24000 for commercial
#define WHEEL_TO_RADIANS		PI / WHEEL_HALF_REVOLUTION
#define WHEEL_TO_DEGREES		180.0 / WHEEL_HALF_REVOLUTION
#define SOFT_SERIAL_BAUDRATE	19200
#define SOFT_SERIAL_DELAY		(1000000 / SOFT_SERIAL_BAUDRATE) // us

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h> 
#include <avr/interrupt.h>
#include "usart.h"
#include "PWM.h"
#include "PID.h"
#include "io_utils.h"
#include "master.h"

//------------------PROTOTYPES

void velocityStep(double d);
void controlSequence();
uint16_t getPosition(uint8_t encoder);
void getHighLowPosition(uint8_t encoder, volatile uint8_t* high, volatile uint8_t* low);
void softSerial(uint8_t message);
void startADC();
void selectADC(uint8_t n);
void home();
void setMotor1Speed(float f);
void print_debug_info();
void updateGlobals();

//------------------GLOBAL FLAGS

volatile bool debug = false;
volatile bool controllable = true;

//------------------VARIABLES

Output
	/* 
	alt1(			ALT1,			&ALT_PORT,		&DDRB), 
	alt2(			ALT2,			&ALT_PORT,		&DDRB),
	*/
	encoderSelect(	HCTL_SELECT,	&CONTROL_PORT,	&DDRD),
	encoderEnable(	HCTL_ENABLE,	&CONTROL_PORT,	&DDRD),
	encoderClock(	HCTL_CLK,		&CONTROL_PORT,  &DDRD),
	encoderByte(	HCTL_BYTE,		&CONTROL_PORT,	&DDRD),
	motorSerial(	MOTOR1,			&CONTROL_PORT,	&DDRD)
	;
Input 
	panelButton(	ALT2,			&PINC,			&DDRB);
PID angular, linear;

volatile uint8_t analogLow = 0, analogHigh = 0;
uint16_t desiredTheta = PI / WHEEL_TO_RADIANS, desiredR = 0;

volatile uint16_t left, right;
volatile float xIn = 0, yIn = 0;
volatile uint16_t r, lastR, theta, lastTheta;
volatile int16_t angularV, linearV;
volatile float speed;
PWM clockOut(0), controlCycle(1), motor2(2); // timer0, 1, 2
char str[20]; // debug output buffer
double path[] = {1, 2, 3, 4, 5};

//------------------INTERRUPT SERVICE ROUTINES

ISR(TIMER1_COMPA_vect) {
	
	TCNT1 = 0; // restart control cycle
	controlSequence();
}

ISR(ADC_vect) {
	
	analogLow = ADCL;
	analogHigh = ADCH;
}

//------------------MAIN

int main(void) {
	
	DDRB = 0b11110000;
	DDRC = 0b00000000;
	DDRD = 0b11111111;
	
	selectADC(6);
	USART_Init(convertBaud(COM_SPEED));
	sei(); // enable global interrupts
	
	motor2.start();
	clockOut.start();
	controllable = true;
	
	angular.setGains(1, 0, 0);
	linear.setGains(1, 0, 0);
	// home();
	
	volatile uint8_t high, low;
	
	while(1) {
		getHighLowPosition(ENCODERX, &high, &low);
		cli();
		sprintf(str, "high: %d, low: %d\n", (uint16_t)(high), (uint16_t)(low));
		USART_Send_string(str);
		_delay_ms(100);
		sei();
	}
	
	while(1) {
		selectADC(ADC_LEFT);
		startADC();
		_delay_ms(100);
		left = ADC10BIT;
	
		selectADC(ADC_RIGHT);
		startADC();
		_delay_ms(100);
		right = ADC10BIT;
		
		sprintf(str, "L: %d, R: %d\n", left, right);
		USART_Send_string(str);
		
		translate(left, right, &xIn, &yIn);
		
		sprintf(str, "X: %d, Y: %d\n", (int16_t)(xIn), (int16_t)(yIn));
		USART_Send_string(str);
	}
	
	while(0) {
		// both motors controlled by single potentiometer on A6
		motor2.setSpeed(1);
		for(int i = 0; i < 20; i++) {
			updateGlobals();
			startADC();
			_delay_ms(100);
			sprintf(str, "%d,", angularV);
			USART_Send_string(str);
		}
		motor2.setSpeed(-1);
		for(int i = 0; i < 20; i++) {
			updateGlobals();
			startADC();
			_delay_ms(100);
			sprintf(str, "%d,", angularV);
			USART_Send_string(str);
		}
	}
	
	controlCycle.start();
	
	while(1) {
		debug = true;
	}
}

//------------------FUNCTIONS

void setMotor1Speed(float f) {
	// accepts between -1 (full reverse) and 1 (full forward)
	if(f > 1)
		f = 1;
	else if(f < -1)
		f = -1;
	softSerial((uint8_t)(192 + (f * 63.5)));
}

void home() {
	
	bool homed = false;
	uint32_t currentPosition = 32000;
	while(!homed) {
		motor2.setSpeed(-0.5f);
		currentPosition = getPosition(ENCODERX);
		if(currentPosition > 65000 || currentPosition == 0) {
			homed = true;
		}
	}
}

void selectADC(uint8_t n) {
	
	ADMUX  = 0b01100000 | n; // port n ADC selected
	ADCSRA = 0b10001011; // on, 2x clock
	ADCSRB = 0b00000000; // free running
}

uint16_t getPosition(uint8_t encoder) {
	
	uint16_t count;
	
	setBitTo(HCTL_SELECT, encoder, &CONTROL_PORT);

	clockOut.stop();
	
	setBitTo(HCTL_BYTE, 0, &CONTROL_PORT); // select MSB
	setBitTo(HCTL_ENABLE, 0, &CONTROL_PORT); // enable
	setBitTo(HCTL_CLK, 0, &CONTROL_PORT); // clock low
	
	count = DATA_HIGH_NIBBLE | DATA_LOW_NIBBLE; // get byte
	
	setBitTo(HCTL_BYTE, 1, &CONTROL_PORT); // select LSB

	count = count << 8 | DATA_HIGH_NIBBLE | DATA_LOW_NIBBLE; // get byte
	
	setBitTo(HCTL_ENABLE, 1, &CONTROL_PORT); // disable
	
	clockOut.start();
	
	return count;
}

void getHighLowPosition(uint8_t encoder, volatile uint8_t* high, volatile uint8_t* low) {
	
	setBitTo(HCTL_SELECT, encoder, &CONTROL_PORT);

	clockOut.stop();
	
	setBitTo(HCTL_BYTE, 0, &CONTROL_PORT); // select MSB
	setBitTo(HCTL_ENABLE, 0, &CONTROL_PORT); // enable
	setBitTo(HCTL_CLK, 0, &CONTROL_PORT); // clock low
	
	// count = DATA_HIGH_NIBBLE | DATA_LOW_NIBBLE; // get byte
	*high = (uint8_t) DATA_HIGH_NIBBLE | DATA_LOW_NIBBLE;
	
	setBitTo(HCTL_BYTE, 1, &CONTROL_PORT); // select LSB

	// count = count << 8 | DATA_HIGH_NIBBLE | DATA_LOW_NIBBLE; // get byte
	*low = (uint8_t) DATA_HIGH_NIBBLE | DATA_LOW_NIBBLE;
	
	setBitTo(HCTL_ENABLE, 1, &CONTROL_PORT); // disable
	
	clockOut.start();
}

void updateGlobals() {
	
	lastR = r;
	r = getPosition(ENCODERX);
	linearV = r - lastR;
	
	_delay_ms(1);
	
	lastTheta = theta;
	theta = getPosition(ENCODERY) % 2048;
	angularV = theta - lastTheta;
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

void velocityStep(float f) {
	
	if(!panelButton.isUp()) {
		setMotor1Speed(f);
		sprintf(str, "%lu,", (uint32_t)getPosition(ENCODERX));
		USART_Send_string(str);
	} else {
		setMotor1Speed(0);
	}
}

void controlSequence() {
	
	// alt1.set();
	
	if(controllable) {
		desiredTheta = (uint16_t)(WHEEL_HALF_REVOLUTION * (1 + ADC_TO_DOUBLE));
	}
	
	updateGlobals();
		
	speed = angular.compute(desiredTheta, theta) * WHEEL_TO_RADIANS;
	motor2.setSpeed(speed);
	
	// alt2.set();

	if(debug) {
		// print_debug_info();
	}
	
	// alt2.clear();
	
	startADC();
	
	// alt1.clear();
}

void startADC() { setBitTo(ADSC, 1, &ADCSRA); }

void print_debug_info() {
	
	sprintf(str,
		"d: %u, p: %u\n",
		(uint16_t)(desiredTheta * WHEEL_TO_DEGREES),
		(uint16_t)(theta * WHEEL_TO_DEGREES)
	);
	USART_Send_string(str);
}