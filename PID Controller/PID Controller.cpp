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

// measuring linear conversion: 1191 = 100.6mm, 1239 = 149.2mm

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
#define DATA_HIGH_NIBBLE		((PINC & 0b00001111) << 4)
#define DATA_LOW_NIBBLE			(PINB & 0b00001111)
#define ADC10BIT				((analogHigh << 2) | (analogLow >> 6))

// conversion constants
#define ADC_TO_DOUBLE			(2 * ADC10BIT * 0.000976562) - 1 // -1.0 to 1.0
#define PI						3.1415926
#define SLIDE_TO_CM				0.01
#define WHEEL_HALF_REVOLUTION	2048 // 24000 for commercial
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

//------------------ENUMS

enum Color { GREEN, RED, BLUE, OFF };

//------------------PROTOTYPES

void velocityStep(double d);
void controlSequence();
uint16_t getPosition(uint8_t encoder);
void softSerial(uint8_t message);
void startADC();
void selectADC(uint8_t n);
void linearHome();
void angularHome();
void setLinearSpeed(float f);
void setSolenoid();
void print_debug_info();
void updateGlobals();
void readMaster();
void setColor(Color c);
void rectangularToPolar(float x, float y, volatile uint16_t* rOut, volatile uint16_t* thetaOut);

//------------------GLOBAL FLAGS

volatile bool debug = false;
volatile bool controllable = true;

//------------------VARIABLES

Output
	alt1(			ALT1,			&ALT_PORT,		&DDRB), 
	alt2(			ALT2,			&ALT_PORT,		&DDRB),
	encoderSelect(	HCTL_SELECT,	&CONTROL_PORT,	&DDRD),
	encoderEnable(	HCTL_ENABLE,	&CONTROL_PORT,	&DDRD),
	encoderClock(	HCTL_CLK,		&CONTROL_PORT,  &DDRD),
	encoderByte(	HCTL_BYTE,		&CONTROL_PORT,	&DDRD),
	motorSerial(	MOTOR1,			&CONTROL_PORT,	&DDRD)
	;
Input 
	controlSwitch(	5,				&PINC,			&DDRC);
PID angular, linear;

volatile bool solenoid;
volatile uint16_t count;
volatile uint8_t analogLow, analogHigh;
volatile uint16_t left, right;
volatile float xIn, yIn;
volatile uint16_t desiredTheta, desiredR;
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
	
	motor2.setSpeed(0);
	softSerial(0);
	
	selectADC(6);
	USART_Init(convertBaud(COM_SPEED));
	sei(); // enable global interrupts
	
	motor2.start();
	clockOut.start();
	controllable = true;
	
	angular.setGains(2, 0, 1);
	linear.setGains(1, 0, 1);
	setColor(RED);
	linearHome();
	
	desiredR = 65000;
	desiredTheta = 2048;
	setLinearSpeed(-0.5f);
	_delay_ms(500);
	setLinearSpeed(0);
	controlCycle.start();
	
	while(1) {
		controllable = false;
		debug = false;
		
		setColor(GREEN);
		_delay_ms(1000);
		solenoid = true;
		
		setColor(RED);
		debug = true;
		motor2.setSpeed(1);
		_delay_ms(1000);
		motor2.setSpeed(-1);
		_delay_ms(1000);
		motor2.setSpeed(0);
		debug = false;
		
		setColor(BLUE);
		_delay_ms(1000);
		solenoid = false;
		
		// readMaster();
	}
}

//------------------FUNCTIONS

void setLinearSpeed(float f) {
	// accepts between -1 (full reverse) and 1 (full forward)
	if(f > 1)
		f = 1;
	else if(f < -1)
		f = -1;
	softSerial((uint8_t)(192 - f * 63));
}

void setSolenoid() {
	if(solenoid) 
		softSerial(127);
	else
		softSerial(64);
}

void linearHome() {
	
	bool homed = false;
	while(!homed) {
		setLinearSpeed(0.5f);
		motor2.setSpeed(0);
		updateGlobals();
		print_debug_info();
		if(r == 0 && lastR == 0) {
			homed = true;
		}
	}
}

void angularHome() {
	
	bool homed = false;
	uint32_t currentPosition = 32000;
	while(!homed) {
		motor2.setSpeed(-0.5f);
		currentPosition = getPosition(ENCODERY);
		if(currentPosition > 65000 || currentPosition == 0) {
			homed = true;
		}
	}
}

void readMaster() {
	
	selectADC(ADC_LEFT);
	startADC();
	_delay_us(5);
	left = ADC10BIT;
		
	selectADC(ADC_RIGHT);
	startADC();
	_delay_us(5);
	right = ADC10BIT;
	
	translate(left, right, &xIn, &yIn);
}

void selectADC(uint8_t n) {
	
	ADMUX  = 0b01100000 | n; // port n ADC selected
	ADCSRA = 0b10001011; // on, 2x clock
	ADCSRB = 0b00000000; // free running
}

uint16_t getPosition(uint8_t encoder) {
	
	uint16_t count = 0;
	
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

void updateGlobals() {
	
	lastR = r;
	r = getPosition(ENCODERX);
	linearV = r - lastR;
	
	lastTheta = theta;
	theta = getPosition(ENCODERY) % 4096;
	if(lastTheta > 2048 && theta < 2048)
		angularV = 4096 + theta - lastTheta;
	else
		angularV = theta - lastTheta;
}

void softSerial(uint8_t message) {
	
	uint8_t serialByte = message;
	setBitTo(MOTOR1, 0, &CONTROL_PORT);
	_delay_us(SOFT_SERIAL_DELAY);
	
	for(uint8_t i = 0; i < 8; i++) {
		setBitTo(MOTOR1, serialByte & 1, &CONTROL_PORT);
		serialByte = serialByte >> 1;
		_delay_us(SOFT_SERIAL_DELAY - 1);
	}
	
	setBitTo(MOTOR1, 1, &CONTROL_PORT);
	_delay_us(SOFT_SERIAL_DELAY);
	_delay_us(SOFT_SERIAL_DELAY);
}

void velocityStep(float f) {
	
	motor2.setSpeed(1);
	sprintf(str, "%lu,", (uint32_t)getPosition(ENCODERY));
	USART_Send_string(str);
	_delay_ms(1000);
	motor2.setSpeed(-1);
	
	/*
	if(!controlSwitch.isHigh()) {
		setLinearSpeed(f);
		sprintf(str, "%lu,", (uint32_t)getPosition(ENCODERY));
		USART_Send_string(str);
	} else {
		setLinearSpeed(0);
	}
	*/
}

void controlSequence() {
		
	updateGlobals();
	
	/*
	desiredR = 1150 - (uint16_t)(sqrt(xIn * xIn + yIn * yIn) / (4 * SLIDE_TO_CM));
	desiredTheta = (uint16_t)((4096 * (atan2(xIn, -yIn) + PI)) / (2 * PI));
	*/
	
	//	rectangularToPolar(xIn, yIn, &desiredR, &desiredTheta);
		
	if(controllable) {
		speed = linear.compute(desiredR, r) * SLIDE_TO_CM;
		setLinearSpeed(speed);
		
		speed = angular.compute(desiredTheta, theta) * WHEEL_TO_RADIANS;
		motor2.setSpeed(speed);
	}
	
	setSolenoid();
	
	if(debug)
		print_debug_info();
}

void startADC() { setBitTo(ADSC, 1, &ADCSRA); }

void print_debug_info() {
	
	sprintf(str,
		"%u %u\n",
		(uint16_t)(r),
		(uint16_t)(theta)
		);
	USART_Send_string(str);
}

void rectangularToPolar(float x, float y, volatile uint16_t* rOut, volatile uint16_t* thetaOut) {
	*rOut = (uint16_t)(512 - (sqrt(x * x + y * y) * SLIDE_TO_CM));
	*thetaOut = (uint16_t)(atan2(x, y) / WHEEL_TO_RADIANS);
}

void setColor(Color c) {
	switch(c) {
		case RED :
			alt1.set();
			alt2.clear();
			break;
		case GREEN :
			alt1.set();
			alt2.set();
			break;
		case BLUE :
			alt1.clear();
			alt2.set();
			break;
		case OFF :
			alt1.clear();
			alt2.clear();
			break;
	}
}