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
#define CHAR_BUFFER_LENGTH		20
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
#define WHEEL_HALF_REVOLUTION	4096 // 24000 for commercial
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
enum ControlState { 
	SW_DOWN_BUTTON_UP,
	SW_DOWN_BUTTON_DOWN,
	SW_UP_BUTTON_UP,
	SW_UP_BUTTON_DOWN
};

//------------------PROTOTYPES

void angularVelocityStep();
void linearVelocityStep();
void linearPositionStep();
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
ControlState readSwitches();
void setColor(Color c);
void rectangularToPolar(float x, float y, volatile int16_t* rOut, volatile uint16_t* thetaOut);

//------------------GLOBAL FLAGS

volatile bool debug = false;
volatile bool controllable = true;
volatile bool pattern = false;

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
	angularLimit(	4,				&PINC,			&DDRC),
	controlSwitch(	5,				&PINC,			&DDRC)
	;
PID angular, linear;

volatile uint16_t angleOffset = 0;
volatile uint16_t centerR = 64895;
volatile bool solenoid;
volatile uint16_t count;
volatile uint8_t analogLow, analogHigh;
volatile uint16_t left, right;
volatile float xIn, yIn;
volatile uint16_t desiredTheta;
volatile int16_t desiredR;
volatile uint16_t r, lastR, theta, lastTheta;
volatile int16_t angularV, linearV;
volatile float speed;
PWM clockOut(0), controlCycle(1), motor2(2); // timer0, 1, 2
char strOut[CHAR_BUFFER_LENGTH]; // debug output buffer
char strIn[CHAR_BUFFER_LENGTH]; // debug input buffer
float rBuffer[255];

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
	solenoid = true;
	setSolenoid();
	
	selectADC(6);
	USART_Init(convertBaud(COM_SPEED));
	sei(); // enable global interrupts
	
	for(int i = 0; i < 256; i++) {
		// rBuffer[i] = 50 * sin((8 * PI * i) / 256) + 450;
		if (i < 128)
		rBuffer[i] = 4 * i;
		else
		rBuffer[i] = 4 * (256 - i);
	}
	
	motor2.start();
	clockOut.start();
	
	debug = false;
	pattern = false;
	
	angular.setGains(5, 0, 1);
	linear.setGains(5, 0, 1);
	
	setColor(RED);
	linearHome();
	setLinearSpeed(-0.5f);
	_delay_ms(100);
	setLinearSpeed(0);
	angularHome();
	
	desiredR = 0;
	desiredTheta = 4096;
	controllable = true;
	controlCycle.start();
	
	setColor(GREEN);
	
	char cIn;
	uint8_t bufferIndex;
	bool pressed = false;

	while(1) {
		switch(readSwitches()) {
			case SW_DOWN_BUTTON_DOWN:
				if(!pressed) {
					pressed = true;
				}
				pressed = true;
				break;
			case SW_DOWN_BUTTON_UP:
				pressed = false;
				break;
			case SW_UP_BUTTON_DOWN:
				if(!pressed) {
					setColor(RED);
					readMaster();
					rectangularToPolar(xIn, yIn, &desiredR, &desiredTheta);
					sprintf(strOut, "%d, %u\n", desiredR, desiredTheta);
					// sprintf(strOut, "%d, %d\n", int16_t(xIn * 1000), int16_t(yIn * 1000));
					// USART_Send_string(strOut);
					debug = true;
					pressed = true;
				}
				break;
			case SW_UP_BUTTON_UP:
				pressed = false;
				setColor(GREEN);
				break;
		}
	}
	
	while(1) {
		switch(cIn = USART_Receive()) {
			case '?':
				USART_Send_string("[?scdpar+-0m]\n");
				break;
			case 's':
				solenoid = !solenoid;
				if(solenoid)
					USART_Send_string("Solenoid on\n");
				else
					USART_Send_string("Solenoid off\n");
				break;
			case 'c': 
				controllable = !controllable;
				if(controllable)
					USART_Send_string("Control on\n");
				else
					USART_Send_string("Control off\n");
				break;
			case 'd':
				debug = !debug;
				if(debug)
					USART_Send_string("Debug on\n");
				else
					USART_Send_string("Debug off\n");
				break;
			case 'p':
				pattern = !pattern;
				if(pattern) {
					USART_Send_string("Pattern on\n");
					solenoid = false;
					setColor(BLUE);
				} else {
					USART_Send_string("Pattern off\n");
					solenoid = true;
				}
				break;
			case 'a':
				// USART_Send_string("Angular v step\n");
				angularVelocityStep();
				break;
			case 'r':
				// USART_Send_string("Linear v step\n");
				// linearVelocityStep();
				linearVelocityStep();
				break;
			case '+':
				desiredR += 10;
				USART_Send_string("r++\n");
				break;
			case '-':
				desiredR -= 10;
				USART_Send_string("r--\n");
				break;
			case '0':
				desiredR = 0;
				desiredTheta = 2048;
				controllable = true;
				break;
			case 'm':
				centerR = r;
				USART_Send_string("Setting center\n");
				break;
			default :
				break;
		}
		_delay_ms(1);
	}
}

//------------------FUNCTIONS

#define MAX_SPEED 0.5f

void angularVelocityStep() {
	setColor(RED);
	controllable = false;
	_delay_ms(500);
	debug = true;
	solenoid = true;
	motor2.setSpeed(MAX_SPEED);
	_delay_ms(1000);
	motor2.setSpeed(0);
	setColor(GREEN);
	_delay_ms(1000);
	debug = false;
}

void linearVelocityStep() {
	setColor(RED);
	controllable = false;
	solenoid = true;
	setLinearSpeed(-1);
	_delay_ms(150);
	setLinearSpeed(0);
	_delay_ms(250);
	debug = true;
	setLinearSpeed(1);
	_delay_ms(100);
	setLinearSpeed(0);
	setColor(GREEN);
	_delay_ms(1000);
	debug = false;
}

void linearPositionStep() {
	setColor(RED);
	controllable = true;
	solenoid = true;
	desiredR = -500;
	_delay_ms(500);
	debug = true;
	desiredR = 500;
	_delay_ms(1000);
	setColor(GREEN);
	debug = false;
}

void angularPositionStep() {
	setColor(RED);
	controllable = true;
	solenoid = true;
	desiredTheta = theta + 1 / WHEEL_TO_RADIANS;
	debug = true;
	_delay_ms(2000);
	setColor(GREEN);
	debug = false;
}

void setLinearSpeed(float f) {
	// accepts between -1 (full reverse) and 1 (full forward)
	if(f > 1)
		f = 1;
	else if(f < -1)
		f = -1;
	softSerial((uint8_t)(192 - f * 63.5));
}

void setSolenoid() {
	if(solenoid) 
		softSerial(127);
	else
		softSerial(64);
}

void linearHome() {
	
	solenoid = true;
	bool homed = false;
	while(!homed) {
		setLinearSpeed(0.5f);
		motor2.setSpeed(0);
		updateGlobals();
		if(r == 0 && lastR == 0) {
			homed = true;
		}
	}
}

void angularHome() {
	
	solenoid = true;
	bool homed = false;
	while(!homed) {
		motor2.setSpeed(0.5f);
		if(angularLimit.isLow()) {
			angleOffset = getPosition(ENCODERY) % 8192;
			homed = true;
		}
	}
}

void readMaster() {
	
	selectADC(ADC_LEFT);
	startADC();
	_delay_us(25);
	left = ADC10BIT;
		
	selectADC(ADC_RIGHT);
	startADC();
	_delay_us(25);
	right = ADC10BIT;
	
	translate(left, right, &xIn, &yIn);
}

ControlState readSwitches() {
	
	selectADC(5);
	startADC();
	_delay_us(5);
	uint16_t control = ADC10BIT;
	if(control > 975)
		return SW_DOWN_BUTTON_UP;
	else if(control > 750)
		return SW_DOWN_BUTTON_DOWN;
	else if(control > 350)
		return SW_UP_BUTTON_UP;
	else 
		return SW_UP_BUTTON_DOWN;
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
	theta = getPosition(ENCODERY) % 8192;
	if(lastTheta > 4096 && theta < 4096)
		angularV = 8192 + theta - lastTheta;
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

void controlSequence() {
		
	updateGlobals();
	
	/*
	desiredR = (uint16_t)(sqrt(xIn * xIn + yIn * yIn) / (4 * SLIDE_TO_CM));
	desiredTheta = (uint16_t)((4096 * (atan2(xIn, -yIn) + PI)) / (2 * PI));
	*/
	#define R_LENGTH 500
	
	if(pattern) {
		float tRad = PI * theta / 4096;

		if (theta < 2048)
			desiredR = R_LENGTH / (sin(tRad) + cos(tRad));
		else if (theta < 4096)
			desiredR = R_LENGTH / (sin(tRad) - cos(tRad));
		else if (theta < 6144)
			desiredR = R_LENGTH / (sin(-tRad) - cos(tRad));
		else
			desiredR = R_LENGTH / (sin(-tRad) + cos(tRad));
		// desiredR = rBuffer[count++ % 256];
	}
	//	rectangularToPolar(xIn, yIn, &desiredR, &desiredTheta);
		
	if(desiredR > 600)
		desiredR = 600;
	else if(desiredR < -600)
		desiredR = -600;
		
	if(controllable) {
		speed = linear.compute(-desiredR, r - centerR) * SLIDE_TO_CM;
		setLinearSpeed(speed);		
		speed = angular.compute(desiredTheta, ((theta + 8000 -angleOffset) % 8192) * WHEEL_TO_RADIANS);
		motor2.setSpeed(speed);
	}
	
	setSolenoid();
	
	if(debug)
		print_debug_info();
}

void startADC() { setBitTo(ADSC, 1, &ADCSRA); }

void print_debug_info() {
	/*
	sprintf(strOut,
		"%d, %u\n",
		desiredR,
		r - centerR
		);
		*/
	sprintf(strOut,
		"%u, %u\n",
		desiredTheta,
		theta
		);
	USART_Send_string(strOut);
}

void rectangularToPolar(float x, float y, volatile int16_t* rOut, volatile uint16_t* thetaOut) {
	*rOut = (int16_t)(sqrt(x * x + y * y) * 50);
	*thetaOut = (uint16_t)((atan2(x, -y) + PI) * 1304);
}

void setColor(Color c) {
	switch(c) {
		case RED :
			alt1.set();
			alt2.clear();
			break;
		case GREEN :
			alt1.clear();
			alt2.set();
			break;
		case BLUE :
			alt1.set();
			alt2.set();
			break;
		case OFF :
			alt1.clear();
			alt2.clear();
			break;
	}
}