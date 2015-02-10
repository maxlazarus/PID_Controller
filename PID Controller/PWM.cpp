#include "PWM.h"
#include <avr/io.h>

PWM::PWM (unsigned short u) {
	PWM_number = u;
	init(PWM_number);
}

void PWM::init() {
	PRR = 0; // disable all power reduction
	/*
	
	/////////////////FOR ROBOTS
	PCINT0; // interrupt vector
	EICRA = 0x01; // change on INT0 triggers interrupt
	EIMSK = 0x01; // enable INTO0 interrupt
	
	TCCR1A = 0;     // set entire TCCR1A register to 0
	TCCR1B = 0;     // same for TCCR1B
	// set compare match register to desired timer count:
	OCR1A = 1562;
	// turn on CTC mode:
	TCCR1B |= (1 << WGM12);
	// Set CS10 and CS12 bits for 1024 prescaler:
	TCCR1B |= (1 << CS10);
	TCCR1B |= (1 << CS12);
	// enable timer compare interrupt:
	TIMSK1 |= (1 << OCIE1A);
	*/
}

void PWM::init(unsigned short u) {
	PRR = 0;
	setPrescaler(1);
	if(u == 0) { // init counter0 on pin D6
		OCR0A = 0x20;
		OCR0B = 0x7f;
		TCCR0A = 0;
		TCCR0B = 0;
		TCCR0A = (1 << COM0A1) | (0b11); // toggle pin D6 on match, TOP = OCR0A
		TIMSK0 = (1 << OCIE1A); // output compare match A interrupt enable
		TIMSK0 |= (1 << TOIE0); // overflow interrupt enable
	} if (u == 1) {
		TCCR1A = 0;
		TCCR1B = 0;  
		TCCR1B |= (1 << WGM12);
		TCCR1B |= (1 << CS10);
		TCCR1B |= (1 << CS12);
		// enable timer compare interrupt:
		TIMSK1 |= (1 << OCIE1A);
	}
}

unsigned int PWM::read() {
	//TIFR1;
	TCNT1 = 0x1ff;
	return TCNT1;
}

void PWM::setDuty(double d) {
	if(PWM_number == 0)
		OCR0A = (unsigned int)(0xff * d);
}

void PWM::setPrescaler(int n) {
	switch(n) {
		case 1: prescaler = 0b1;
		break;
		case 8: prescaler = 0b10;
		break;
		case 64: prescaler = 0b11;
		break;
		case 256: prescaler = 0b100;
		break;
		case 1024: prescaler = 0b101;
		break;
	}
}

void PWM::start() {
	if(PWM_number == 0)
		TCCR0B = (0 << WGM02) | prescaler;
}

void PWM::stop() {
	if(PWM_number == 0)
		TCCR0B = (0 << WGM02);
}
