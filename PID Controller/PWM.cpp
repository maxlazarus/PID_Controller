#include "PWM.h"
#include <avr/io.h>

PWM::PWM (unsigned short u) {
	PWM_number = u;
	static bool initialized = false;
	if(!initialized)
		PWM::init();
	init(PWM_number);
}

void PWM::init() {
	PRR = 0; // disable all power reduction
	/*
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
	if(u == 0) { // init counter0 on pin D6
		OCR0A = 0x01;
		OCR0B = 0x03;
		TCCR0A = (0 << COM0A1) | (1 << COM0A0) | (1 << WGM01) | (1 << WGM00);
		TCCR0B = (0 << COM0B1) | (0 << COM0B0) | (1 << WGM02) | (1 << CS00);
		//TIMSK0 = (1 << OCIE1A) | (1 << OCIE1B);
		//TIMSK0 |= (1 << TOIE0);
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
	/*
	if(PWM_number == 0)
		OCR0A = (unsigned int)(0xff * d);
		*/
}
