#include "PWM.h"
#include <avr/io.h>

PWM::PWM (unsigned short i) {
	unsigned short PWM_number = i;
	static bool initialized = false;
	if(!initialized)
		PWM::init();
	init(PWM_number);
	unsigned int compare = 0xff;
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

void PWM::init(unsigned short i) {
	if(i == 0) { // init counter0 on pin D6
		OCR0A = 0x20;
		OCR0B = 0x7f;
		TCCR0A = 0;
		TCCR0B = 0;
		TCCR0A = (1 << COM0A1) | (0b11); // toggle pin D6 on match, TOP = OCR0A
		TCCR0B = (0 << WGM02) | (0b00000010); // toggle part above, clk/8 prescaler
		TIMSK0 = (1 << OCIE1A); // output compare match A interrupt enable
		TIMSK0 |= (1 << TOIE0); // overflow interrupt enable
	} if (i == 1) {
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

