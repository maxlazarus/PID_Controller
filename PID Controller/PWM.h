/*
 * PWM.h
 *
 * February 2015
 *  Author: Max Prokopenko
 *  max@theprogrammingclub.com
 */

using namespace std;

class PWM {
	private:
		unsigned short PWM_number;
		unsigned short savedState;
	public:
		PWM(unsigned short i);
		void init();
		void setDuty(double d);
		unsigned int read();
		void start();
		void stop();
};

#include "F_CPU.h"
#include <avr/io.h>

#define TIMER_1_FREQUENCY 500 // Hz

PWM::PWM (unsigned short u) {
	PWM_number = u;
	init();
}

void PWM::init() {
	PRR = 0; // disable all power reduction
	
	if(PWM_number == 0) { // init counter0 on pin D6
		OCR0A = 0x01;
		OCR0B = 0x03;
		TCCR0A = (0 << COM0A1) | (1 << COM0A0) | (1 << WGM01) | (1 << WGM00);
		// TIMSK0 = (1 << OCIE1A) | (1 << OCIE1B);
		// TIMSK0 |= (1 << TOIE0);
		} else if (PWM_number == 1) {
		OCR1A = F_CPU / TIMER_1_FREQUENCY;
		OCR1B = 0;
		TCCR1A = (0 << COM0A1) | (0 << COM1A0) | (0 << WGM11) | (0 << WGM10);
		TIMSK1 |= (1 << OCIE1A) | (1 << TOIE0); // timer1 compare, overflow
	}
}

unsigned int PWM::read() {return 0;}

void PWM::start() {
	if(this->PWM_number == 0) {
		TCCR0B = (0 << COM0B1) | (0 << COM0B0) | (1 << WGM02) | (1 << CS00);
		} else {
		TCCR1B = (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (1 << CS10);
	}
}

void PWM::stop() {
	if(this->PWM_number == 0) {
		TCCR0B = 0;
		} else {
		TCCR1B = 0;
	}
}

void PWM::setDuty(double d) {}