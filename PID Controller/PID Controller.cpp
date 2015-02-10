/*
 * PID_Controller.cpp
 *
 * Created: 1/23/2015 9:28:17 AM
 *  Author: Max Prokopenko
 *  max@theprogrammingclub.com
 *
 *  Nano pinout:
 *	 D2:	Hall Effect input
 *   D6:	PWM0 out
 */ 

#define F_CPU 16000000L
#include <avr/io.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h> 
#include <avr/interrupt.h>
#include "led.h"
#include "usart.h"
#include "PWM.h"

LED led1(0x05,5);
PWM pwm0(0);
bool flip = false;

void print(const char* name, unsigned int reg){
	USART_Send_string(name);
	USART_Send_int(reg);
	USART_Send_string("\n");
}
void print(const char* name){
	USART_Send_string(name);
	USART_Send_string("\n");
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

class Motor {
	public:
		// accepts a decimal between -1 (full reverse) and 1 (full forward)
		static void setSpeed(double d) {
			USART_Sendbyte((unsigned short)(192 + (d * 63.5)));
		}
};

ISR(INT0_vect) {
	//print("INT0 triggered");
}
ISR(TIMER0_COMPA_vect) {}
ISR(TIMER0_OVF_vect) {}

int main(void) {
	
	pwm0.setDuty(0.5);
	pwm0.setPrescaler(1);
	pwm0.start();
	
	DDRB = 0b11110000;	//B5 output: board LED
	DDRD = 0b11111111;
	DDRC = 0b11110000;
	PORTD = 0xff;
	
	//-Ulfuse:w:0x22:m
	
	//USART_Init(convertBaud(9600)); // motor serial speed
	USART_Init(convertBaud(57600));
	
	int interval = 1;
	uint8_t dir = 0b00001000;
	uint8_t mask = 0b00000000;
	ADMUX = 0b1100000;
	ADCSRA = 0b10000011;
	ADCSRB = 0b00000000;

	SREG = SREG | 0x80;
	sei(); // enable global interrupts
	long count = 0;
	
	while(1) {
		count = 0;
		USART_Send_string("HELLO\n");
		
		PORTD = 0b00001000 | (PORTD & 0b11110011);
		count = PINC | PINB<<4;
		/*
		PORTD = 0b00001100 | (PORTD & 0b11110011);
		count = count<<8 | PINC | PINB<<4;
		
		PORTD = 0b00000000 | (PORTD & 0b11110011);
		count = count<<8 | PINC | PINB<<4;

		PORTD = 0b00000100 | (PORTD & 0b11110011);
		count = count<<8 | PINC | PINB<<4;
		*/
		USART_Send_int(count);
		USART_Send_string("\n");
		_delay_ms(500);
	}
	
    while(1) {
		_delay_ms(1000);
		Motor::setSpeed(-1);
		_delay_ms(1000);
		Motor::setSpeed(1);
		_delay_ms(1000);
		Motor::setSpeed(0);
    }
}