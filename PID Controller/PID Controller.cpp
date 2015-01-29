/*
 * PID_Controller.cpp
 *
 * Created: 1/23/2015 9:28:17 AM
 *  Author: Maxim
 *
 *  Nano pinout:
 *	 D2:	Hall Effect input
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

ISR(INT0_vect) {
	print("INT0 triggered");
}

ISR(TIMER0_COMPA_vect) {	
}

ISR(TIMER0_OVF_vect) {
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

int main(void) {
	DDRB = 0b11111111;	//B5 output: board LED
	DDRD = 0b11111111;
	PORTD = 0xff;
	USART_Init(MYUBRR); // Initializes the serial communication
	int interval = 1;
	uint8_t dir = 0b00001000;
	uint8_t mask = 0b00000000;
	ADMUX = 0b1100000;
	ADCSRA = 0b10000011;
	ADCSRB = 0b00000000;

	SREG = SREG | 0x80;
	sei(); // enable global interrupts
	
    while(1) {
		OCR0A = 64;
		_delay_ms(1000);
		OCR0A = 100;
		_delay_ms(1000);
		OCR0A = 127;
		_delay_ms(1000);
		OCR0A = 192;
		_delay_ms(1000);
		OCR0A = 154;
		_delay_ms(1000);
		OCR0A = 127;
		_delay_ms(1000);
    }
}