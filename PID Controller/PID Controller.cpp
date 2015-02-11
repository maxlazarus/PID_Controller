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

void print(const char* name, unsigned int reg){
	USART_Send_string(name);
	USART_Send_string(" ");
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
uint32_t getPosition() {
	// PORTD2,3 selects HCTL-2022, PORTD4 enables
	// PORTC bits 0-3 read lower nibble of HCTL data
	// PORTB bits 0-3 read upper nibble of HCTL data
	uint32_t count = 0;

	// toggle enable on D4
	PORTD = 0b00010000 | (PORTD & 0b11101111);
	PORTD = 0b00000000 | (PORTD & 0b11101111);

	// read MSB
	PORTD = 0b00001000 | (PORTD & 0b11110011);
	count = PINC | (PINB << 4);

	PORTD = 0b00001100 | (PORTD & 0b11110011);
	count = count<<8 | PINC | (PINB << 4);

	PORTD = 0b00000000 | (PORTD & 0b11110011);
	count = count<<8 | PINC | (PINB << 4);

	// read LSB
	PORTD = 0b00000100 | (PORTD & 0b11110011);
	return count<<8 | PINC | (PINB << 4);
}
class Motor {
	public:
		// accepts a decimal between -1 (full reverse) and 1 (full forward)
		static void setSpeed(double d) {
			USART_Sendbyte((unsigned short)(192 + (d * 63.5)));
		}
};

ISR(INT0_vect) {
	// print("INT0 triggered");
}
ISR(TIMER0_COMPA_vect) {}
ISR(TIMER0_OVF_vect) {}

int main(void) {
	
	// pwm0.setDuty(0.2); // pwm set to clock output at the moment
	
	DDRB = 0b11110000;	//B5 output: board LED
	DDRD = 0b11111111;
	DDRC = 0b11110000;
	PORTD = 0xff;
	
	//-Ulfuse:w:0x22:m // argument for AVRdude to get clock out
	
	USART_Init(convertBaud(9600)); // motor serial speed
	// USART_Init(convertBaud(57600)); // computer com speed
	
	ADMUX = 0b1100000;
	ADCSRA = 0b10000011;
	ADCSRB = 0b00000000;

	SREG = SREG | 0x80;
	sei(); // enable global interrupts
	uint32_t count = 0;
	uint32_t desiredPosition = 15000;
	
	while(1) {
		count++;
		if(count > 100) {
			count = 0;
			if(desiredPosition == 15000)
				desiredPosition = 45000;
			else
				desiredPosition = 15000;
		}
		uint32_t dTheta = desiredPosition - getPosition();
		double speed = dTheta / 1000;
		Motor::setSpeed(speed);
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