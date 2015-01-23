/*
 * PID_Controller.cpp
 *
 * Created: 1/23/2015 9:28:17 AM
 *  Author: Maxim
 */ 


#define F_CPU 16000000L
#include <avr/io.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h> 
#include "led.h"
#include "usart.h"

LED led1(0x05,5);

unsigned int TIM16_ReadTCNT1( void )
{
	unsigned char sreg;
	unsigned int i;
	/* Save global interrupt flag */
	sreg = SREG;
	/* Disable interrupts */
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11!!!!!!!!!!!!!!
	//_CLI();
	
	/* Read TCNT1 into i */
	i = TCNT1;
	/* Restore global interrupt flag */
	SREG = sreg;
	return i;
}

int main(void)
{
	// INITIALIZATIONS
	DDRB = 0b00100000;	//B5 output: board LED
	DDRD = 0b11111100;
	USART_Init(MYUBRR); // Initializes the serial communication
	// Go to USART.H AND CHANGE YOUR FOSC AND BAUD
	TCCR0A = 0x00;
	int interval = 1;
	uint8_t dir = 0b00001000;
	uint8_t mask = 0b00000000;
	ADMUX = 0b1100000;
	ADCSRA = 0b10000011;
	ADCSRB = 0b00000000;
	PRR = 0;//&= ~(1<<PRADC);
	char output[8];
	sprintf(output, "%d", PRR);
	unsigned int test;
	
    while(1)
    {
		//led1.on();
		PORTD = 0x04 | mask;
		char str0[8];
		char str1[8];
		
		test = ADCH;
		ADCSRA = 0b11000011;
		/*
		int counter = 0;
		while ((ADCSRA & (1<<ADSC)) > 0) {
			sprintf(output, "%d", counter);
			USART_Send_string("\nwaiting...");
			USART_Send_string(output);
			USART_Send_string("\n");
			counter++;
		}
		*/
		sprintf(str0, "%d", test);
		if(test <= 127)
			mask = dir;
		else {
			mask = 0;
			test = 255 - test;
		} 
		
		interval = (test + 32) / 32;	

		/*
        USART_Send_string("A0: ");
		USART_Send_string(str0);
		USART_Send_string("\n");
		*/

		for(int i = 0; i < interval; i++)
			_delay_us(250);	
		//led1.off();
		PORTD = 0 | mask;

		for(int i = 0; i < interval; i++)
			_delay_us(250);
    }
}