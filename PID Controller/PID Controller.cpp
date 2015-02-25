/*
 * PID_Controller.cpp
 *
 * Created: 1/23/2015 9:28:17 AM
 *  Author: Max Prokopenko
 *  max@theprogrammingclub.com
 *
 *  NOPE
 *  Nano pinout:
 *	 D2:	Hall Effect input
 *   D6:	PWM0 out
 */ 

#define CONTROL_PORT PORTD
#define HCTL_BYTE_SELECT_BIT 2
#define HCTL_ENABLE_BIT 4
#define MOTOR_BIT_1 5 // soft serial
#define HCTL_CLK_BIT 6
#define DATA_HIGH_NIBBLE (PINC << 4)
#define DATA_LOW_NIBBLE PINB

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
#include "button.h"

LED led1(0x05,5);
PWM pwm0(0);

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
void setBitTo(uint8_t bit, uint8_t value, volatile uint8_t *reg) {
	*reg = (value << bit) | (*reg & (0xff - (1 << bit)));
}
uint16_t getPosition() {
	
	uint16_t count = 0;

	// turn off clock
	TCCR0A = 0;
	TCCR0B = 0;
	
	setBitTo(HCTL_BYTE_SELECT_BIT, 0, &CONTROL_PORT); // select MSB
	setBitTo(HCTL_ENABLE_BIT, 0, &CONTROL_PORT); // enable
	setBitTo(HCTL_CLK_BIT, 0, &CONTROL_PORT); // clock low
	
	count = (count << 8) | DATA_HIGH_NIBBLE | DATA_LOW_NIBBLE; // get byte
	
	setBitTo(HCTL_BYTE_SELECT_BIT, 1, &CONTROL_PORT); // select LSB

	count = count << 8 | DATA_HIGH_NIBBLE | DATA_LOW_NIBBLE; // get byte
	
	setBitTo(HCTL_ENABLE_BIT, 1, &CONTROL_PORT); // disable
	
	// turn on clock
	TCCR0A = (0 << COM0A1) | (1 << COM0A0) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (0 << COM0B1) | (0 << COM0B0) | (1 << WGM02) | (1 << CS00);
	return count;
}
void softSerial(uint8_t message) {
	
	uint8_t serialByte = message ^ 0xff;
	setBitTo(MOTOR_BIT_1, 0, &CONTROL_PORT);
	_delay_us(51);
	
	for(uint8_t i = 1; i < 8; i++) {
		setBitTo(MOTOR_BIT_1, serialByte & 1, &CONTROL_PORT);
		serialByte = serialByte >> 1;
		_delay_us(51);
	}
	
	setBitTo(MOTOR_BIT_1, 1, &CONTROL_PORT);
	_delay_us(52);
}
class Motor {
	public:
		// accepts a decimal between -1 (full reverse) and 1 (full forward)
		static void setSpeed(double d) {
			if(d > 1)
				d = 1;
			else if(d < -1)
				d = -1;
			//USART_Sendbyte((uint8_t)(192 + (d * 63.5)));
			softSerial((uint8_t)(192 + (d * 63.5)));
		}
};

ISR(INT0_vect) {
	// print("INT0 triggered");
}
ISR(TIMER0_COMPA_vect) {}
ISR(TIMER0_OVF_vect) {}
volatile uint8_t analogLow = 0, analogHigh = 0;
ISR(ADC_vect) {
	
	analogLow = ADCL;
	analogHigh = ADCH;
}

int main(void) {
	
	// pwm0.setDuty(0.2); // pwm set to clock output at the moment
	
	DDRB = 0b11110000;	//B5 output: board LED
	DDRD = 0b11111111;
	PORTC = 0;
	DDRC = 0b00000000;
	PORTD = 0xff;
	
	//-Ulfuse:w:0x22:m // argument for AVRdude to get clock out
	
	//USART_Init(convertBaud(19200)); // motor serial speed
	USART_Init(convertBaud(57600)); // computer com speed
	
	ADMUX  = 0b01100100; // port A4 ADC selected
	ADCSRA = 0b10001011; // on, 2x clock
	ADCSRB = 0b00000000; // free running
	DIDR0  = 0b00010000; // what for?
	
	uint16_t desiredPosition = 3000;
	uint16_t lastPosition = 0, position = 0, zero = 0;
	int16_t velocity = 0;
	int32_t P = 0, I = (int32_t)desiredPosition - getPosition(), D = 0;
	int32_t E = 0, lastE = 0;
	double speed;
	char str[20];
	
	SREG = SREG | 0x80;
	sei(); // enable global interrupts
	PRR = 0;
	Button panelButton(5, &PINC);
	
	while(0) {
		if(panelButton.isUp())
			USART_Send_string("YES\n");
		else
			USART_Send_string("NO\n");
	}
	
	while(0) {
		Motor::setSpeed(0.5);
		_delay_ms(1);
		sprintf(str, "Position: %lu\n", (uint32_t)getPosition());
		USART_Send_string(str);
		_delay_ms(1);
	}
	
	while(1) {	
		
		if(!panelButton.isUp()) {
			zero = getPosition();
		}
		else {
			lastPosition = position - zero;
			position = getPosition() - zero;
			velocity = position - lastPosition;
		
			/*
			if(lastPosition < 250 && position > 16250)
				E = (int32_t)desiredPosition - (0xFFFF - position);
			else if(lastPosition > 16250 && position < 250)
				E = (int32_t)desiredPosition - (0xFFFF + position);
			else
			*/
			
			desiredPosition = (10 * (analogHigh << 2) | (analogLow >> 6)) + 6000;
		
			//desiredPosition = 6000;
		
			lastE = E;	
			E = (int32_t)desiredPosition - position; 
		
			P = E;
			I = I + E;
			D = lastE - E;
		
			speed = (double)(0.5 * D + 0.3 * P) / 1000;
	
			Motor::setSpeed(speed);
		}
		
		sprintf(str, "p: %u,", position);
		USART_Send_string(str);
		
		sprintf(str, "v: %i\n\r", velocity);
		USART_Send_string(str);
		
		setBitTo(ADSC, 1, &ADCSRA); // ADC read start
	}
}