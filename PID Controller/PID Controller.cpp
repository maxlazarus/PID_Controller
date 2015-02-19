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

const uint8_t HCTL_CLK_BIT = 6; // PORTD
LED led1(0x05,5);
PWM pwm0(0);

void print(const char* name, uint32_t value){
	USART_Send_string(name);
	USART_Send_string(" ");
	USART_Send_uint(value);
	USART_Send_string("\n");
}
void print(const char* name, int16_t value){
	USART_Send_string(name);
	USART_Send_string(" ");
	USART_Send_int(value);
	USART_Send_string("\n");
}
void print(const char* name, double value){
	USART_Send_string(name);
	USART_Send_string(" ");
	USART_Send_int((int32_t)(value * 100));
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
void setBitTo(uint8_t bit, uint8_t value, volatile uint8_t *reg) {
	*reg = (value << bit) | (*reg & (0xff - (1 << bit)));
}
uint16_t getPosition() {
	// PORTD2,3 selects HCTL-2022, PORTD4 enables
	// PORTC bits 0-3 read lower nibble of HCTL data
	// PORTB bits 0-3 read upper nibble of HCTL data
	uint16_t count = 0;

	// turn off clock
	TCCR0A = 0;
	TCCR0B = 0;
	
	//read MSB
	PORTD = 0b00000000 | (PORTD & 0b11110011);
	
	// toggle enable on D4
	setBitTo(4, 0, &PORTD);
	// clock low
	setBitTo(HCTL_CLK_BIT, 0, &PORTD);
	
	count = (count << 8) | (PINC << 4) | PINB;
	
	// read LSB
	PORTD = 0b00000100 | (PORTD & 0b11110011);

	count = count << 8 | (PINC << 4) | PINB;
	
	// turn on enable
	setBitTo(4, 1, &PORTD);
	
	// turn on clock
	TCCR0A = (0 << COM0A1) | (1 << COM0A0) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (0 << COM0B1) | (0 << COM0B0) | (1 << WGM02) | (1 << CS00);
	return count;
}
void sendSerial(uint8_t message) {
	uint8_t serialByte = message ^ 0xff;
	setBitTo(5, 0, &PORTD);
	_delay_us(51);
	for(uint8_t i = 1; i < 8; i++) {
		setBitTo(5, serialByte & 1, &PORTD);
		serialByte = serialByte >> 1;
		_delay_us(51);
	}
	setBitTo(5, 1, &PORTD);
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
			sendSerial((uint8_t)(192 + (d * 63.5)));
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
	// print("ADC: ", (uint32_t)analogIn);
}

int main(void) {
	
	// pwm0.setDuty(0.2); // pwm set to clock output at the moment
	
	DDRB = 0b11110000;	//B5 output: board LED
	DDRD = 0b11111111;
	PORTC = 0;
	DDRC = 0b11100000;
	PORTD = 0xff;
	
	//-Ulfuse:w:0x22:m // argument for AVRdude to get clock out
	
	//USART_Init(convertBaud(19200)); // motor serial speed
	USART_Init(convertBaud(57600)); // computer com speed
	
	ADMUX  = 0b01100100; // port A4 ADC selected
	ADCSRA = 0b10001011; // on, 2x clock
	ADCSRB = 0b00000000; // free running
	DIDR0  = 0b00010000;
	
	uint16_t count = 0;
	uint16_t desiredPosition = 3000;
	uint16_t lastPosition = 0, position = 0;
	int16_t velocity = 0;
	int32_t P = 0, I = (int32_t)desiredPosition - getPosition(), D = 0;
	int32_t E = 0, lastE = 0;
	double speed;
	char str[20];
	uint16_t analogIn = 0;
	
	SREG = SREG | 0x80;
	sei(); // enable global interrupts
	PRR = 0;
	
	while(0) {
		setBitTo(ADSC, 1, &ADCSRA);
	}
	
	while(0) {
		Motor::setSpeed(0.5);
		_delay_ms(1);
		print("Position: ", (uint32_t)getPosition());
		_delay_ms(1);
	}
	
	while(1) {	
		/*
		count++;
		if(count > 5000) {
			count = 0;
			if(desiredPosition == 3000)
				desiredPosition = 9000;
			else
				desiredPosition = 3000;
		}
		*/
		
		lastPosition = position;
		position = getPosition();
		velocity = position - lastPosition;
		
		/*
		if(lastPosition < 250 && position > 16250)
			E = (int32_t)desiredPosition - (0xFFFF - position);
		else if(lastPosition > 16250 && position < 250)
			E = (int32_t)desiredPosition - (0xFFFF + position);
		else
		*/
	
		desiredPosition = (10 * (analogHigh << 2) | (analogLow >> 6)) + 6000;
		lastE = E;	
		E = (int32_t)desiredPosition - position; 
		
		P = E;
		I = I + E;
		D = lastE - E;
		
		speed = (double)(2 * D + 0.5 * P) / 1000;
	
		Motor::setSpeed(speed);
		/*
		sprintf(str, "%u,", position);
		USART_Send_string(str);
		*/
		sprintf(str, "%i,", velocity);
		USART_Send_string(str);
		
		setBitTo(ADSC, 1, &ADCSRA); // ADC read start
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