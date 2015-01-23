
#include "led.h"
#include <avr/io.h>


LED::LED (unsigned char  p,int x)
{	
	PORT = p;
	PIN = x;
}

void LED::on(void)
{
	sbi(_SFR_IO8(PORT),PIN);
}

void LED::off(void)
{
	cbi(_SFR_IO8(PORT),PIN);
}

