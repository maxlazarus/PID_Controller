/*
 *  button.h
 *  February 2015
 *  Author: Max Prokopenko
 *  max@theprogrammingclub.com
 */ 

#ifndef button_h
#define button_h

using namespace std;

class Button {
	private:
		uint8_t bit;
		volatile uint8_t *port;
	
	public:
		Button(uint8_t bit, volatile uint8_t *port);
		bool isUp(void);
};

Button::Button(uint8_t bit, volatile uint8_t *port) {
	this->bit = bit;
	this->port = port;
}

bool Button::isUp() {
	return *(this->port) & (1 << this->bit);
}

#endif