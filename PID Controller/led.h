#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

using namespace std;

class LED
{
	unsigned int  PIN;
	unsigned char PORT;
	public:
		LED(unsigned char,unsigned int );
		void on(void);
		void off(void);	
};
