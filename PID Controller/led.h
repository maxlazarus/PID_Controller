#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

using namespace std;

class LED
{
	int PIN;
	unsigned char PORT;
	public:
		LED(unsigned char,int);
		void on(void);
		void off(void);	
};
