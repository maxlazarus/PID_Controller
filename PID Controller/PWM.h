using namespace std;

class PWM {
	private:
		unsigned short PWM_number;
		unsigned short prescaler;	
	public:
		PWM(unsigned short i);
		void init(unsigned short i);
		static void init();
		void setDuty(double d);
		void start();
		void stop();
		void setPrescaler(int n);
		unsigned int read();
};