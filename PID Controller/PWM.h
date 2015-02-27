using namespace std;

class PWM {
	private:
		unsigned short PWM_number;
		unsigned short savedState;
	public:
		PWM(unsigned short i);
		void init();
		void setDuty(double d);
		unsigned int read();
		void start();
		void stop();
};