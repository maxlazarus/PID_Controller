using namespace std;

class PWM {
	private:
		unsigned short PWM_number;
	public:
		PWM(unsigned short i);
		void init();
		void setDuty(double d);
		static void init(unsigned short i);
		unsigned int read();
};