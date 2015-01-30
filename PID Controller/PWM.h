using namespace std;

class PWM {
	public:
		PWM(unsigned short i);
		void init();
		static void init(unsigned short i);
		unsigned int read();
};