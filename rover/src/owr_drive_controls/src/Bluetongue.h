#include <cstdint>
#include <string>
#include <cstdint>
#include <vector>

struct status {
    bool roverOk;
    double batteryVoltage;
};
	
class Bluetongue {
	private:
		void comm(bool forBattery, void *message, int message_len, void *resp, 
				int resp_len);
		int port_fd;
        fd_set uart_set;
        struct timeval timeout;
	
	public:
		Bluetongue(const char* port);
		~Bluetongue();
		struct status update(double leftMotor, double rightMotor);
};

