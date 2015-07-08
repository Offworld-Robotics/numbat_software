#include "Bluetongue.h"
#include <iostream>
#include <unistd.h>

using namespace std;

static void printStatus(struct status *s) {
	cout << "Battery voltage: " << s->batteryVoltage << endl;
}

int main() {
	Bluetongue* steve = new Bluetongue("/dev/ttyACM0");

	for (int i = 256; i < 500; i++) {
		struct status s = steve->update((i - 256)/256.0, (i - 256)/256.0);
        //if (s.roverOk == false) {
        //    delete steve;
        //    Bluetongue* steve = new Bluetongue("/dev/ttyACM0");
        //}
		printStatus(&s);
		usleep(100000);
	}
	delete steve;
	return 0;
}
	
