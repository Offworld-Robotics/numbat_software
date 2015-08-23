#include "GPSInputManager.h"
#include <cstdlib>

#define BKSPC 8

GPSInputManager::GPSInputManager() {
	index = 0;
	buffer[0] = '\0';
	enable = false;
}

void GPSInputManager::enableInput() {
	enable = true;
}

void GPSInputManager::disableInput() {
	enable = false;
}

const char *GPSInputManager::getBuffer() {
	return buffer;
}

double GPSInputManager::convert2Double() {
	return atof(buffer);
}

void GPSInputManager::input(unsigned char key) {
	if(!enable) return;
	if((key >= '0' && key <= '9') || key == '.' || key == '-' || key == BKSPC) {
		if(key == BKSPC) {
			index--;
			if(index < 0) index = 0;
			buffer[index] = '\0';
		} else if(index < BUFFER_SIZE - 1) {
			buffer[index++] = key;
			buffer[index] = '\0';
		}
	}
}

bool GPSInputManager::isEnabled() {
	return enable;
}

void GPSInputManager::clearBuffer() {
	index = 0;
	buffer[0] = '\0';
}
