#include "GPSInputManager.h"
#include <cstdlib>
#include <sstream>
#include <cstring>

#define BKSPC 8

GPSInputManager::GPSInputManager() {
	index = 0;
	memset(buffer, '\0', BUFFER_SIZE);
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
	char dir = buffer[0];
	std::string s = &buffer[1];
	std::stringstream ss(s);
	std::string dms[3];
	for(int i = 0;i < 3;i++) {
		std::getline(ss, dms[i], ',');
	}
	double deg = atof(dms[0].c_str());
	double min = atof(dms[1].c_str());
	double sec = atof(dms[2].c_str());
	double dd = deg + min/60.0 + sec/3600.0;
	if(dir == 'S' || dir == 'W') dd *= -1;
	
	return dd;
}

bool GPSInputManager::DMSChars(unsigned char key) {
	return (key == 'W' || key == 'S' || key == 'E' || key == 'N');
}

void GPSInputManager::input(unsigned char key) {
	if(!enable) return;
	if((key >= '0' && key <= '9') || key == '.' || key == ',' || key == BKSPC || DMSChars(key)) {
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
