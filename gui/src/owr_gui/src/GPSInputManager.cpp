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

void GPSInputManager::str2DD(char *str, double *lat, double *lon) {
	if(strlen(str) > 1) {
		double coords[2];
		std::string s = str; // convert buffer into c++ string
		std::stringstream ss(s); // convert string into stream
		std::string sgps[2]; // to store the lat/lon substrings
		// split the original string
		for(int i = 0;i < 2;i++) {
			std::getline(ss, sgps[i], '/');
		}
		
		for(int i = 0;i < 2;i++) {
			// grab the first character - the compass directions
			char dir = sgps[i].at(0);
			// remove the first character
			sgps[i] = sgps[i].substr(1);
			// make into stream
			std::stringstream sss(sgps[i]);
			// make space to store dms string and values
			std::string sdms;
			double dms[3];
			// grab the d,m,s and convert to double
			for(int j = 0;j < 3;j++) {
				std::getline(sss, sdms, ',');
				dms[j] = atof(sdms.c_str());
			}
			coords[i] = dms[0] + dms[1]/60.0 + dms[2]/3600.0;
			if(dir == 'S' || dir == 'W') coords[i] *= -1;
		}
		*lat = coords[0];
		*lon = coords[1];
	}
}

void GPSInputManager::convert2DD(double *lat, double *lon) {
	str2DD(buffer, lat, lon);
}
	
bool GPSInputManager::compassChars(unsigned char key) {
	return (key == 'W' || key == 'S' || key == 'E' || key == 'N');
}

bool GPSInputManager::numericChars(unsigned char key) {
	return (key >= '0' && key <= '9');
}

void GPSInputManager::input(unsigned char key) {
	if(!enable) return;
	if(numericChars(key) || key == '.' || key == ',' || key == '/' || key == BKSPC || compassChars(key)) {
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
