#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#define BUFFER_SIZE 100

class GPSInputManager {
	private:
		char buffer[BUFFER_SIZE];
		int index;
		bool enable;
		bool compassChars(unsigned char key);
		bool numericChars(unsigned char key);
	public:
		GPSInputManager();
		void enableInput();
		void disableInput();
		void input(unsigned char key);
		const char *getBuffer();
		void convert2DD(double *lat, double *lon);
		void str2DD(char *str, double *lat, double *lon);
		bool isEnabled();
		void clearBuffer();
};

#endif // INPUTMANAGER_H
