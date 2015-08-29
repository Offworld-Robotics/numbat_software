#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#define BUFFER_SIZE 100

class GPSInputManager {
	private:
		char buffer[BUFFER_SIZE];
		int index;
		bool enable;
		bool DMSChars(unsigned char key);
	public:
		GPSInputManager();
		void enableInput();
		void disableInput();
		void input(unsigned char key);
		const char *getBuffer();
		double convert2Double();
		bool isEnabled();
		void clearBuffer();
};

#endif // INPUTMANAGER_H
