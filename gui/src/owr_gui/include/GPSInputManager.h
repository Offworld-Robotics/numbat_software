#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#define BUFFER_SIZE 50

class GPSInputManager {
	private:
		char buffer[BUFFER_SIZE];
		int index;
		bool enable;
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
