#include "Button.h"
#include "GLUTWindow.h"
#include <vector>

class FineControlGUI : public GLUTWindow {
	public:
		FineControlGUI(int width, int height, int *argc, char *argv[]);
		void idle();
		void display();
		void keydown(unsigned char key, int x, int y);
		void mouse(int button, int state, int x, int y);
		
		bool arrows[4];
		std::vector<Button*> buttons;
		GLuint vidTex[2];
                std::vector<Video_Feed_Frame*> videoFeeds;
};
