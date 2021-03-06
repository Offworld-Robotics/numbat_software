#ifndef GLUTWINDOW_H
#define GLUTWINDOW_H

/*
	GLUTWindow abstract class header
	Must derive from this class to create something useful
*/

#include <GL/freeglut.h>
#include <string>

#define BMP_HEADER_SIZE 0x36

class GLUTWindow {
	protected:
		// GLUT essential functions
		
		// Callback for resizing the window (has default)
		virtual void reshape(int w, int h);
		// Function for computations when the CPU is idle
		virtual void idle() = 0;
		// Function to draw the screen
		virtual void display() = 0;
		
		// GLUT keyboard/mouse callback functions
		virtual void keydown(unsigned char key, int x, int y) {}
		virtual void keyup(unsigned char key, int x, int y) {}
		virtual void special_keydown(int keycode, int x, int y) {}
		virtual void special_keyup(int keycode, int x, int y) {}
		virtual void mouse(int button, int state, int x, int y) {}
		
		// Wrapper functions needed for GLUT because it is not compatible with c++ classes
		static void glut_reshape(int w, int h);
		static void glut_idle();
		static void glut_display();
		static void glut_keydown(unsigned char key, int x, int y);
		static void glut_keyup(unsigned char key, int x, int y);
		static void glut_special_keydown(int keycode, int x, int y);
		static void glut_special_keyup(int keycode, int x, int y);
		static void glut_mouse(int button, int state, int x, int y);
		
		// Window control variables
		int currWinH;
		int currWinW;
		int frameCount;
		
		// Some commonly used functions
		
		// Assuming texture has been generated, reads a bmp file and loads the data into the texture
		// Parameter format is either GL_RGB or GL_BGR, usually GL_BGR for bmp files
		void loadTexture(char *filename, GLuint texture, GLenum format);
		
		// Fill the data array with the BMP header given the width and height of the image
		void fillBMPHeader(unsigned char *data, int width, int height);
		
		// Save BMP data to a file given its dimensions and a filename
		void saveBMPFile(std::string filename, unsigned char *data, int width, int height);
		
		// Draws text using a specified font and raster position
		void drawText(char *text, void *font, int x, int y);
		
	public:
		// Instantiates the class given the initial window size, command line arguments, and a title string
		GLUTWindow(int width, int height, int *argc, char *argv[], const char *title);
		
		// Loop of no return
		void run();
		
		// Static instance for c++ compatibility
		static GLUTWindow *instance;	
};

#endif // GLUTWINDOW_H
