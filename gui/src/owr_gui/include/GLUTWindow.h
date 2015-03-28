#ifndef GLUTWINDOW_H
#define GLUTWINDOW_H

#include <GL/freeglut.h>

#define BMP_HEADER_SIZE 0x36

class GLUTWindow {
	protected:
		// GLUT essential functions
		void reshape(int w, int h); // not virtual because function is common for all GLUT apps
		virtual void idle() = 0;
		virtual void display() = 0;
		
		// GLUT keyboard functions
		virtual void keydown(unsigned char key, int x, int y) = 0;
		virtual void keyup(unsigned char key, int x, int y) = 0;
		virtual void special_keydown(int keycode, int x, int y) = 0;
		virtual void special_keyup(int keycode, int x, int y) = 0;
		
		// wrapper functions needed for GLUT because it is not compatible with c++ classes
		static void glut_reshape(int w, int h);
		static void glut_idle();
		static void glut_display();
		static void glut_keydown(unsigned char key, int x, int y);
		static void glut_keyup(unsigned char key, int x, int y);
		static void glut_special_keydown(int keycode, int x, int y);
		static void glut_special_keyup(int keycode, int x, int y);
		
		// window control variables
		int currWinH;
		int currWinW;
		int frameCount;
		
		// some commonly used functions
		
		// assuming texture has been generated, reads a bmp file and loads the data into the texture
		void loadTexture(char *filename, GLuint texture, GLenum format);
		
		// fill the data array with the BMP header depending on the width and height of the image
		void fillBMPHeader(unsigned char *data, int width, int height);
		
		// draws text using a specified font and raster position
		void drawText(char *text, void *font, int x, int y);
		
	public:
		GLUTWindow();
		void run();
		// static instance for c++ compatibility
		static GLUTWindow *instance;	
};

#endif // GLUTWINDOW_H
