#ifndef GLUTWINDOW_H
#define GLUTWINDOW_H

#include <GL/glut.h>

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
		
		// draws text at a specified font, raster position and color
		void drawText(char *text, void *font, int x, int y, float r, float g, float b, float alpha);
		
	public:
		GLUTWindow();
		
		// static instance for c++ compatibility
		static GLUTWindow *instance;	
};

#endif // GLUTWINDOW_H
