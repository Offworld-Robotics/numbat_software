#include "GLUTWindow.h"
#include <GL/freeglut.h>
#include <cstring>
#include <cstdio>

GLUTWindow *GLUTWindow::instance;

GLUTWindow::GLUTWindow() {
	instance = this;
	currWinH = 0;
	currWinW = 0;
	frameCount = 0;
}

void GLUTWindow::glut_display() {
	instance->display();
}

void GLUTWindow::glut_idle() {
	instance->idle();
}

void GLUTWindow::glut_reshape(int w, int h) {
	instance->reshape(w, h);
}

void GLUTWindow::glut_keydown(unsigned char key, int x, int y) {
	instance->keydown(key, x, y);
}

void GLUTWindow::glut_keyup(unsigned char key, int x, int y) {
	instance->keyup(key, x, y);
}

void GLUTWindow::glut_special_keydown(int keycode, int x, int y) {
	instance->special_keydown(keycode, x, y);
}

void GLUTWindow::glut_special_keyup(int keycode, int x, int y) {
	instance->special_keyup(keycode, x, y);
}

void GLUTWindow::reshape(int w, int h) {
	currWinH = h;
	currWinW = w;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, (GLdouble)w, -(GLdouble)h, 0); // (0,0) is top left corner of window, use cartesian co-ordinates
	glMatrixMode(GL_MODELVIEW);
}

void GLUTWindow::drawText(char *text, void *font, int x, int y, float r, float g, float b, float alpha) {
	glRasterPos2i(x, y);
	glColor4f(r, g, b, alpha);
	glutBitmapString(font, reinterpret_cast<const unsigned char *>(text));
}

void GLUTWindow::loadTexture(char *filename, GLuint texture, GLenum format) {
	int width, height, size;
	unsigned char dataStart;
	unsigned char *data;
	FILE *f;

	f = fopen(filename, "r");
	if (f == NULL) {
		printf("Could not open '%s'!\n", filename);
	} else {
		fseek(f, 0x0A, SEEK_SET);
		fread(&dataStart, sizeof(unsigned char), 1, f); // get the location of starting byte of pixel data
		fseek(f, 0x12, SEEK_SET);
		fread(&width, sizeof(int), 1, f); // get the width of the image
		fseek(f, 0x16, SEEK_SET);
		fread(&height, sizeof(int), 1, f); // get the height of the image
		size = width*height * 3;

		data = (unsigned char *)malloc(size*sizeof(unsigned char));
		fseek(f, dataStart, SEEK_SET);
		fread(data, size, 1, f);
		fclose(f);

		glBindTexture(GL_TEXTURE_2D, texture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, format, GL_UNSIGNED_BYTE, data);
		free(data);
	}
}
