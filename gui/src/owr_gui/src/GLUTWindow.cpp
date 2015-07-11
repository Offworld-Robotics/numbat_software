#include "GLUTWindow.h"
#include <GL/freeglut.h>
#include <cstring>
#include <cstdio>

GLUTWindow *GLUTWindow::instance;

GLUTWindow::GLUTWindow(int width, int height, int *argc, char *argv[], const char *title) {
	instance = this;
	currWinH = height;
	currWinW = width;
	frameCount = 0;
	
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(width, height);
	glutInitWindowPosition(0, 0);
	glutCreateWindow(title);
	glutIdleFunc(glut_idle);
	glutDisplayFunc(glut_display);
	glutReshapeFunc(glut_reshape);
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

void GLUTWindow::glut_mouse(int button, int state, int x, int y) {
	instance->mouse(button, state, x, y);
}

void GLUTWindow::run() {
	glutMainLoop();
}

void GLUTWindow::reshape(int w, int h) {
	currWinH = h;
	currWinW = w;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, (GLdouble)w, -(GLdouble)h, 0); // (0,0) is top left corner of window, use cartesian co-ordinates
	glMatrixMode(GL_MODELVIEW);
	//printf("%d, %d\n", w, h);
}

void GLUTWindow::drawText(char *text, void *font, int x, int y) {
	glRasterPos2i(x, y);
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

void GLUTWindow::fillBMPHeader(unsigned char *data, int width, int height) {
	memset(data, 0, BMP_HEADER_SIZE);
	int datasize = width*height*3;
	data[0x00] = 'B';
	data[0x01] = 'M';
	*(int *)&data[0x02] = datasize + BMP_HEADER_SIZE;
	*(int *)&data[0x0A] = BMP_HEADER_SIZE;
	data[0x0E] = 0x28;
	*(int *)&data[0x12] = width;
	*(int *)&data[0x16] = height;
	data[0x1A] = 0x01;
	data[0x1C] = 0x18;
	*(int *)&data[0x22] = datasize;
	data[0x26] = 0x25;
	data[0x27] = 0x16;
	data[0x2A] = 0x25;
	data[0x2B] = 0x16;
}

void GLUTWindow::saveBMPFile(char *filename, unsigned char *data, int width, int height) {
	int datasize = width*height*3;
	FILE *f = fopen(filename, "w");
	if (f == NULL) {
		printf("Failed to open %s for writing.\n", filename);
		return;
	}
	unsigned char *bmp = (unsigned char *)malloc(datasize + BMP_HEADER_SIZE);
	if (bmp == NULL) {
		printf("Failed to allocate memory for BMP.\n");
		return;
	}
	fillBMPHeader(bmp, width, height);
	memcpy(&bmp[BMP_HEADER_SIZE], data, datasize);
	fwrite(bmp, datasize + BMP_HEADER_SIZE, 1, f);
	fclose(f);
	free(bmp);
}
