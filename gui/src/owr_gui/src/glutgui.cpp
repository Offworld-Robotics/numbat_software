// catkin_make
// source devel/setup.bash
// rosrun owr_gui glutgui
#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <GL/glut.h>
//#include "comms.h"

using namespace std;

#define PI 3.1415926535897932384626433832795

#define WINDOW_W 1000
#define WINDOW_H 500

void init();
void keyboard(unsigned char key, int x, int y);
void reshape(int w, int h);
void display();

void drawBackground();
void drawButtons();
void drawBattery();
void drawSignal();
void displayText();

float battery = 0;
float signal = 0;
float tiltX = 30; // degrees
float tiltY = 30; // degrees

double longitude = 0;
double latitude = 0;

unsigned int currentWindowH = WINDOW_H;
unsigned int currentWindowW = WINDOW_W;

int main(int argc, char *argv[]) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_W, WINDOW_H);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("OWR GUI");
	init();
	glutKeyboardFunc(keyboard);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(display);
	glutMainLoop();
	return 0;
}

void drawFeeds(void) {
	glPushMatrix();
	
	glColor3ub(0, 153, 0);
	glTranslated(50, -50, 0);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '1');
	
	glColor3ub(255, 133, 51);
	glTranslated(0, -75, 0);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '2');
	
	glColor3ub(255, 133, 51);
	glTranslated(0, -75, 0);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '3');
	
	glColor3ub(230, 0, 0);
	glTranslated(0, -75, 0);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '4');
	
	glPopMatrix();
}

void drawGPS() { // far from complete
	glPushMatrix();
	glTranslated(425, -275, 0);
	glColor3ub(255, 0, 0);
	
	glBegin(GL_POLYGON);
	glVertex2i(-25, 0);
	glVertex2i(25, 0);
	glVertex2i(0, 75);
	glEnd();
	
	glColor3f(0, 0, 0);
	glTranslated(-50, -150, 0);
	char GPSLat[20];
	char GPSLong[20];
	sprintf(GPSLat, "Lat: %.10f", latitude);
	sprintf(GPSLong, "Long: %.10f", longitude);
	int x = 0;
	for (unsigned int i = 0; i < strlen(GPSLat); i++) {
		glRasterPos3f(x, 0, 0);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, GPSLat[i]);
		x += 10;
	}
	glTranslated(0, -20, 0);
	x = 0;
	for (unsigned int i = 0; i < strlen(GPSLong); i++) {
		glRasterPos3f(x, 0, 0);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, GPSLong[i]);
		x += 10;
	}
	
	glPopMatrix();
}

void drawTilt() {
	glPushMatrix();
	glTranslated(700, -150, 0);
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);
	glVertex2d(0, 0);
	glVertex2d(100, 0);
	glVertex2d(-100*sin(tiltX * PI / 180), 100*cos(tiltX * PI / 180));
	glEnd();
	glTranslated(0, -50, 0);
	char text[30];
	sprintf(text, "X: %.2fdeg", tiltX);
	int x = 0;
	for (unsigned int i = 0; i < strlen(text); i++) {
		glRasterPos3f(x, 0, 0);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
		x += 10;
	}
	
	glTranslated(0, -200, 0);
	glBegin(GL_LINES);
	glVertex2d(0, 0);
	glVertex2d(100, 0);
	glVertex2d(-100*sin(tiltY * PI / 180), 100*cos(tiltY * PI / 180));
	glEnd();
	glTranslated(0, -50, 0);
	sprintf(text, "Y: %.2fdeg", tiltY);
	x = 0;
	for (unsigned int i = 0; i < strlen(text); i++) {
		glRasterPos3f(x, 0, 0);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
		x += 10;
	}
	glPopMatrix();
}

void drawBattery() {
	glPushMatrix();
	
	if (battery < 1)
		glColor3f(1,0,0);
	else
		glColor3f(0,1,0);
	
	glTranslated(currentWindowW - 125, -50, 0);
	glBegin(GL_LINE_LOOP);
	glVertex2i(0, 30);
	glVertex2i(0, -30);
	glVertex2i(105, -30);
	glVertex2i(105, 30);
	glEnd();
	
	glBegin(GL_LINE_LOOP);
	glVertex2i(105, 15);
	glVertex2i(105, -15);
	glVertex2i(115, -15);
	glVertex2i(115, 15);
	glEnd();
	
	glPopMatrix();
}

void drawSignal() {
	glPushMatrix();
	if (signal < 1)
		glColor3f(1,0,0);
	else
		glColor3f(0,1,0);
	
	glTranslated(currentWindowW - 150, -((int) currentWindowH - 100), 0);
	
	glBegin(GL_POLYGON);
	glVertex2i(0, 0);
	glVertex2i(100, 0);
	glVertex2i(100, 50);
	glEnd();
	
	glPopMatrix();
}

void init(void) {
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_FLAT);
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 27:
		exit(0);
	}
}

void reshape(int w, int h) {
	currentWindowH = h;
	currentWindowW = w;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, (GLdouble)w, -(GLdouble)h, 0); // (0,0) is top left corner of window, use cartesian co-ordinates
	glMatrixMode(GL_MODELVIEW);
}

void display(void) {
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	drawFeeds();
	drawGPS();
	drawTilt();
	drawBattery();
	drawSignal();
	glFlush();
	glutSwapBuffers();
	usleep(16666);
}
