#include "Button.h"
#include <cstdio>
#include <cstring>
#include <GL/freeglut.h>

/*
#define BUTTON_ACTIVE_COLOR_COMPONENT_RED
#define BUTTON_ACTIVE_COLOR_COMPONENT_GREEN
#define BUTTON_ACTIVE_COLOR_COMPONENT_BLUE

#define BUTTON_INACTIVE_COLOR_COMPONENT_RED
#define BUTTON_INACTIVE_COLOR_COMPONENT_GREEN
#define BUTTON_INACTIVE_COLOR_COMPONENT_BLUE
*/


void Button::draw() {
	glPushMatrix();
	glTranslated(posX, posY, 0);
	if (isClicked)
		glColor3f(1, 0, 0);
	else
		glColor3f(R, G, B);
	glRectd(-xLen/2.0, -yLen/2.0, xLen/2.0, yLen/2.0);
	glColor3f(1, 1, 1);
	glRasterPos2i(-5, -6);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(label));
	glPopMatrix();
}

Button::Button(double x, double y, double width, double height, float r, float g, float b, char *txt) {
	posX = x;
	posY = y;
	xLen = width;
	yLen = height;
	R = r;
	G = g;
	B = b;
	strncpy(label, txt, 9);
	label[9] = '\0';
	isClicked = false;
}

void Button::changeColour(float r, float g, float b) {
	R = r;
	G = g;
	B = b;
}

bool Button::isOnButton(int x, int y) {
	return (
		x > (posX - xLen/2.0) && x < (posX + xLen/2.0) &&
		y > (posY - yLen/2.0) && y < (posY + yLen/2.0)
	);
}

void Button::click() {
	isClicked = true;
}

void Button::unclick() {
	isClicked = false;
}

double Button::getX() {
	return posX;
}

double Button::getY() {
	return posY;
}

void Button::setPosition(double x, double y) {
	posX = x;
	posY = y;
}
