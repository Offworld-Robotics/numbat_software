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

/* default constructor
 */
Button::Button() {

}
Button::Button(double x, double y, double width, double height, float r, float g, float b, char *txt) {
	posX = x;
	posY = y;
	xHalfLen = width/2.0;
	yHalfLen = height/2.0;
	R = r;
	G = g;
	B = b;
	strncpy(label, txt, 9);
	label[9] = '\0';
	isClicked = false;
}

void Button::draw() {
	glPushMatrix();
	glTranslated(posX, posY, 0);
	if (isClicked)
		glColor3f(1, 0, 0);
	else
		glColor3f(R, G, B);
	glRectd(-xHalfLen, -yHalfLen, xHalfLen, yHalfLen);
	glColor3f(1, 1, 1);
	glRasterPos2i(-5, -6);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(label));
	glPopMatrix();
}

void Button::changeColour(float r, float g, float b) {
	R = r;
	G = g;
	B = b;
}

bool Button::isPointInBounds(int x, int y) {
	return (
		x > (posX - xHalfLen) && x < (posX + xHalfLen) &&
		y > (posY - yHalfLen) && y < (posY + yHalfLen)
	);
}

void Button::click() {
	isClicked = true;
        clickDownOperation();
}

void Button::unclick() {
	isClicked = false;
        clickUpOperation();
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

virtual void Button::clickDownOperation() {
   // override this in derived classes
}

virtual void Button::clickUpOperation() {
   // override this in derived classes
}
