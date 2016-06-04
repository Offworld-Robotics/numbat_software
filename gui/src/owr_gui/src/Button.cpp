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

Button::Button(double x, double y, double width, double height, float r, float g, float b, char *txt, void (*downFunc)(void), void (*upFunc)(void)) : label(txt) {
	this->x = x;
	this->y = y;
	this->halfWidth = width/2.0;
	this->halfHeight = height/2.0;
	this->r = r;
	this->g = g;
	this->b = b;
	this->downFunc = downFunc;
	this->upFunc = upFunc;
	this->isClicked = false;
}

void Button::draw() {
	glPushMatrix();
	glLoadIdentity();
	glTranslated(x, y, 0);
	if (isClicked) {
		glColor3f(1, 0, 0);
	} else {
		glColor3f(r, g, b);
	}
	glRectd(-halfWidth, -halfHeight, halfWidth, halfHeight);
	glColor3f(1, 1, 1);
	glRasterPos2i(-5, -6);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(label.c_str()));
	glPopMatrix();
}

void Button::setColour(float r, float g, float b) {
	this->r = r;
	this->g = g;
	this->b = b;
}

bool Button::isInside(int x, int y) {
	return (
		x > (this->x - halfWidth) && x < (this->x + halfWidth) &&
		y > (this->y - halfHeight) && y < (this->y + halfHeight)
	);
}

void Button::click() {
	isClicked = true;
        //clickDownOperation();
        downFunc();
}

void Button::unclick() {
	isClicked = false;
        //clickUpOperation();
        upFunc();
}

double Button::getX() {
	return x;
}

double Button::getY() {
	return y;
}

void Button::setPosition(double x, double y) {
	this->x = x;
	this->y = y;
}

void Button::setDownFunc(void (*downFunc)(void)) {
	this->downFunc = downFunc;
}

void Button::setUpFunc(void (*upFunc)(void)) {
	this->upFunc = upFunc;
}

void Button::clickDownOperation() {
   // override this in derived classes
}

void Button::clickUpOperation() {
   // override this in derived classes
}
