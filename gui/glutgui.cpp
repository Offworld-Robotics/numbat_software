#include <iostream>
#include <string>
#include <string.h>
#include <math.h>
#include <GL/glut.h>

using namespace std;

#define WINDOW_WIDTH  500
#define WINDOW_HEIGHT 500

// Viewmodes
#define NONE 0
#define CAM1 1
#define CAM2 2
#define CAM3 3

void init();
void keyboard(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void reshape(int w, int h);
void display();

void drawBackground();
void drawButtons();
void displayText();

char viewMode = NONE;

unsigned int currentWindowH;
unsigned int currentWindowW;

int main(int argc, char *argv[]) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutInitWindowPosition(100, 100);
	glutCreateWindow(argv[0]);
	init();
	glutKeyboardFunc(keyboard);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(display);
	glutMouseFunc(mouse);
	glutMainLoop();
	return 0;
}

void drawButtons(void) {
	glPushMatrix();
	
	glColor3f(1.0, 1.0, 1.0);
	glTranslated(50, -50, 0);
	glRectd(-25, -25, 25, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos3d(-5, -6, 0);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '1');
	
	glColor3f(1.0, 1.0, 1.0);
	glTranslated(0, -75, 0);
	glRectd(-25, -25, 25, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos3d(-5, -6, 0);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '2');
	
	glColor3f(1.0, 1.0, 1.0);
	glTranslated(0, -75, 0);
	glRectd(-25, -25, 25, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos3d(-5, -6, 0);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '3');
	
	glPopMatrix();
}

void displayText() {
	string str;
	char *modeText;
	switch (viewMode) {
		case CAM1:
			str = "CAM 1 ACTIVE";
			break;
		case CAM2:
			str = "CAM 2 ACTIVE";
			break;
		case CAM3:
			str = "CAM 3 ACTIVE";
			break;
		default:
			str = "OFF";
	}
		
	modeText = &str[0];
	
	int length = strlen(modeText);
	
	glColor3f(1.0, 1.0, 1.0);
	int x = currentWindowW/2-length*20/2; // dynamic x-ordinate depending on text length
	for (int i = 0; i < length; i++) {
		glRasterPos3f(x, -25, 0);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, modeText[i]);
		x += 20;
	}
	glRasterPos3f(0, 0, 0);
}

void init(void) {
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_FLAT);
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 27:
		exit(0);
	}
}

void mouse(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		if (x > 25 && x < 75) {
			if (y > 25 && y < 75) {
				if (viewMode != CAM1)
					viewMode = CAM1;
				else
					viewMode = NONE;
			} else if (y > 100 && y < 150) {
				if (viewMode != CAM2)
					viewMode = CAM2;
				else
					viewMode = NONE;
			} else if (y > 175 && y < 225) {
				if (viewMode != CAM3)
					viewMode = CAM3;
				else
					viewMode = NONE;
			}
		}
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
	// replace following bit with code to draw the camera feed
	if (viewMode == CAM1)
		glClearColor(1.0, 0.0, 0.0, 0.0);
	else if (viewMode == CAM2)
		glClearColor(0.0, 1.0, 0.0, 0.0);
	else if (viewMode == CAM3)
		glClearColor(0.0, 0.0, 1.0, 0.0);
	else
		glClearColor(0.0, 0.0, 0.0, 0.0);

	glClear(GL_COLOR_BUFFER_BIT);
	drawButtons();
	displayText();
	glFlush();
	glutSwapBuffers();
}
