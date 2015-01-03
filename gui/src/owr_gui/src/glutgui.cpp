/*
 * Main source for OffWorld Robotics Widget Gui
 *
 * Draws a gui for ground station for BLUEsat OffWorld Robotics Groundstation
 *    Gui shows video feed controls
 *    GPS
 *    Signal strength
 *    Robot Battery level
 *
 * Contributers
 *
 */

// catkin_make // (add argument "-j4" for ros indigo)
// source devel/setup.bash
// rosrun owr_gui glutgui
#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <unistd.h>
#include <GL/glut.h>
#include "comms.h"
#include "GpsGUI.h"

//enable this to put in random data
//#define RANDOM
using namespace std;

#define PI 3.1415926535897932384626433832795

// default window size
#define WINDOW_W 1000
#define WINDOW_H 300

#define VID_FEED_ACTIVE_BUTTON_RED 0
#define VID_FEED_ACTIVE_BUTTON_GREEN 153
#define VID_FEED_ACTIVE_BUTTON_BLUE 0

#define VID_FEED_ACTIVE_NOT_LIVE_BUTTON_RED 235
#define VID_FEED_ACTIVE_NOT_LIVE_BUTTON_GREEN 133
#define VID_FEED_ACTIVE_NOT_LIVE_BUTTON_BLUE 51

#define VID_FEED_INACTIVE_BUTTON_RED 230
#define VID_FEED_INACTIVE_BUTTON_GREEN 0
#define VID_FEED_INACTIVE_BUTTON_BLUE 0

#define SCALE 150000
#define ARTIFICIAL_HORIZON_SKY_HEIGHT 50
#define ARTIFICIAL_HORIZON_SKY_HALF_WIDTH 70
#define ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH 40

#define UP    0
#define DOWN  1
#define LEFT  2
#define RIGHT 3

// OpenGL essential functions
void init();
void reshape(int w, int h);
void idle();
void display();

// OpenGL keyboard functions (mainly for debugging)
void keydown(unsigned char key, int x, int y);
//void keyup(unsigned char key, int x, int y);
void special_keydown(int keycode, int x, int y);
void special_keyup(int keycode, int x, int y);

// function to display some text
void drawText(char *text, int x, int y);
// function to insert a given co-ordinate to the front of the path list
void GPSAddPos(double x, double y);
// function to insert a random co-ordinate to the front of the path list
void GPSAddRandPos();
// function to generate a target co-ordinate
void generateTarget();
// function to print the path
void printGPSPath();
// draw functions
void drawFeeds();
void drawGPS();
void drawTilt();
void drawBattery();
void drawSignal();
void drawUltrasonic();

// default status values
float owr_battery = 0;
float owr_signal = 0;
float tiltX = 0; // tilt of left-right in degrees
float tiltY = 0; // tilt of forward-back in degrees
float ultrasonic = 0;
double longitude = 0;
double latitude = 0;
double prevAngle = 90;

// GPS related variables
ListNode path = NULL;
vector2D target;

// OpenGL control related variables
unsigned int currentWindowH = WINDOW_H;
unsigned int currentWindowW = WINDOW_W;
unsigned int frame = 0;
bool arrowKeys[3] = {0};

void updateConstants(float bat, float sig,float ultrason, ListNode points, vector2D tar) {
    owr_battery = bat;
    owr_signal = sig;
    path = points;
    target = tar;
    ultrasonic = ultrason;
    ROS_INFO("Updated");
}

int main(int argc, char **argv) {
	srand(time(NULL));
	generateTarget();
	ros::init(argc, argv, "GUI");
	GPSGUI *gpsnode = new GPSGUI(updateConstants);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_W, WINDOW_H);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("OWR GUI");
	init();
	glutKeyboardFunc(keydown);
	glutSpecialFunc(special_keydown);
	glutSpecialUpFunc(special_keyup);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);
	glutMainLoop();
	return 0;
}

void idle(void) {
	ros::spinOnce();
	
	/*// produce random tilt data
	int randn = rand() % 100 + 1;
	if (randn <= 25)
		tiltX++;
	else if (randn <= 50)
		tiltX--;
	else if (randn <= 75)
		tiltY++;
	else
		tiltY--;*/
	
	// debug - arrow key control for tilt
	if (arrowKeys[UP])
		tiltY--;
	if (arrowKeys[DOWN])
		tiltY++;
	if (arrowKeys[LEFT])
		tiltX--;
	if (arrowKeys[RIGHT])
		tiltX++;

   // clean tilt data
	if ((int) tiltY % 90 == 0) tiltY = 0;
	
	#ifdef RANDOM
	// debug - randomly generate GPS values every second
	if (frame == 0 || frame % 60 == 0) GPSAddRandPos();
	#endif
	
	// debug - arrow key control for path
	/*if (path != NULL) {
		double X = path->x, Y = path->y;
		double inc = 1.0/1000.0/200.0;
		if (arrowKeys[UP])
			Y += inc;
		if (arrowKeys[DOWN])
			Y -= inc;
		if (arrowKeys[LEFT])
			X -= inc;
		if (arrowKeys[RIGHT])
			X += inc;
		GPSAddPos(X, Y);
	} else if (path == NULL) {
		GPSAddRandPos();
	}*/
	
	// debug - print out the list of GPS co-ordinates
	//printGPSPath();
	
	#ifdef RANDOM	
	// debug - animate battery and signal
	owr_battery += 0.01;
	owr_signal -= 0.01;
	if (owr_battery < 0)
		owr_battery = 10;
	if (owr_battery > 10)
		owr_battery = 0;
	if (owr_signal < 0)
		owr_signal = 10;
	if (owr_signal > 10)
		owr_signal = 0;
	#endif
	
	frame++;
	if (frame > 6001)
		frame -= 6000;
		
	display();
	usleep(16666);
}

void display(void) {
	glClearColor(1,1,1,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	drawFeeds();
	drawGPS();
	drawTilt();
	drawBattery();
	drawSignal();
	drawUltrasonic();
	glutSwapBuffers();
}

void GPSAddRandPos() {
	double randx, randy;
	ListNode rest = path;
	path = (ListNode) malloc(sizeof(_vector2D));
	randx = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	randy = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	if (rest == NULL) {
		path->x = randx + 151.139;
		path->y = randy - 33.718;
	} else {
		randx -= 1.0/1000.0/2.0;
		path->x = rest->x + randx;
		randy -= 1.0/1000.0/2.0;
		path->y = rest->y + randy;
	}
	path->next = rest;
}

void GPSAddPos(double x, double y) {
	ListNode rest = path;
	path = (ListNode) malloc(sizeof(_vector2D));
	path->x = x;
	path->y = y;
	path->next = rest;
}

void generateTarget() {
	double randn;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	target.x = randn + 151.139;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	target.y = randn - 33.718;
	target.next = NULL;
}

void printGPSPath() {
	ListNode curr = path;
	cout << "Start" << endl;
	while (curr != NULL) {
		printf("%.15f, %.15f\n", curr->x, curr->y);
		curr = curr->next;
	}
	cout << "End" << endl;
}

double distance(double x1, double y1, double x2, double y2) {
	return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

// draws GPS path and co-ordinates near the centre of the window
void drawGPS() {
	char GPSLat[30];
	char GPSLong[30];
	char scale[] = "Scale: 1 metre";
	glPushMatrix();
	glTranslated(400, -WINDOW_H/3, 0);
	
	// draw the rover as a red square
	glPointSize(10);
	glBegin(GL_POINTS);
	glColor3f(1,0,0);
	glVertex2d(0, 0);
	glEnd();
		
	if (path != NULL) {
		longitude = path->x;
		latitude = path->y;
		
		// draw out the scale of the path
		glPushMatrix();
		glTranslated(-20, -WINDOW_H/3 - 15, 0);
		glColor3f(0,0,0);
		glScaled(SCALE,SCALE,1);
		glBegin(GL_LINE_STRIP);
		glColor3f(0,0,0);
		glVertex2d(0,0);
		glVertex2d(0.0001 / 0.9059, 0); // scale - this is 1 metre (approx)
		glEnd();
		glPopMatrix();
		glPushMatrix();
		glTranslated(10, -WINDOW_H/3 - 20, 0);
		drawText(scale, 0,0);
		glPopMatrix();
		
		// draws out the path so that the forward direction of the rover always facing up on the screen
		glPushMatrix();
		glScaled(SCALE,SCALE,1);
		
		glColor3f(0, 0, 1);
		double xoff = path->x, yoff = path->y;
		if (path->next != NULL) {
			double angle = -atan2(path->next->y - path->y, path->next->x - path->x) * 180.0/PI - 90;
			// if the rover did not move, its orientation is the same as previously
			if (angle == 0)
				angle = prevAngle;
			else
				prevAngle = angle;
			glRotated(angle, 0, 0, 1);
		}
		glTranslated(-xoff, -yoff, 0);
		ListNode curr = path;
		ListNode prev = NULL;
		
		glBegin(GL_LINE_STRIP);
		while (curr != NULL) {
			glVertex2d(curr->x, curr->y);
			prev = curr;
			curr = curr->next;
		}
		glEnd();
		
		// draw the origin point bigger
		glPointSize(10);
		glBegin(GL_POINTS);
		glVertex2d(prev->x, prev->y);
		glEnd();
		
		// draw the path to target green
		glColor3f(0,1,0);
		glBegin(GL_LINES);
		glVertex2d(path->x, path->y);
		glVertex2d(target.x, target.y);
		glEnd();
		glBegin(GL_POINTS);
		glVertex2d(target.x, target.y);
		glEnd();
		
		glPointSize(1);
		glPopMatrix();
		
		// draw text for GPS co-ordinates
		glColor3f(0, 0, 0);
		glTranslated(-50, -WINDOW_H/2, 0);
		sprintf(GPSLat, "Lat: %.10f", latitude);
		sprintf(GPSLong, "Lon: %.10f", longitude);
		drawText(GPSLat, 0, 0);
		glTranslated(0, -20, 0);
		drawText(GPSLong, 0, 0);
	}
	glPopMatrix();
}

// draw the ultrasonic
void drawUltrasonic() {
	char ultrasonicText[30];
	glPushMatrix();
	glTranslated(400, -WINDOW_H/3, 0);
		
	// draw text for GPS co-ordinates
	glColor3f(0, 0, 0);
	glTranslated(-50, 50.0, 0);
	sprintf(ultrasonicText, "Ultrasonic: %f m", ultrasonic);
	drawText(ultrasonicText, 0, 0);
	glPopMatrix();
}

// draws feeds boxes on the left side of the window
void drawFeeds(void) {
	glPushMatrix();
	
	glColor3ub(VID_FEED_ACTIVE_BUTTON_RED, VID_FEED_ACTIVE_BUTTON_GREEN, VID_FEED_ACTIVE_BUTTON_BLUE);
	glTranslated(50, -37.5, 0);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '1');
	
	glColor3ub(VID_FEED_ACTIVE_NOT_LIVE_BUTTON_RED, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_GREEN, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_BLUE);
	glTranslated(0, -75, 0);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '2');
	
	glColor3ub(VID_FEED_ACTIVE_NOT_LIVE_BUTTON_RED, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_GREEN, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_BLUE);
	glTranslated(0, -75, 0);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '3');
	
	glColor3ub(VID_FEED_INACTIVE_BUTTON_RED, VID_FEED_INACTIVE_BUTTON_GREEN, VID_FEED_INACTIVE_BUTTON_BLUE);
	glTranslated(0, -75, 0);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '4');
	
	glPopMatrix();
}

// draws the tilt angles for the left-right and front-back directions near the right of the screen
void drawTilt() {
	char text[30];
	glPushMatrix();

	glTranslated(720, -WINDOW_H/2, 0);

   glPushMatrix();

        // draw horizon
	glTranslated(0, 100*tan(-tiltY*PI/180),0);
	glRotated(-tiltX,0,0,1);
	glBegin(GL_LINE_STRIP);
	glVertex2d(-70,0);
	glVertex2d(70,0);
	glEnd();

        // draw sky indicator lines
        glPushMatrix();
	glColor3f(0,0,1);
        //glTranslated(0, 15, 0);
        glBegin(GL_QUADS);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	//glEnd();
        //glTranslated(0, 15, 0);
        //glBegin(GL_LINE_STRIP);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glEnd();
        glPopMatrix();

        // draw ground (ie below horizon) indicator lines
        glPushMatrix();
	glColor3f(0.1,0.1,0.1);
        glBegin(GL_QUADS);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	//glEnd();
        //glTranslated(0, 15, 0);
        //glBegin(GL_LINE_STRIP);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,-ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,-ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glEnd();
        glPopMatrix();

        glPopMatrix();

        //DRAW robot indicator
	glColor3f(1,0,0);
	glBegin(GL_LINE_STRIP);
	glVertex2d(-70,0);
	glVertex2d(70,0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glVertex2d(0,-70);
	glVertex2d(0,70);
	glEnd();

        // Draw Tilt-Text
	glTranslated(-50, -100, 0);
	sprintf(text, "Left-Right: %.2fdeg", tiltX);
	drawText(text, 0, 0);
	glTranslated(0, -20, 0);
	sprintf(text, "Front-Back: %.2fdeg", tiltY);
	drawText(text, 0, 0);
	
	glPopMatrix();
	
	// todo: write leftright inside the crosshair
}

// draws the battery level near the top-right of the window
void drawBattery() {
	glPushMatrix();
	
	if (owr_battery < 3)
		glColor3f(1,0,0);
	else
		glColor3f(0,1,0);
	if (owr_battery < 0)
		owr_battery = 0;
	if (owr_battery > 10)
		owr_battery = 10;
	
	glTranslated(currentWindowW - 125, -50, 0);
	glBegin(GL_LINE_LOOP);
	glVertex2i(0, 30);
	glVertex2i(0, -30);
	glVertex2i(100, -30);
	glVertex2i(100, 30);
	glEnd();
	glBegin(GL_QUADS);
	glVertex2i(0, 30);
	glVertex2i(0, -30);
	glVertex2i(owr_battery * 10, -30);
	glVertex2i(owr_battery * 10, 30);
	glEnd();
	
	glBegin(GL_LINE_LOOP);
	glVertex2i(100, 15);
	glVertex2i(100, -15);
	glVertex2i(110, -15);
	glVertex2i(110, 15);
	glEnd();
	
	glTranslated(0, -50, 0);
	glColor3f(0, 0, 0);
	char text[] = "battery";
	drawText(text, 15, 0);
	
	glPopMatrix();
}

// draw the signal level near the bottom-right of the window
void drawSignal() {
	glPushMatrix();
	
	if (owr_signal < 3)
		glColor3f(1,0,0);
	else
		glColor3f(0,1,0);
	if (owr_signal < 0)
		owr_signal = 0;
	if (owr_signal > 10)
		owr_signal = 10;
	
	glTranslated(currentWindowW - 125, -((int) currentWindowH - 100), 0);
	
	glBegin(GL_LINE_LOOP);
	glVertex2i(0, 0);
	glVertex2i(100, 0);
	glVertex2i(100, 50);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex2i(0, 0);
	glVertex2i(owr_signal * 10, 0);
	glVertex2i(owr_signal * 10, 5 * owr_signal);
	glEnd();
	
	glTranslated(0, -20, 0);
	glColor3f(0, 0, 0);
	char text[] = "signal";
	drawText(text, 25, 0);
	
	glPopMatrix();
}

void init(void) {
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_FLAT);
}

void keydown(unsigned char key, int x, int y) {
	switch (key) {
	case 27:
		exit(0);
		break;
	case '1':
		// activate feed 1
		break;
	case '2':
		// activate feed 2
		break;
	case '3':
		// activate feed 3
		break;
	case '4':
		// activate feed 4
		break;
	}
}

void special_keydown(int keycode, int x, int y) {
	switch (keycode) {
	case GLUT_KEY_UP:
		arrowKeys[0] = 1;
		break;
	case GLUT_KEY_DOWN:
		arrowKeys[1] = 1;
		break;
	case GLUT_KEY_LEFT:
		arrowKeys[2] = 1;
		break;
	case GLUT_KEY_RIGHT:
		arrowKeys[3] = 1;
		break;
	}
}

void special_keyup(int keycode, int x, int y) {
	switch (keycode) {
	case GLUT_KEY_UP:
		arrowKeys[0] = 0;
		break;
	case GLUT_KEY_DOWN:
		arrowKeys[1] = 0;
		break;
	case GLUT_KEY_LEFT:
		arrowKeys[2] = 0;
		break;
	case GLUT_KEY_RIGHT:
		arrowKeys[3] = 0;
		break;
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

void drawText(char *text, int x, int y) {
	for (unsigned int i = 0; i < strlen(text); i++) {
		glRasterPos3f(x, y, 0);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
		x += 10;
	}
}

void updateConstants(float bat, float sig, ListNode points, vector2D tar) {
	owr_battery = bat;
	owr_signal = sig;
	path = points;
	target = tar;
	ROS_INFO("Updated");
}
