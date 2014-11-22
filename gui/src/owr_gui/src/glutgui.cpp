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

// catkin_make
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

using namespace std;

#define PI 3.1415926535897932384626433832795

// default window size
#define WINDOW_W 1000
#define WINDOW_H 500

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
void init();
void keyboard(unsigned char key, int x, int y);
void reshape(int w, int h);
void display();

// function to display some text
void drawText(char *text, int x, int y);
// function to insert a co-ordinate to the front of the path list
void GPSAddRandPos();
// function to print the path
void printGPSPath();

// default status values
float owr_battery = 5;
float owr_signal = 5;
float tiltX = 0; // tilt of left-right in degrees
float tiltY = 0; // tilt of forward-back in degrees
double longitude = 0;
double latitude = 0;

// GPS related variables
ListNode path = NULL;
vector2D target;

// OpenGL control related variables
unsigned int currentWindowH = WINDOW_H;
unsigned int currentWindowW = WINDOW_W;
unsigned int frame = 0;

void updateConstants(float bat, float sig, ListNode points, vector2D tar) {
	owr_battery = bat;
	owr_signal = sig;
	path = points;
	target = tar;
	ROS_INFO("Updated");
}

int main(int argc, char **argv) {
	srand(time(NULL));
	ros::init(argc, argv, "GUI");
	GPSGUI *gpsnode = new GPSGUI(updateConstants);
	//gpsnode->spin();
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

void GPSAddRandPos() {
	ListNode rest = path;
	path = (ListNode) malloc(sizeof(_vector2D));
	double randn;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	path->x = randn + 151.139;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	path->y = randn - 33.718;
	path->next = rest;
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

// draws GPS path and co-ordinates near the centre of the window
void drawGPS() {
	char GPSLat[30];
	char GPSLong[30];
	char scale[] = "Scale: 1 metre";
	glPushMatrix();
	glTranslated(425, -200, 0);
	glColor3ub(255, 0, 0);
	
	// draw the rover as a red triangle
	/*glBegin(GL_POLYGON);
	glVertex2i(-15, 0);
	glVertex2i(15, 0);
	glVertex2i(0, 45);
	glEnd();*/
	
	if (frame == 0 || frame % 60 == 0) GPSAddRandPos(); // debug - randomly generate GPS values
		
	if (path != NULL) {
		longitude = path->x;
		latitude = path->y;
		//printGPSPath(); // debug - print out the list of GPS co-ordinates
		
		// draw out the scale of the path
		glPushMatrix();
		glTranslated(-20, -200, 0);
		glColor3f(0,0,0);
		glScaled(SCALE,SCALE,1);
		glBegin(GL_LINE_STRIP);
		glColor3f(0,0,0);
		glVertex2d(0,0);
		glVertex2d(0.0001 / 0.9059, 0); // scale - this is 1 metre (approx)
		glEnd();
		glPopMatrix();
		glPushMatrix();
		glTranslated(10, -205, 0);
		drawText(scale, 0,0);
		glPopMatrix();
		
		// draws out the path so that the forward direction of the rover always facing up on the screen
		glPushMatrix();
		glScaled(SCALE,SCALE,1);
		
		glColor3f(0, 0, 1);
		double xoff = path->x, yoff = path->y;
		if (path->next != NULL) {
			double angle = -atan2(path->next->y - path->y, path->next->x - path->x) * 180.0/PI - 90;
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
		glPointSize(1);
		glPopMatrix();
		
		// draw text for GPS co-ordinates
		glColor3f(0, 0, 0);
		glTranslated(-50, -250, 0);
		sprintf(GPSLat, "Lat: %.10f", latitude);
		sprintf(GPSLong, "Lon: %.10f", longitude);
		drawText(GPSLat, 0, 0);
		glTranslated(0, -20, 0);
		drawText(GPSLong, 0, 0);
	}
	glPopMatrix();
}

// draws feeds boxes on the left side of the window
void drawFeeds(void) {
	glPushMatrix();
	
	glColor3ub(VID_FEED_ACTIVE_BUTTON_RED, VID_FEED_ACTIVE_BUTTON_GREEN, VID_FEED_ACTIVE_BUTTON_BLUE);
	glTranslated(50, -50, 0);
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

	glTranslated(720, -250, 0);

        glPushMatrix();
        // produce random tilt data
	int randn = rand() % 100 + 1;
	if (randn <= 25)
		tiltX++;
	else if (randn <= 50)
		tiltX--;
	else if (randn <= 75)
		tiltY++;
	else
		tiltY--; 

        // clean tilt data
	if ((int) tiltY % 90 == 0) tiltY = 0;

        // daw horizon
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
	ros::spinOnce();
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
	frame++;
	if (frame > 6001)
		frame -= 6000;
}

void drawText(char *text, int x, int y) {
	for (unsigned int i = 0; i < strlen(text); i++) {
		glRasterPos3f(x, y, 0);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
		x += 10;
	}
}
