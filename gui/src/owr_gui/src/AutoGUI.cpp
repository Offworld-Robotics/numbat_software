#include "GLUTWindow.h"
#include "ListNode.h"
#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <list>
#include "AutoGUI.h"
#include "AutoNode.h"
#include <ros/ros.h>
#include "GPSInputManager.h"

void AutoGUI::updateInfo(ListNode cur) {
	if (cur != NULL) path.push_front(cur);
	memcpy(&currentPos, cur, sizeof(vector3D));
}

void AutoGUI::drawFullMap(double refLat, double refLon) {
	glPushMatrix();
	glPointSize(5);
	
	glBegin(GL_POINTS);
	glColor3f(1, 0, 0);
	glVertex2d(dests[0][1] - refLon, dests[0][0] - refLat);
	glColor3f(0, 0, 1);
	glVertex2d(dests[1][1] - refLon, dests[1][0] - refLat);
	glColor3f(0, 1, 0);
	glVertex2d(dests[2][1] - refLon, dests[2][0] - refLat);
	glEnd();
	
	// THIS ANIMATES
	// see http://www.felixgers.de/teaching/jogl/stippledLines.html
	glLineStipple(1000, 0xAAAA);
	glEnable(GL_LINE_STIPPLE);
	glBegin(GL_LINE_STRIP);
	for (int i = 2;i >= 0;i--)
		glVertex2d(dests[i][1] - refLon, dests[i][0] - refLat);
	glEnd();
	
	glDisable(GL_LINE_STIPPLE);
	
	glPointSize(1);
	
	if(path.size()) {
		glColor3f(0,0,1);
		glBegin(GL_LINE_STRIP);
		for (std::list<ListNode>::iterator i = path.begin(); i != path.end(); ++i)
			glVertex2d((*i)->lon - refLon, (*i)->lat - refLat);
		glEnd();
		
		glPointSize(5);
		glBegin(GL_POINTS);
		for (std::list<ListNode>::iterator i = path.begin(); i != path.end(); ++i)
			glVertex2d((*i)->lon - refLon, (*i)->lat - refLat);
		glEnd();
		glPointSize(1);
	}
	glPopMatrix();
}

void AutoGUI::idle() {
	display();
	usleep(15000);
}

void AutoGUI::drawOverviewMap() {
	glPushMatrix();
	
	glTranslated(currWinW/6.0, -currWinH/2.0, 0);
	
	glScaled(SCALE, SCALE, 1);
	
	drawFullMap(mapCentre[0], mapCentre[1]);
	glPopMatrix();
}

void AutoGUI::drawTrackingMap() {
	glPushMatrix();
	
	glTranslated(currWinW - currWinW/3.0, -currWinH/2.0, 0);
	
	glScaled(1.5*SCALE, 1.5*SCALE, 1);
	
	drawFullMap(currentPos.lat, currentPos.lon);
	
	glPopMatrix();
}

void AutoGUI::drawGPSPos() {
	glPushMatrix();
	
	glTranslated(0.75*currWinW, -0.85*currWinH, 0);
	
	char GPSLat[30];
	char GPSLong[30];
	char GPSAlt[30];
	if (path.size() == 0) {
		sprintf(GPSLat, "Lat: Unknown");
		sprintf(GPSLong, "Long: Unknown");
		sprintf(GPSAlt, "Alt: Unknown");
	} else {
		sprintf(GPSLat, "Lat: %.10f", (*path.begin())->lat);
		sprintf(GPSLong, "Long: %.10f", (*path.begin())->lon);	
		sprintf(GPSAlt, "Alt: %.10f", (*path.begin())->alt);
	}
	
	glColor3f(1, 1, 1);
	drawText(GPSLat, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -20, 0);
	drawText(GPSLong, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -20, 0);
	drawText(GPSAlt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glPopMatrix();
}

void AutoGUI::display() {
	glClear(GL_COLOR_BUFFER_BIT);
	
	// draw dividing line
	glPushMatrix();
	glColor3f(1,1,1);
	glBegin(GL_LINES);
	glVertex2d(currWinW/3.0, 0);
	glVertex2d(currWinW/3.0, -currWinH);
	glEnd();
	glPopMatrix();
	
	drawOverviewMap();
	drawTrackingMap();
	drawGPSPos();
	
	glPushMatrix();
	
	glTranslated(50, -100, 0);
	glColor3f(1,1,1);
	char txt[100];
	sprintf(txt, "Text buffer: %s", keymanager->getBuffer());
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -30, 0);
	if(haveTargetLat) {
		sprintf(txt, "Target Lat: %.10f", targetLat);
	} else {
		sprintf(txt, "Target Lat: ?");
	}
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glTranslated(0, -30, 0);
	if(haveTargetLon) {
		sprintf(txt, "Target Lon: %.10f", targetLon);
	} else {
		sprintf(txt, "Target Lon: ?");
	}
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glPopMatrix();
	
	glutSwapBuffers();
}

void AutoGUI::keydown(unsigned char key, int x, int y) {
	switch (key) {
		case 27:
			exit(0);
			break;
		case 'i':
			if(!keymanager->isEnabled()) {
				keymanager->enableInput();
			}
			break;
		case 'a':
			if(keymanager->isEnabled()) {
				targetLat = keymanager->convert2Double();
				keymanager->clearBuffer();
				keymanager->disableInput();
				haveTargetLat = true;
			}
			break;
		case 'o':
			if(keymanager->isEnabled()) {
				targetLon = keymanager->convert2Double();
				keymanager->clearBuffer();
				keymanager->disableInput();
				haveTargetLon = true;
			}
			break;
		default:
			keymanager->input(key);
			break;
	}
}

AutoGUI::AutoGUI(int width, int height, int *argc, char *argv[], double destPos[3][2]) : GLUTWindow(width, height, argc, argv, "AutoGUI") {
	autoNode = new AutoNode(this);
	
	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	
	memset(arrows, 0, 4*sizeof(bool));
	memcpy(dests, destPos, 6*sizeof(double));
	mapCentre[0] = (dests[0][0]+dests[1][0]+dests[2][0])/3.0;
	mapCentre[1] = (dests[0][1]+dests[1][1]+dests[2][1])/3.0;
	
	memset(&currentPos, 0, sizeof(currentPos));
	
	keymanager = new GPSInputManager();
	haveTargetLat = haveTargetLon = false;
	
	ListNode init = (ListNode)malloc(sizeof(vector3D));
	init->lat = -33.9178303;
	init->lon = 151.2318155;
	init->alt = 0;
	path.push_front(init);
	memcpy(&currentPos, init, sizeof(vector3D));
	
	init = (ListNode)malloc(sizeof(vector3D));
	init->lat = -33.9188303;
	init->lon = 151.2328155;
	init->alt = 0;
	path.push_front(init);
	
	init = (ListNode)malloc(sizeof(vector3D));
	init->lat = -33.914873;
	init->lon = 151.225468;
	init->alt = 0;
	path.push_front(init);
}

int main(int argc, char *argv[]) {
	double dest[3][2] = {
		{-33.914873, 151.225468},
		{-33.916547, 151.236519},
		{-33.920643, 151.235553}
	};
	/*for(int i = 0;i<3;i++) {
		for(int j = 0;j<2;j++) {
			scanf("%lf", &dest[i][j]);
		}
	}*/
	ros::init(argc, argv, "AutoGUI");
	AutoGUI gui(1855, 1056, &argc, argv, dest);
	gui.run();
	return 0;
}
