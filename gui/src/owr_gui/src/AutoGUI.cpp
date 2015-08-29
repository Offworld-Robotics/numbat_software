#include "GLUTWindow.h"
#include "ListNode.h"
#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <list>
#include <limits>
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

void AutoGUI::drawDividingLine() {
	glPushMatrix();
	glColor3f(1,1,1);
	glBegin(GL_LINES);
	glVertex2d(currWinW/3.0, 0);
	glVertex2d(currWinW/3.0, -currWinH);
	glEnd();
	glPopMatrix();
}

void AutoGUI::drawGPSDests() {
	glPushMatrix();
	
	glTranslated(50, -50, 0);
	char txt[100];
	glColor3f(1,0,0);
	sprintf(txt, "Input format: [NSEW]deg,min,sec");
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -30, 0);
	glColor3f(1,1,1);
	sprintf(txt, "Currently inputting destination %d", destNum);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -30, 0);
	sprintf(txt, "Text buffer: %s", keymanager->getBuffer());
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -30, 0);
	
	for(int i = 0;i < NUM_DESTS;i++) {
		sprintf(txt, "Dest %d Lat: %.10f", i, dests[i][0]);
		drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
		glTranslated(0, -30, 0);
		sprintf(txt, "Dest %d Lon: %.10f", i, dests[i][1]);
		drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
		glTranslated(0, -30, 0);
	}
	
	
	glPopMatrix();
}

void AutoGUI::display() {
	glClear(GL_COLOR_BUFFER_BIT);
	
	if(haveDests) {
		drawDividingLine();
		
		drawOverviewMap();
		drawTrackingMap();
		drawGPSPos();
		
		
	}
	
	drawGPSDests();	
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
			} else {
				keymanager->disableInput();
			}
			break;
		case 'c':
			keymanager->clearBuffer();
			break;
		case 'a':
			if(keymanager->isEnabled()) {
				dests[destNum][0] = keymanager->convert2Double();
			}
			break;
		case 'o':
			if(keymanager->isEnabled()) {
				dests[destNum][1] = keymanager->convert2Double();
			}
			break;
		case 'n':
			if(!haveDests && destNum < NUM_DESTS-1) {
				destNum++;
			}
			break;
		case 'b':
			if(!haveDests && destNum > 0) {
				destNum--;
			}
			break;
		case 'p':
			//publishGPS();
			haveDests = true;
			break;
		default:
			keymanager->input(key);
			break;
	}
}


/*void AutoGUI::publishGPS(GPSData gps) {
	sensor_msgs::NavSatFix msg;
	msg.longitude = ((float)gps.longitude)/GPS_FLOAT_OFFSET;
	msg.latitude = ((float)gps.latitude)/GPS_FLOAT_OFFSET;
	msg.altitude = gps.altitude;
	
	if (gps.fixValid) {
		msg.status.status = msg.status.STATUS_FIX;
	} else {
		msg.status.status = msg.status.STATUS_NO_FIX;
	}
	msg.status.service = msg.status.SERVICE_GPS; //NOt sure this is right
	msg.header.seq = gpsSequenceNum;
	msg.header.frame_id = 1; // global frame
	gpsPublisher.publish(msg);
}*/

AutoGUI::AutoGUI(int width, int height, int *argc, char *argv[]) : GLUTWindow(width, height, argc, argv, "AutoGUI") {
	autoNode = new AutoNode(this);
	
	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	
	memset(arrows, 0, 4*sizeof(bool));
	
	haveDests = false;
	
	double maxdouble = std::numeric_limits<float>::max();
	
	currentPos.lat = currentPos.lat = maxdouble;
	currentPos.alt = 0;
	
	for(int i = 0;i < NUM_DESTS;i++)
		dests[i][0] = dests[i][1] = maxdouble;
	destNum = 0;
	
	keymanager = new GPSInputManager();
	haveTargetLat = haveTargetLon = false;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "AutoGUI");
	AutoGUI gui(1855, 1056, &argc, argv);
	gui.run();
	return 0;
}
