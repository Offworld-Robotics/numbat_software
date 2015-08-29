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
	for (int i = NUM_DESTS-1;i >= 0;i--)
		glVertex2d(dests[i][1] - refLon, dests[i][0] - refLat);
	glEnd();
	
	// THIS ANIMATES
	// see http://www.felixgers.de/teaching/jogl/stippledLines.html
	glLineStipple(1000, 0xAAAA);
	glColor3f(0,1,0);
	glEnable(GL_LINE_STIPPLE);
	glBegin(GL_LINE_STRIP);
	for (int i = 0;i < NUM_DESTS;i++)
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
	
	glScaled(scale[0], scale[0], 1);
	
	drawFullMap(mapCentre[0], mapCentre[1]);
	glPopMatrix();
}

void AutoGUI::drawTrackingMap() {
	glPushMatrix();
	
	glTranslated(currWinW - currWinW/3.0, -currWinH/2.0, 0);
	
	glScaled(1.5*scale[1], 1.5*scale[1], 1);
	
	drawFullMap(currentPos.lat, currentPos.lon);
	
	glPopMatrix();
}

void AutoGUI::drawScales() {
	glPushMatrix();
	glTranslated(0.75*currWinW, -0.1*currWinH, 0);
	
	char scaleL[30];
	char scaleR[30];
	sprintf(scaleL, "L Map Scale: %f", scale[0]);
	sprintf(scaleR, "R Map Scale: %f", scale[1]);
	
	glColor3f(1, 1, 1);
	drawText(scaleL, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -20, 0);
	drawText(scaleR, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
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
	if(!haveDests) {
		glColor3f(1,0,0);
		sprintf(txt, "Input format: [NS]deg,min,sec/[EW]deg,min,sec");
		drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
		glTranslated(0, -30, 0);
		glColor3f(1,1,1);
		sprintf(txt, "Currently inputting destination %d", destNum);
		drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
		glTranslated(0, -30, 0);
		sprintf(txt, "Text buffer: %s", keymanager->getBuffer());
		drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
		glTranslated(0, -30, 0);
	}
	
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
	drawScales();
	drawGPSDests();
	glutSwapBuffers();
	usleep(15000);
}

void AutoGUI::special_keydown(int keycode, int x, int y) {
	if(keycode == GLUT_KEY_UP) {
		scale[1] += 1000;
	} else if(keycode == GLUT_KEY_DOWN) {
		scale[1] -= 1000;
	} else if(keycode == GLUT_KEY_LEFT) {
		scale[0] -= 1000;
	} else if(keycode == GLUT_KEY_RIGHT) {
		scale[0] += 1000;
	}
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
		case 13:
			keymanager->convert2DD(&dests[destNum][0], &dests[destNum][1]);
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
		case 'P':
			//publishGPS();
			keymanager->clearBuffer();
			for(int i = 0;i < NUM_DESTS;i++) {
				mapCentre[0] += dests[i][0];
				mapCentre[1] += dests[i][1];
			}
			mapCentre[0] /= NUM_DESTS;
			mapCentre[1] /= NUM_DESTS;
			haveDests = true;
			break;
		default:
			keymanager->input(key);
			break;
	}
}

void AutoGUI::publishGPS(double lat, double lon) {
	sensor_msgs::NavSatFix msg;
	msg.longitude = lon;
	msg.latitude = lat;
	gpsPublisher.publish(msg);
	ros::spinOnce();
}

AutoGUI::AutoGUI(int width, int height, int *argc, char *argv[]) : GLUTWindow(width, height, argc, argv, "AutoGUI") {
	autoNode = new AutoNode(this);
	
	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	glutSpecialFunc(glut_special_keydown);
	
	memset(arrows, 0, 4*sizeof(bool));
	
	haveDests = false;
	
	double maxdouble = std::numeric_limits<float>::max();
	
	currentPos.lat = currentPos.lat = maxdouble;
	currentPos.alt = 0;
	
	for(int i = 0;i < NUM_DESTS;i++)
		dests[i][0] = dests[i][1] = maxdouble;
	destNum = 0;
	mapCentre[0] = mapCentre[1] = 0;
	
	keymanager = new GPSInputManager();
	
	scale[0] = scale[1] = SCALE;
	
	ListNode l = (ListNode)malloc(sizeof(vector3D));
	l->lat = -33.91779377339266;
	l->lon = 151.23166680335999;
	l->alt = 0;
	path.push_front(l);
	
	currentPos.lat = l->lat;
	currentPos.lon = l->lon;
	currentPos.alt = l->alt;
	
	
	char d0[] = "S33,54,53.673/E151,13,31.907";
	char d1[] = "S33,54,59.763/E151,14,11.458";
	char d2[] = "S33,55,14.506/E151,14,8.059";
	char d3[] = "S33,55,10.147/E151,13,35.229";
	
	keymanager->str2DD(d0, &dests[0][0], &dests[0][1]);
	keymanager->str2DD(d1, &dests[1][0], &dests[1][1]);
	keymanager->str2DD(d2, &dests[2][0], &dests[2][1]);
	keymanager->str2DD(d3, &dests[3][0], &dests[3][1]);
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "AutoGUI");
	AutoGUI gui(1855, 1056, &argc, argv);
	gui.run();
	return 0;
}
