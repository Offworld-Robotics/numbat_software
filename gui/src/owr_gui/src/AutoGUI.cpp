#include "GLUTWindow.h"
#include "ListNode.h"
#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <list>
#include "AutoGUI.h"
#include "AutoNode.h"

void AutoGUI::updateInfo(ListNode cur) {
	if (cur != NULL) path.push_front(cur);
}

void AutoGUI::drawFullMap() {
	glPushMatrix();
	glPointSize(5);
	
	glBegin(GL_POINTS);
	glColor3f(1, 0, 0);
	glVertex2d(dests[0][1] - mapCentre[1], dests[0][0] - mapCentre[0]);
	glColor3f(0, 0, 1);
	glVertex2d(dests[1][1] - mapCentre[1], dests[1][0] - mapCentre[0]);
	glColor3f(0, 1, 0);
	glVertex2d(dests[2][1] - mapCentre[1], dests[2][0] - mapCentre[0]);
	glEnd();
	
	// THIS ANIMATES
	// see http://www.felixgers.de/teaching/jogl/stippledLines.html
	glLineStipple(1000, 0xAAAA);
	glEnable(GL_LINE_STIPPLE);
	glBegin(GL_LINE_STRIP);
	for (int i = 2;i >= 0;i--)
		glVertex2d(dests[i][1] - mapCentre[1], dests[i][0] - mapCentre[0]);
	glEnd();
	
	glDisable(GL_LINE_STIPPLE);
	
	glPointSize(1);
	glPopMatrix();
}

void AutoGUI::idle() {
	display();
	usleep(15000);
}

void AutoGUI::drawOverviewMap() {
	glPushMatrix();
	
	glTranslated(currWinW/6.0, -currWinH/2.0, 0);
	
	glScaled(SCALE, SCALE, 0);
	
	drawFullMap();
	glPopMatrix();
}

void AutoGUI::drawTrackingMap() {
	glPushMatrix();
	
	glTranslated(currWinW - currWinW/3.0, -currWinH/2.0, 0);
	
	glScaled(1.5*SCALE, 1.5*SCALE, 0);
	
	drawFullMap();
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
	
	glutSwapBuffers();
}

void AutoGUI::keydown(unsigned char key, int x, int y) {
	switch (key) {
		case 27:
			exit(0);
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
	AutoGUI gui(1855, 1056, &argc, argv, dest);
	gui.run();
	return 0;
}
