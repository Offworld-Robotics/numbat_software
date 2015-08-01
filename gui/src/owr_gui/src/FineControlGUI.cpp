#include "GLUTWindow.h"
#include "Button.h"
#include "Video_Feed_Frame.hpp"
#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <ros/ros.h>
#include "FineControlGUI.h"
#include "FineControlNode.h"
#include "ListNode.h"

void FineControlGUI::reshape(int w, int h) {
	GLUTWindow::reshape(w, h);
	for(std::vector<Video_Feed_Frame*>::iterator i = videoFeeds.begin(); i != videoFeeds.end(); ++i)
		(*i)->setNewWindowSize(w, h);
}

void FineControlGUI::idle() {
	ros::spinOnce();
	display();
	usleep(15000);
}

void FineControlGUI::updateVideo(unsigned char *frame, int width, int height, int cam) {
	if(cam == LScreenCam)
		videoScreens[0]->setNewStreamFrame(frame, width, height);
	else if(cam == RScreenCam)
		videoScreens[1]->setNewStreamFrame(frame, width, height);
}

void FineControlGUI::updateFeedsStatus(unsigned char *feeds, int numOnline) {
	memcpy(LFeedStatus, feeds, TOTAL_FEEDS*sizeof(unsigned char));
	memcpy(RFeedStatus, feeds, TOTAL_FEEDS*sizeof(unsigned char));
	if(LScreenCam >= 0 && RScreenCam >= 0 && LScreenCam != RScreenCam) {
		LFeedStatus[RScreenCam] = FEED_INACTIVE;
		RFeedStatus[LScreenCam] = FEED_INACTIVE;
	}
	onlineFeeds = numOnline;
	//ROS_INFO("updating online feeds: [%d,%d,%d,%d]", feeds[0], feeds[1], feeds[2], feeds[3]);
}

void FineControlGUI::updateInfo(float volt, float ultrason, float ph, float humid, ArmState *arm, float head, float tx, float ty, ListNode cur) {
	voltage = volt;
	ultrasonic = ultrason;
	pH = ph;
	humidity = humid;
	if(arm != NULL) armState = *arm;
	heading = head;
	tiltX = tx;
	tiltY = ty;
	if(cur != NULL) currentPos = *cur;
	
	//ROS_INFO("Updated info");
}

void FineControlGUI::display() {
	glClear(GL_COLOR_BUFFER_BIT);
	
	/*for(std::vector<Button*>::iterator i = buttons.begin();i != buttons.end();++i) {
		(*i)->draw();
	}*/

        //Draw Video Feeds to Screen
        for(std::vector<Video_Feed_Frame*>::iterator feed = videoScreens.begin(); feed != videoScreens.end(); ++feed)
		(*feed)->draw();
	
	drawFeedStatus();
	displayInfo();
	glutSwapBuffers();
}

void FineControlGUI::keydown(unsigned char key, int x, int y) {
	if (key == 27) {
		exit(0);
	} else if (key >= '0' && key <= '3') {
		toggleStream(key - '0', true);
	} else if (key >= '4' && key <= '7') {
		toggleStream(key - '4', false);
	}
}

void FineControlGUI::mouse(int button, int state, int x, int y) {
	/*if (state == GLUT_UP) {
		for(std::vector<Button*>::iterator i = buttons.begin();i != buttons.end();++i) {
			(*i)->unclick();
		}
	} else {
		for(std::vector<Button*>::iterator i = buttons.begin();i != buttons.end();++i) {
			if ((*i)->isPointInBounds(x, -y))
				(*i)->click();
		}
	}*/
}

// draw the buttons
void FineControlGUI::drawLFeedBox(int feed) {
	if (LFeedStatus[feed] == FEED_ACTIVE)
		glColor3ub(FEED_ACTIVE_BUTTON_R, FEED_ACTIVE_BUTTON_G, FEED_ACTIVE_BUTTON_B);
	else if (LFeedStatus[feed] == FEED_INACTIVE)
		glColor3ub(FEED_INACTIVE_BUTTON_R, FEED_INACTIVE_BUTTON_G, FEED_INACTIVE_BUTTON_B);
	else
		glColor3ub(FEED_OFFLINE_BUTTON_R, FEED_OFFLINE_BUTTON_G, FEED_OFFLINE_BUTTON_B);

	glRecti(-30, -25, 30, 25);
	glColor3f(0, 0, 1);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, feed + '0');
}

void FineControlGUI::drawRFeedBox(int feed) {
	if (RFeedStatus[feed] == FEED_ACTIVE)
		glColor3ub(FEED_ACTIVE_BUTTON_R, FEED_ACTIVE_BUTTON_G, FEED_ACTIVE_BUTTON_B);
	else if (RFeedStatus[feed] == FEED_INACTIVE)
		glColor3ub(FEED_INACTIVE_BUTTON_R, FEED_INACTIVE_BUTTON_G, FEED_INACTIVE_BUTTON_B);
	else
		glColor3ub(FEED_OFFLINE_BUTTON_R, FEED_OFFLINE_BUTTON_G, FEED_OFFLINE_BUTTON_B);

	glRecti(-30, -25, 30, 25);
	glColor3f(0, 0, 1);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, feed + '0');
}

// draws feeds boxes for each of the feeds
void FineControlGUI::drawFeedStatus() {
	glPushMatrix();
	glTranslated(currWinW*0.05, -currWinH*0.7, 0);
	for(int i = 0;i < TOTAL_FEEDS;i++) {
		drawLFeedBox(i);
		glTranslated(75, 0, 0);
	}
	glPopMatrix();
	
	glPushMatrix();
	glTranslated(currWinW*0.55, -currWinH*0.7, 0);
	for(int i = 0;i < TOTAL_FEEDS;i++) {
		drawRFeedBox(i);
		glTranslated(75, 0, 0);
	}
	glPopMatrix();
}

FineControlGUI::FineControlGUI(int width, int height, int *argc, char *argv[]) : GLUTWindow(width, height, argc, argv, "Fine Control") {
	ros::NodeHandle node;
	streamPub = node.advertise<owr_messages::stream>("owr/control/activateFeeds", 1000);
	fineControlNode = new FineControlNode(this);
	
	glClearColor(1, 1, 1, 1);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	glutMouseFunc(glut_mouse);
	
	// push the two feeds
        videoFeeds.push_back(new Video_Feed_Frame(width, height, 0.25, -0.375, 0.5, 0.75));
        videoFeeds.push_back(new Video_Feed_Frame(width, height, 0.75, -0.375, 0.5, 0.75));

	for (int i = 0;i < 4;i++)
		arrows[i] = false;
	
	for (int i = 0;i < TOTAL_FEEDS;i++) {
		LFeedStatus[i] = FEED_OFFLINE;
		RFeedStatus[i] = FEED_OFFLINE;
		sendStreamMsg(i, false);
	}
	onlineFeeds = 0;
	
	// setting both screens to unmapped to any cam
	LScreenCam = RScreenCam = -1;
	
	voltage = 0;
	memset(&armState, 0, sizeof(armState));
	pH = humidity = 0;
	memset(&currentPos, 0, sizeof(currentPos));
	heading = tiltX = tiltY = 0;
	ultrasonic = 0;
	
	/*char txt[2] = {'+', '\0'};
	buttons.push_back(new Button(currWinW/2 - 100, -currWinH/2, 50, 50, 0, 0.5, 0.5, txt));
	txt[0] = '-';
	buttons.push_back(new Button(currWinW/2 + 100, -currWinH/2, 50, 50, 0, 0.5, 0.5, txt));*/
}

void FineControlGUI::sendStreamMsg(int stream, bool on) {
	owr_messages::stream s;
	s.stream = stream;
	s.on = on;
	streamPub.publish(s);
	ros::spinOnce();
}

// toggles between available streams for the leftside screen
void FineControlGUI::toggleStream(int feed, bool left) {
	ROS_INFO("FineControl: Switching feed %d", feed);
	
	if (LFeedStatus[feed] == FEED_OFFLINE) {
		ROS_INFO("FineControl: Error: Feed %d is offline", feed);
		return;
	} else {
		if(left) LScreenCam = feed;
		else RScreenCam = feed;
		sendStreamMsg(feed, true);
		/*for(int i = 0;i < TOTAL_FEEDS;i++) {
			if(i != LScreenCam && i != RScreenCam) {
				sendStreamMsg(i, false);
			}
		}*/
	}
	//ros::spinOnce();
}

void FineControlGUI::displayInfo() {
	char txt[50] = {0};
	glColor3f(0, 0, 0);
	glPushMatrix();
	glTranslated(0, -5*currWinH/6, 0);
	
	// left info column: voltage, heading, and tilts
	glPushMatrix();
	
	sprintf(txt, "Voltage: %.2f", voltage);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glTranslated(0, -30, 0);
	sprintf(txt, "Heading: %.2f", heading);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glTranslated(0, -30, 0);
	sprintf(txt, "TiltX: %.2f", tiltX);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glTranslated(0, -30, 0);
	sprintf(txt, "TiltY: %.2f", tiltY);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glPopMatrix();
	
	// middle info column: arm position
	glTranslated(2*currWinW/5, 0, 0);
	glPushMatrix();
	
	sprintf(txt, "Top Actuator: %d", armState.topActPos);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glTranslated(0, -30, 0);
	sprintf(txt, "Bot Actuator: %d", armState.botActPos);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glTranslated(0, -30, 0);
	sprintf(txt, "Arm Rotation: %.2f", armState.rotation);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glPopMatrix();
	
	// right info column: environmentals, GPS
	glTranslated(2*currWinW/5, 0, 0);
	glPushMatrix();
	
	sprintf(txt, "pH: %.2f", pH);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glTranslated(0, -30, 0);
	sprintf(txt, "Humidity: %.2f", humidity);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glTranslated(0, -30, 0);
	sprintf(txt, "Lat: %.2f", currentPos.lat);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glTranslated(0, -30, 0);
	sprintf(txt, "Lon: %.2f", currentPos.lon);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glTranslated(0, -30, 0);
	sprintf(txt, "Altitude: %.2f", currentPos.alt);
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	
	glPopMatrix();
	
	glPopMatrix();
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "FineControlGUI");
	FineControlGUI gui(WINDOW_W, WINDOW_H, &argc, argv);
	gui.run();
	return 0;
}
