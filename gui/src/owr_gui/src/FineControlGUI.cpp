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

void FineControlGUI::idle() {
	ros::spinOnce();
	display();
	usleep(15000);
}

void FineControlGUI::updateVideo0(unsigned char *frame, int width, int height) {
	videoFeeds[0]->setNewStreamFrame(frame, width, height);
}

void FineControlGUI::updateVideo1(unsigned char *frame, int width, int height) {
	videoFeeds[1]->setNewStreamFrame(frame, width, height);
}

void FineControlGUI::updateFeedsStatus(unsigned char *feeds, int numOnline) {
	memcpy(feedStatus, feeds, TOTAL_FEEDS*sizeof(unsigned char));
	onlineFeeds = numOnline;
	//ROS_INFO("updating online feeds: [%d,%d,%d,%d]", feeds[0], feeds[1], feeds[2], feeds[3]);
}

void FineControlGUI::display() {
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(0,0,0);
	
	// draw dividing lines
	glPushMatrix();
	glTranslated(currWinW/2, -currWinH/2, 0);
	glBegin(GL_LINES);
	glVertex2d(0, currWinH);
	glVertex2d(0, -currWinH/4);
	glVertex2d(-currWinW, -currWinH/4);
	glVertex2d(currWinW, -currWinH/4);
	glEnd();
	glPopMatrix();
	
	/*for(std::vector<Button*>::iterator i = buttons.begin();i != buttons.end();++i) {
		(*i)->draw();
	}*/
	glPushMatrix();
	glTranslated(currWinW/2, -7*currWinH/8, 0);
	glColor3f(0,0,0);
	char txt[] = "GUI info placeholder";
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glPopMatrix();

        //Draw Video Feeds to Screen
        for(std::vector<Video_Feed_Frame*>::iterator feed = videoFeeds.begin(); feed != videoFeeds.end(); ++feed)
		(*feed)->draw();

	glutSwapBuffers();
}

void FineControlGUI::keydown(unsigned char key, int x, int y) {
	if (key == 27) {
		exit(0);
	} else if (key >= '0' && key <= '3') {
		toggleStream(key - '0');
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

FineControlGUI::FineControlGUI(int width, int height, int *argc, char *argv[]) : GLUTWindow(width, height, argc, argv, "Fine Control") {
	streamPub = node.advertise<owr_messages::stream>("owr/control/activateFeeds", 1000);
	fineControlNode = new FineControlNode(this);
	
	glClearColor(1, 1, 1, 1);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	glutMouseFunc(glut_mouse);
	
	// push the two feeds
        videoFeeds.push_back(new Video_Feed_Frame(width*1/4, -height*3/8, width/2, height*3/4));
        videoFeeds.push_back(new Video_Feed_Frame(width*3/4, -height*3/8, width/2, height*3/4));

        // setup Zoom Buttons
	for (int i = 0;i < 4;i++)
		arrows[i] = false;
	
	for (int i = 0;i < TOTAL_FEEDS;i++)
		feedStatus[i] = FEED_OFFLINE;
	onlineFeeds = 0;
	
	//start on stream 0
	usleep(150000);
	toggleStream(0);
	
	/*char txt[2] = {'+', '\0'};
	buttons.push_back(new Button(currWinW/2 - 100, -currWinH/2, 50, 50, 0, 0.5, 0.5, txt));
	txt[0] = '-';
	buttons.push_back(new Button(currWinW/2 + 100, -currWinH/2, 50, 50, 0, 0.5, 0.5, txt));*/
}

// toggles between available streams
void FineControlGUI::toggleStream(int feed) {
	printf("Switching feed %d\n", feed);
	
	if (feedStatus[feed] == FEED_OFFLINE) {
		printf("Error: feed %d is offline\n", feed);
		return;
	} else if (feedStatus[feed] == FEED_INACTIVE) {
		feedStatus[feed] = FEED_ACTIVE;
		for(int i = 0;i < TOTAL_FEEDS;i++) {
			if (i != feed && feedStatus[i] == FEED_ACTIVE) {
				owr_messages::stream off;
				feedStatus[i] = FEED_INACTIVE;
				off.stream = i;
				off.on = false;
				streamPub.publish(off);
			}
		}
	} else {
		feedStatus[feed] = FEED_INACTIVE;
	}
	
	owr_messages::stream msg;
	msg.stream = feed;
	if (feedStatus[feed] == FEED_ACTIVE) {
		msg.on = true;
	} else {
		msg.on = false;
	}
	streamPub.publish(msg);
	
	ros::spinOnce();
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "FineControlGUI");
	FineControlGUI gui(WINDOW_W, WINDOW_H, &argc, argv);
	gui.run();
	return 0;
}
