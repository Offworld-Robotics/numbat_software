/*
	Class header for FineControlGUI
*/
 
#ifndef FINECONTROLGUI_H
#define FINECONTROLGUI_H

#include "Button.h"
#include "GLUTWindow.h"
#include "ListNode.h"
#include "ArmState.h"
#include <vector>
#include <ros/ros.h>
#include "owr_messages/stream.h"
#include "Video_Feed_Frame.hpp"

// default window size
#define WINDOW_W 1855
#define WINDOW_H 917

#define FEED_ACTIVE_BUTTON_R 0
#define FEED_ACTIVE_BUTTON_G 153
#define FEED_ACTIVE_BUTTON_B 0

#define FEED_INACTIVE_BUTTON_R 235
#define FEED_INACTIVE_BUTTON_G 133
#define FEED_INACTIVE_BUTTON_B 51

#define FEED_OFFLINE_BUTTON_R 230
#define FEED_OFFLINE_BUTTON_G 0
#define FEED_OFFLINE_BUTTON_B 0

#define TOTAL_FEEDS 4

// possible feed status
#define FEED_OFFLINE  0 // Disconnected
#define FEED_INACTIVE 1 // Connected but not streaming
#define FEED_ACTIVE   2 // Connected and streaming

class FineControlGUI : public GLUTWindow {
	public:
		FineControlGUI(int width, int height, int *argc, char *argv[]);
		void updateInfo(float voltage, float ultrasonic, float pH, float humidity, ArmState *armState, float heading, float tiltX, float tiltY, ListNode cur);
		void updateVideo(unsigned char *frame, int width, int height, int camera);
		void updateFeedsStatus(unsigned char *feeds, int numOnline);
	
	private:
		void reshape(int w, int h);
		void idle();
		void display();
		void keydown(unsigned char key, int x, int y);
		void mouse(int button, int state, int x, int y);
		
		void drawLFeedBox(int feed);
		void drawRFeedBox(int feed);
		void drawFeedStatus();
		void displayInfo();
		
		float voltage;
		ArmState armState;
		float pH;
		float humidity;
		vector3D currentPos;
		float heading;
		float tiltX;
		float tiltY;
		float ultrasonic;
		
		// pointer to the ROS handler
		void *fineControlNode;
		
		//ros stuff
		ros::Publisher streamPub;
		void toggleStream(int feed, bool left);
		void sendStreamMsg(int stream, bool on);
		
		bool arrows[4];
		//std::vector<Button*> buttons;
		unsigned char FeedStatus[TOTAL_FEEDS]; // status for each feed
		int onlineFeeds;
                std::vector<Video_Feed_Frame*> videoScreens;
                int LScreenCam, RScreenCam;
};

#endif // FINECONTROLGUI_H
