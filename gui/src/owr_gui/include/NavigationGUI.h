/*
	Class header for NavigationGUI
*/
 
#ifndef NAVIGATIONGUI_H
#define NAVIGATIONGUI_H

#include "GLUTWindow.h"
#include "ListNode.h"
#include <GL/freeglut.h>
#include <ros/ros.h>
#include "owr_messages/stream.h"
#include <list>
#include "Video_Feed_Frame.hpp"

#define PI 3.1415926535897932384626433832795

// default window size
#define WINDOW_W 1855
#define WINDOW_H 917

#define TOTAL_FEEDS 8 

#define FEED_ACTIVE_BUTTON_R 0
#define FEED_ACTIVE_BUTTON_G 153
#define FEED_ACTIVE_BUTTON_B 0

#define FEED_INACTIVE_BUTTON_R 235
#define FEED_INACTIVE_BUTTON_G 133
#define FEED_INACTIVE_BUTTON_B 51

#define FEED_OFFLINE_BUTTON_R 230
#define FEED_OFFLINE_BUTTON_G 0
#define FEED_OFFLINE_BUTTON_B 0

#define DEFAULT_SCALE 30000
#define ARTIFICIAL_HORIZON_SKY_HEIGHT 50
#define ARTIFICIAL_HORIZON_SKY_HALF_WIDTH 70
#define ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH 40

#define ULTRASONIC_MAX 10.0

#define NUM_ARROWKEYS 4
#define UP    0
#define DOWN  1
#define LEFT  2
#define RIGHT 3

// possible feed status
#define FEED_OFFLINE  0 // Disconnected
#define FEED_INACTIVE 1 // Connected but not streaming
#define FEED_ACTIVE   2 // Connected and streaming

#define ALPHA 1.0 // transparency factor
#define TEXTBOX_ALPHA 0.001


class NavigationGUI : public GLUTWindow {

	public:
		NavigationGUI(int width, int height, int *argc, char **argv);
		void updateInfo(float battery, float signal, float ultrasonic, ListNode current, vector3D target, float lidar);
		void updateVideo(unsigned char *frame, int width, int height);
		void updateFeedsStatus(unsigned char *feeds, int numOnline);
		
	private:
		// GLUT essential functions
		void idle();
		void display();
		void reshape(int w, int h);

		// GLUT keyboard functions
		void keydown(unsigned char key, int x, int y);
		void keyup(unsigned char key, int x, int y);
		void special_keydown(int keycode, int x, int y);
		void special_keyup(int keycode, int x, int y);
		
		//draws button
		void drawButton(int feed);

		// function to insert a given co-ordinate to the front of the path list
		void GPSAddPos(double x, double y);
		// function to insert a random co-ordinate to the front of the path list
		void GPSAddRandPos();
		// function to generate a target co-ordinate
		void generateTarget();
		// function to print the path
		void printGPSPath();
		
		// draw functions
		void drawVideo();
		void drawFeedStatus();
		void drawGPS();
		void drawTilt();
		void drawBattery();
		void drawSignal();
		void drawUltrasonic();
		
		
		void drawLidarTilt();
		
		// pointer to the ROS handler
		void *navigationNode;
		
		// status values
		float battery;
		float signal;
		float tiltX; // tilt of left-right in degrees
		float tiltY; // tilt of forward-back in degrees
		float ultrasonic;
		float lidarTilt;
		vector3D currentPos;
		
		double pathRotation; // angle to rotate GPS path when drawing
		double prevRotation;

		unsigned char feedStatus[TOTAL_FEEDS]; // status for each feed
		int currFeed;
		int onlineFeeds;

		// GPS related variables
		std::list<ListNode> GPSList; // path history (front is current point, back is origin point)
		vector3D target;

		// OpenGL control related variables
		bool arrowKeys[4];
		GLuint feedTexture;
		double scale;
		double cursorSpin;
		bool displayOverlay;
                bool displayTilt;
		//ros stuff
		ros::Publisher streamPub;
		void toggleStream(int feed);
		
		Video_Feed_Frame *videoScreen;
};


#endif
