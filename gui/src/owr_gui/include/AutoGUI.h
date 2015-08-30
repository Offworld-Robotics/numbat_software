/*
	Class header for AutoGUI
*/

#ifndef AUTOGUI_H
#define AUTOGUI_H

#include "ListNode.h"
#include <list>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "GLUTWindow.h"
#include "GPSInputManager.h"

#define SCALE 40000

#define INPUT_BUFFER_SIZE 30
#define INPUT_DISABLED 0
#define INPUT_LAT 1
#define INPUT_LON 2
#define NUM_DESTS 4

class AutoGUI : public GLUTWindow {
	public:
		AutoGUI(int width, int height, int *argc, char *argv[]);
		void updateInfo(ListNode cur);
	private:
		void idle();
		void display();
		void keydown(unsigned char key, int x, int y);
		void special_keydown(int keycode, int x, int y);
		
		// destinations management
		bool haveDests;
		int destNum;
		
		GPSInputManager *keymanager;
		
		void drawFullMap(double refLat, double refLon);
		void drawGPSPos();
		void drawDividingLine();
		void drawGPSDests();
		void drawScales();
		
		void publishGPS(double lat, double lon);
		
		bool arrows[4];
		double dests[NUM_DESTS][2];
		double mapCentre[2];
		vector3D currentPos;
		
		std::list<ListNode> path;
		std::list<ListNode> obstacles;
		double scale[2];
		
		bool animPath;
		
		// draws map for left side of the window
		void drawOverviewMap();
		// draws map for right side of the window
		void drawTrackingMap();
		
		// pointer to the ROS handler
		void *autoNode;
		
		ros::Publisher gpsPublisher;
};

#endif // AUTOGUI_H
