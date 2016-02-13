/*
	Class header for ArmWidgetGUI
*/

#ifndef ARMWIDGETGUI_H
#define ARMWIDGETGUI_H

#include <list>
#include <ros/ros.h>
//#include <sensor_msgs/NavSatFix.h>
#include "GLUTWindow.h"

#define SCALE 40000

class ArmWidgetGUI : public GLUTWindow {
	public:
		ArmWidgetGUI(int width, int height, int *argc, char *argv[]);
	private:
		void idle();
		void display();
		void keydown(unsigned char key, int x, int y);
		void special_keydown(int keycode, int x, int y);
		
		// pointer to the ROS handler
		void *armWidgetNode;
		
		ros::Publisher tfPublisher;
};

#endif // ARMWIDGETGUI_H
