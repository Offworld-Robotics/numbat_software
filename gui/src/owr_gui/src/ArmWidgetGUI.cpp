#include "GLUTWindow.h"
#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <list>
#include <limits>
#include <iostream>
#include "ArmWidgetGUI.h"
#include "ArmWidgetNode.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_listener.h>

using namespace std;

void ArmWidgetGUI::idle() {
	display();
	usleep(15000);
}

void ArmWidgetGUI::display() {
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
}

void ArmWidgetGUI::special_keydown(int keycode, int x, int y) {
	
}

void ArmWidgetGUI::keydown(unsigned char key, int x, int y) {
	switch (key) {
		case 27:
			exit(0);
			break;
	}
}

ArmWidgetGUI::ArmWidgetGUI(int width, int height, int *argc, char *argv[]) : GLUTWindow(width, height, argc, argv, "ArmWidgetGUI") {
	armWidgetNode = new ArmWidgetNode(this);
	
	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	glutSpecialFunc(glut_special_keydown);
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "ArmWidgetGUI");
	//ArmWidgetGUI gui(1855, 1056, &argc, argv);
	//gui.run();
	
	ros::NodeHandle node;
	tf::TransformListener listener;

	ros::Rate rate(10.0);
	while (node.ok()){
		tf::StampedTransform transform;
		try {
			//listener.lookupTransform("/from_frame", "/to_frame", ros::Time(0), transform);
			listener.lookupTransform("/base_link", "/top_actuator_shaft", ros::Time(0), transform);
			cout << "New tf\n";
			cout << transform.getOrigin().x() << endl;
			cout << transform.getOrigin().y() << endl;
			cout << transform.getOrigin().z() << endl << endl;
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		rate.sleep();
	}
	return 0;
}
