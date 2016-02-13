#include "GLUTWindow.h"
#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <list>
#include <limits>
#include "ArmWidgetGUI.h"
#include "ArmWidgetNode.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

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
	ArmWidgetGUI gui(1855, 1056, &argc, argv);
	printf("Running!\n");
	gui.run();
	return 0;
}
