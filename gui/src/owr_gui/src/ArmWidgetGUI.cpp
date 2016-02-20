#include "GLUTWindow.h"
#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "ArmWidgetGUI.h"

using namespace std;

void ArmWidgetGUI::refreshTransforms() {
	try {
		//listener.lookupTransform("/from_frame", "/to_frame", ros::Time(0), transform);
		
		//top_actuator_base
		//	top_actuator_shaft
		//		sync_main
		//structural_main
		//	sync_plate
		//		sync_minor
		//			end_plate
		//	rocker
		//	bottom_actuator_base
		//		bottom_actuator_shaft
		
		tfListener.lookupTransform("/base_link", "/top_actuator_base", ros::Time(0), tfs[0]);
		tfListener.lookupTransform("/base_link", "/top_actuator_shaft", ros::Time(0), tfs[1]);
		tfListener.lookupTransform("/base_link", "/sync_main", ros::Time(0), tfs[2]);
		tfListener.lookupTransform("/base_link", "/structural_main", ros::Time(0), tfs[3]);
		tfListener.lookupTransform("/base_link", "/sync_plate", ros::Time(0), tfs[4]);
		tfListener.lookupTransform("/base_link", "/sync_minor", ros::Time(0), tfs[5]);
		tfListener.lookupTransform("/base_link", "/end_plate", ros::Time(0), tfs[6]);
		tfListener.lookupTransform("/base_link", "/rocker", ros::Time(0), tfs[7]);
		tfListener.lookupTransform("/base_link", "/bottom_actuator_base", ros::Time(0), tfs[8]);
		tfListener.lookupTransform("/base_link", "/bottom_actuator_shaft", ros::Time(0), tfs[9]);
		
		for (int i = 0;i < 10;i++) {
			cout << "tfs[" << i << "]: "
			<< tfs[i].stamp_ << ": "
			<< tfs[i].getOrigin().x() << ","
			<< tfs[i].getOrigin().y() << ","
			<< tfs[i].getOrigin().z() << endl << endl;
		}
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}
}

void ArmWidgetGUI::idle() {
	if(arrow_keys[0]) {
		scale += 50;
	}
	if(arrow_keys[1]) {
		scale -= 50;
	}
	if(scale < 1) {
		scale  = 1;
	}
	//cout << scale << endl;
	refreshTransforms();
	display();
	usleep(15000);
}

void ArmWidgetGUI::display() {
	glClear(GL_COLOR_BUFFER_BIT);
	
	glPushMatrix();
	glTranslated(currWinW/2.0, -currWinH/2.0, 0);
	glColor3f(1,0,0);
	
	/*glBegin(GL_TRIANGLES);
	glVertex3d(0,0,0);
	glVertex3d(100,100,0);
	glVertex3d(-100,100,0);
	glEnd();*/
	
	glPointSize(100);
	glScaled(scale, scale, 0);
	glBegin(GL_LINES);
	for(int i = 0;i < 10;i++) {
		glVertex3d(0,0,0);
		glVertex3d(tfs[i].getOrigin().x(), tfs[i].getOrigin().z(), 0);
	}
	glEnd();
	glPointSize(1);
	
	glPopMatrix();
	glutSwapBuffers();
}

void ArmWidgetGUI::special_keydown(int key, int x, int y) {
	switch(key) {
		case GLUT_KEY_UP:
			arrow_keys[0] = true;
			break;
		case GLUT_KEY_DOWN:
			arrow_keys[1] = true;
			break;
		default:
			break;
	}
}

void ArmWidgetGUI::special_keyup(int key, int x, int y) {
	switch(key) {
		case GLUT_KEY_UP:
			arrow_keys[0] = false;
			break;
		case GLUT_KEY_DOWN:
			arrow_keys[1] = false;
			break;
		default:
			break;
	}
}

void ArmWidgetGUI::keydown(unsigned char key, int x, int y) {
	switch (key) {
		case 27:
			exit(0);
			break;
	}
}

ArmWidgetGUI::ArmWidgetGUI(int width, int height, int *argc, char *argv[]) : GLUTWindow(width, height, argc, argv, "ArmWidgetGUI") {
	
	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	glutSpecialFunc(glut_special_keydown);
	glutSpecialUpFunc(glut_special_keyup);
	scale = 100;
	for(int i = 0;i < 4;i++) {
		arrow_keys[i] = false;
	}
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "ArmWidgetGUI");
	ArmWidgetGUI gui(1855, 1056, &argc, argv);
	gui.run();
	return 0;
}
