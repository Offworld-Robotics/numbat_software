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
		
		/*
		base_link
		arm_base
		arm_top_actuator_base
		arm_top_actuator_shaft
		arm_top_plates
		arm_top_plate_base_link
		arm_bottom_plates
		arm_bottom_actuator_base
		arm_bottom_actuator_shaft
		arm_lower_bottom_plates
		arm_sync_plate_lower
		claw_base
		claw_wrist
		claw_right
		claw_left
		claw_upper_link
		arm_sync_plates
		arm_lower_top_plates
		*/
		
		armTFListener.lookupTransform("/arm_base", "/arm_top_actuator_base", ros::Time(0), armTFs[0]);
		armTFListener.lookupTransform("/arm_base", "/arm_top_actuator_shaft", ros::Time(0), armTFs[1]);
		armTFListener.lookupTransform("/arm_base", "/arm_top_plates", ros::Time(0), armTFs[2]);
		armTFListener.lookupTransform("/arm_base", "/arm_top_plate_base_link", ros::Time(0), armTFs[3]);
		armTFListener.lookupTransform("/arm_base", "/arm_bottom_plates", ros::Time(0), armTFs[4]);
		armTFListener.lookupTransform("/arm_base", "/arm_bottom_actuator_base", ros::Time(0), armTFs[5]);
		armTFListener.lookupTransform("/arm_base", "/arm_bottom_actuator_shaft", ros::Time(0), armTFs[6]);
		armTFListener.lookupTransform("/arm_base", "/arm_lower_bottom_plates", ros::Time(0), armTFs[7]);
		armTFListener.lookupTransform("/arm_base", "/arm_sync_plate_lower", ros::Time(0), armTFs[8]);
		armTFListener.lookupTransform("/arm_base", "/arm_sync_plates", ros::Time(0), armTFs[9]);
		armTFListener.lookupTransform("/arm_base", "/arm_lower_top_plates", ros::Time(0), armTFs[10]);
		
		for (int i = 0;i < NUM_ARM_JOINTS;i++) {
			cout << "tfs[" << i << "]: "
			<< armTFs[i].stamp_ << ": "
			<< armTFs[i].getOrigin().x() << ","
			<< armTFs[i].getOrigin().y() << ","
			<< armTFs[i].getOrigin().z() << endl << endl;
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
	cout << "scale = " << scale << endl;
	display();
	usleep(15000);
}

void ArmWidgetGUI::display() {
	glClear(GL_COLOR_BUFFER_BIT);
	
	glPushMatrix();
	glTranslated(currWinW/2.0, -currWinH/2.0, 0);
	glColor3f(1,0,0);
	
	glScaled(scale, scale, 1);
	glBegin(GL_LINES);
	for(int i = 0;i < NUM_ARM_JOINTS;i++) {
		glVertex3d(0,0,0);
		glVertex3d(armTFs[i].getOrigin().x(), armTFs[i].getOrigin().z(), 0);
	}
	glEnd();
	
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
