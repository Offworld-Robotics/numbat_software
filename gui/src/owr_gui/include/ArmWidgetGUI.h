/*
	Class header for ArmWidgetGUI
*/

#ifndef ARMWIDGETGUI_H
#define ARMWIDGETGUI_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "GLUTWindow.h"

#define NUM_ARM_JOINTS 11

class ArmWidgetGUI : public GLUTWindow {
	public:
		ArmWidgetGUI(int width, int height, int *argc, char *argv[]);
	private:
		void idle();
		void display();
		void keydown(unsigned char key, int x, int y);
		void special_keydown(int key, int x, int y);
		void special_keyup(int key, int x, int y);
		
		void refreshTransforms();
		
		tf::TransformListener armTFListener;
		tf::StampedTransform armTFs[NUM_ARM_JOINTS];
		
		double scale;
		bool arrow_keys[4];
};

#endif // ARMWIDGETGUI_H
