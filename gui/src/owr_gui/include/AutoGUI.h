/*
	Class header for AutoGUI
*/

#ifndef AUTOGUI_H
#define AUTOGUI_H

#include "ListNode.h"
#include <list>
#include "GLUTWindow.h"

#define SCALE 40000

class AutoGUI : public GLUTWindow {
	public:
		AutoGUI(int width, int height, int *argc, char *argv[], double destPos[3][2]);
		void updateInfo(ListNode cur);
	private:
		void drawFullMap();
		void idle();
		void display();
		void keydown(unsigned char key, int x, int y);
		
		bool arrows[4];
		double dests[3][2];
		double mapCentre[2];
		vector3D currentPos;
		
		std::list<ListNode> path;
		std::list<ListNode> obstacles;
		
		// draws map for left side of the window
		void drawOverviewMap();
		// draws map for right side of the window
		void drawTrackingMap();
		
		// pointer to the ROS handler
		void *autoNode;
};

#endif // AUTOGUI_H
