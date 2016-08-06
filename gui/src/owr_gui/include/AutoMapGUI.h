/*
	Class header for AutoMapGUI
*/
 
#ifndef AutoMapGUI_H
#define AutoMapGUI_H

#include "GLUTWindow.h"
#include "Button.h"
#include <GL/freeglut.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

#define PI 3.1415926535897932384626433832795

// default window size
#define WINDOW_W 1855
#define WINDOW_H 917

#define BUFFER_SIZE 100

class AutoMapGUI : public GLUTWindow {
	public:
		AutoMapGUI(int width, int height, int *argc, char **argv);
		void updateGrid(char *grid, int width, int height, std_msgs::Header header);
		
	private:
		// GLUT essential functions
		void idle();
		void display();
		void reshape(int w, int h);

		// GLUT keyboard functions
		void keydown(unsigned char key, int x, int y);
		void keyup(unsigned char key, int x, int y);
		void mouse(int button, int state, int x, int y);
		
		// pointer to the ROS handler
		void *autoMapNode;
		
		GLuint gridTexture;
		
		unsigned char *gridData;
		int gridCols;
		int gridRows;
		static std_msgs::Header lastGridHeader;
		
		bool showHelp;
		
		char textBuffer[BUFFER_SIZE];
		int bufferIndex;
		void extractGoalCoords();
		bool validGoalCoords;
		
		Button startButton;
		Button stopButton;
		Button goalButton;

		static void sendStartMessage();
		static void sendStopMessage();
		static void sendGoalMessage();
		
		static int goalGridRow;
		static int goalGridCol;
		
};


#endif
