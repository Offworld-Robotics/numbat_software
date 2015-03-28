/*
	Class header for AnalysisGUI
*/
 
#ifndef ANALYSISGUI_H
#define ANALYSISGUI_H

#include "GLUTWindow.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <GL/freeglut.h>
#include <ros/ros.h>

// default window size
#define WINDOW_W 1920
#define WINDOW_H 892

#define NUM_IMAGES 3
#define PANORAMIC0 0
#define PANORAMIC1 1
#define HIGH_RES   2

#define PANO_W 640
#define PANO_H 480
#define PANO_DATA_SIZE PANO_W*PANO_H*3
#define HIRES_W 640
#define HIRES_H 480
#define HIRES_DATA_SIZE HIRES_W*HIRES_H*3

#define UP    0
#define DOWN  1
#define LEFT  2
#define RIGHT 3

class AnalysisGUI : public GLUTWindow {
	public:
		AnalysisGUI(int *argc, char **argv);
		void updateSiteInfo(double latitude, double longitude, float altitude, float pH, float ultrasonic, float humidity);
		void updateVideo(unsigned char *frame, int width, int height, int channel);
		
	private:
		// GLUT essential functions
		void display();
		void idle();
		
		// GLUT keyboard functions
		void keydown(unsigned char key, int x, int y);
		void keyup(unsigned char key, int x, int y);
		void special_keydown(int keycode, int x, int y);
		void special_keyup(int keycode, int x, int y);
		
		// saves the panoramic image, hi-res image, and site stats to the home/owr_sites folder
		bool saveState();
		
		// draw functions
		void drawButtons();
		void drawImages();
		void drawTextInfo();
		
		// pointer to the ROS node
		void *analysisNode;
		
		// site values to be displayed on the screen
		double latitude;
		double longitude;
		float altitude;
		float pH;
		float ultrasonic;
		float humidity;

		// OpenGL control related variables
		bool arrowKeys[4];
		GLuint imgTextures[NUM_IMAGES]; // 2 panoramic source images, 1 hi-res image
		
		struct stat st; // needed for stat()

		//TODO: functions to request site images
		bool requestPano;
		bool requestHires;
		bool receivedPano[2];
		unsigned char *pano0;
		unsigned char *pano1;
		
};

#endif
