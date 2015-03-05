/*
 * Class to control the ANALYSIS GUI
 *
 */
 
#ifndef SITE_GUI_H
#define SITE_GUI_H

#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <GL/glut.h>
#include "site.h"
#include <ros/ros.h>

#define PI 3.1415926535897932384626433832795

// default window size
#define WINDOW_W 1920
#define WINDOW_H 892

#define VIDEO_W 640
#define VIDEO_H 480

#define PANO_W 640
#define PANO_H 480
#define PANO_DATA_SIZE PANO_W*PANO_H*3
#define HIRES_W 640
#define HIRES_H 480
#define HIRES_DATA_SIZE HIRES_W*HIRES_H*3

#define UP	0
#define DOWN  1
#define LEFT  2
#define RIGHT 3

class SiteGui {

	public:
		SiteGui();
		void updateSiteConstants(double latitude, double longitude, float altitude, float pH, float ultrasonic, float humidity, unsigned char *frame);
		
		// glut wrapper functions because it dosen't like c++ :(
		static SiteGui *instance;
		static void createInstance(SiteGui gui);
		static void glut_reshape(int w, int h);
		static void glut_idle();
		static void glut_display();
		static void glut_keydown(unsigned char key, int x, int y);
		//static void glut_special_keydown(int keycode, int x, int y);
		//static void glut_special_keyup(int keycode, int x, int y);
		
		void init();
		
	private:
		// OpenGL essential functions
		void reshape(int w, int h);
		void idle();
		void display();

		// OpenGL keyboard functions (mainly for debugging)
		void keydown(unsigned char key, int x, int y);
		//void keyup(unsigned char key, int x, int y);
		//void special_keydown(int keycode, int x, int y);
		//void special_keyup(int keycode, int x, int y);

		// function to display some text
		void drawText(char *text, int x, int y);
		
		void loadTextures();
		void fillBMPHeader(unsigned char *data, int width, int height);
		bool saveState();
		
		// draw functions
		void drawButtons();
		void drawImages();
		void drawTextInfo();
		
		void *analysisGui;
		
		// site values
		double latitude;
		double longitude;
		float altitude;
		float pH;
		float ultrasonic;
		float humidity;

		// OpenGL control related variables
		int currentWindowH;
		int currentWindowW;
		int frameCounter;
		bool arrowKeys[4];
		GLuint textures[2]; // 1 panoramic image, 1 hi-res image
		
		struct stat st; // for stat()

		//TODO: functions to request site images
};

#endif
