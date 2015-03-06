/*
	Class header for NavigationGUI
*/
 
#ifndef NAVIGATIONGUI_H
#define NAVIGATIONGUI_H

#include "GLUTWindow.h"
#include "ListNode.h"
#include <GL/freeglut.h>
#include <ros/ros.h>
#include "owr_camera_control/stream.h"
#include <list>

#define PI 3.1415926535897932384626433832795

// default window size
#define WINDOW_W 1920
#define WINDOW_H 892

#define VID_FEED_ACTIVE_BUTTON_RED 0
#define VID_FEED_ACTIVE_BUTTON_GREEN 153
#define VID_FEED_ACTIVE_BUTTON_BLUE 0

#define VID_FEED_ACTIVE_NOT_LIVE_BUTTON_RED 235
#define VID_FEED_ACTIVE_NOT_LIVE_BUTTON_GREEN 133
#define VID_FEED_ACTIVE_NOT_LIVE_BUTTON_BLUE 51

#define VID_FEED_INACTIVE_BUTTON_RED 230
#define VID_FEED_INACTIVE_BUTTON_GREEN 0
#define VID_FEED_INACTIVE_BUTTON_BLUE 0

#define DEFAULT_SCALE 30000
#define ARTIFICIAL_HORIZON_SKY_HEIGHT 50
#define ARTIFICIAL_HORIZON_SKY_HALF_WIDTH 70
#define ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH 40

#define VIDEO_W 1280
#define VIDEO_H 720

#define UP    0
#define DOWN  1
#define LEFT  2
#define RIGHT 3

// feed related
#define MAX_FEEDS 1
#define NO_FEEDS -1
#define ALL_FEEDS 0

#define ALPHA 0.3 // transparency factor

class NavigationGUI : public GLUTWindow {
    
    public:
        NavigationGUI(int *argc, char **argv);
        void updateConstants(float battery, float signal, float ultrasonic, ListNode current, vector2D target, unsigned char *frame);
        void run();
        
    private:
        // GLUT essential functions
        void idle();
        void display();

        // GLUT keyboard functions
		void keydown(unsigned char key, int x, int y);
		void keyup(unsigned char key, int x, int y);
		void special_keydown(int keycode, int x, int y);
		void special_keyup(int keycode, int x, int y);
		
        //draws button
        void drawButton(float, float, float, bool, char);

        // function to insert a given co-ordinate to the front of the path list
        void GPSAddPos(double x, double y);
        // function to insert a random co-ordinate to the front of the path list
        void GPSAddRandPos();
        // function to generate a target co-ordinate
        void generateTarget();
        // function to print the path
        void printGPSPath();
        
        // draw functions
        void drawBackground();
        void drawFeeds();
        void drawGPS();
        void drawTilt();
        void drawBattery();
        void drawSignal();
        void drawUltrasonic();
        
        // pointer to the ROS handler
        void *navigationNode;
        
        // status values
        float battery;
        float signal;
        float tiltX; // tilt of left-right in degrees
        float tiltY; // tilt of forward-back in degrees
        float ultrasonic;
        double longitude;
        double latitude;
        double prevAngle;

        // GPS related variables
        std::list<ListNode> GPSList; // path history (front is current point, back is origin point)
        vector2D target;

        // OpenGL control related variables
        bool arrowKeys[4];
        GLuint feedTextures[MAX_FEEDS];
        int feedDisplayStatus;
        double scale;

        //ros stuff
        ros::Publisher streamPub;
        void toggleStream(int stream, bool active);
};


#endif
