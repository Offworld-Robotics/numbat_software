/*
 * Class to control the OWR GUI
 *
 */
 
#ifndef OWR_GUI_H
#define OWR_GUI_H

#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <unistd.h>
#include <GL/glut.h>
#include "comms.h"
#include <ros/ros.h>
#include "../../devel/include/owr_camera_control/stream.h"
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

#define VIDEO_W 640
#define VIDEO_H 480

#define UP    0
#define DOWN  1
#define LEFT  2
#define RIGHT 3

// feed related
#define MAX_FEEDS 1
#define NO_FEEDS -1
#define ALL_FEEDS 0

#define ALPHA 0.3 // transparency factor

class OwrGui {
    
    public:
        OwrGui();
        void updateConstants(float battery, float signal, float ultrasonic, ListNode current, vector2D target, unsigned char *frame);
        
        // glut wrapper functions because it doesn't like c++ :(
        static OwrGui *instance;
        static void createInstance(OwrGui gui);
        static void glut_reshape(int w, int h);
        static void glut_idle();
        static void glut_display();
        static void glut_keydown(unsigned char key, int x, int y);
        static void glut_special_keydown(int keycode, int x, int y);
        static void glut_special_keyup(int keycode, int x, int y);
        
        void init();
        
    private:
        // OpenGL essential functions
        void reshape(int w, int h);
        void idle();
        void display();

        // OpenGL keyboard functions (mainly for debugging)
        void keydown(unsigned char key, int x, int y);
        //void keyup(unsigned char key, int x, int y);
        void special_keydown(int keycode, int x, int y);
        void special_keyup(int keycode, int x, int y);
        //draws button
        void drawButton(float, float, float, bool, char);

        // function to display some text
        void drawText(char *text, int x, int y);
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
        
        void *gpsGui;
        
        // status values
        float owr_battery;
        float owr_signal;
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
        int currentWindowH;
        int currentWindowW;
        int frameCounter;
        bool arrowKeys[4];
        GLuint feedTextures[MAX_FEEDS];
        int feedDisplayStatus;
        double scale;

        //ros stuff
        ros::Publisher streamPub;
        void toggleStream(int stream, bool active);
};


#endif
