/*
 * Main source for OffWorld Robotics Widget Gui
 *
 * Draws a gui for ground station for BLUEsat OffWorld Robotics Groundstation
 *    Gui shows video feed controls
 *    GPS
 *    Signal strength
 *    Robot Battery level
 *
 * Contributers
 *
 */

// catkin_make // (add argument "-j4" for ros indigo)
// source devel/setup.bash
// rosrun owr_gui glutgui

#include "OwrGui.h"
#include "GpsGUI.h"





int main(int argc, char **argv) {
    ros::init(argc, argv, "GUI");
    glutInit(&argc, argv);
    OwrGui gui;
    //printf("gui  %lx\n", (long) (this));
	OwrGui::createInstance(gui);
	gui.instance->init();
	return EXIT_SUCCESS;
}
OwrGui * OwrGui::instance = NULL;

void OwrGui::createInstance(OwrGui gui) {
    instance =&gui;
}


//glut wrapper functions because it dosen't life c++ :(
void OwrGui::reshape_wrapper(int w, int h) {
    instance->reshape(w,h);
}
void OwrGui::idle_wrapper() {
    instance->idle();
}
void OwrGui::display_wrapper() {
    instance->display();
}
void OwrGui::keydown_wrapper(unsigned char key, int x, int y) {
    instance->keydown(key,x,y);
}
void OwrGui::special_keydown_wrapper(int keycode, int x, int y) {
    instance->special_keydown(keycode,x,y);
}
void OwrGui::special_keyup_wrapper(int keycode, int x, int y) {
    instance->special_keyup(keycode, x,y);
}  


OwrGui::OwrGui() {
    owr_battery = 0;
    owr_signal = 0;
    tiltX = 0; // tilt of left-right in degrees
    tiltY = 0; // tilt of forward-back in degrees
    ultrasonic = 0;
    longitude = 0;
    latitude = 0;
    prevAngle = 90;
    
    
    // GPS related variables
    path = NULL;
    currentWindowH = WINDOW_H;
    currentWindowW = WINDOW_W;
    frame = 0;
    arrowKeys[0] = 0;
    arrowKeys[1] = 0;
    arrowKeys[2] = 0;
    
    srand(time(NULL));
	generateTarget();
	
	ros::NodeHandle node;
	
    streamPub = node.advertise<owr_camera_control::stream>("control/activateFeeds",  1000);
    
}

void OwrGui::init(void) {
    printf("gui  %lx\n", (long) (this));
    toggleStream(0,true);
	GPSGUI *gpsnode = new GPSGUI(this);
	this->gpsGui = gpsnode;
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_W, WINDOW_H);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("OWR GUI");
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(&keydown_wrapper);
	glutSpecialFunc(special_keydown_wrapper);
	glutSpecialUpFunc(special_keyup_wrapper);
	glutDisplayFunc(display_wrapper);
	glutReshapeFunc(reshape_wrapper);
	glutIdleFunc(idle_wrapper);
	glutMainLoop();

}


void OwrGui::updateConstants(float bat, float sig,float ultrason, ListNode points, vector2D tar) {
    owr_battery = bat;
    owr_signal = sig;
    this->path = points;
    //printf("path %f, %f", this->path->x, this->path->y);
    target = tar;
    ultrasonic = ultrason;
    ROS_INFO("Updated Constants");
}


void OwrGui::idle(void) {
	ros::spinOnce();
	
	/*// produce random tilt data
	int randn = rand() % 100 + 1;
	if (randn <= 25)
		tiltX++;
	else if (randn <= 50)
		tiltX--;
	else if (randn <= 75)
		tiltY++;
	else
		tiltY--;*/
	
	// debug - arrow key control for tilt
	if (arrowKeys[UP])
		tiltY--;
	if (arrowKeys[DOWN])
		tiltY++;
	if (arrowKeys[LEFT])
		tiltX--;
	if (arrowKeys[RIGHT])
		tiltX++;

   // clean tilt data
	if ((int) tiltY % 90 == 0) tiltY = 0;
	
	#ifdef RANDOM
	// debug - randomly generate GPS values every second
	if (frame == 0 || frame % 60 == 0) GPSAddRandPos();
	#endif
	
	// debug - arrow key control for path
	/*if (path != NULL) {
		double X = path->x, Y = path->y;
		double inc = 1.0/1000.0/200.0;
		if (arrowKeys[UP])
			Y += inc;
		if (arrowKeys[DOWN])
			Y -= inc;
		if (arrowKeys[LEFT])
			X -= inc;
		if (arrowKeys[RIGHT])
			X += inc;
		GPSAddPos(X, Y);
	} else if (path == NULL) {
		GPSAddRandPos();
	}*/
	
	// debug - print out the list of GPS co-ordinates
	printGPSPath();
	
	#ifdef RANDOM	
	// debug - animate battery and signal
	owr_battery += 0.01;
	owr_signal -= 0.01;
	if (owr_battery < 0)
		owr_battery = 10;
	if (owr_battery > 10)
		owr_battery = 0;
	if (owr_signal < 0)
		owr_signal = 10;
	if (owr_signal > 10)
		owr_signal = 0;
	#endif
	
	frame++;
	if (frame > 6001)
		frame -= 6000;
		
	display();
	usleep(16666);
}

void OwrGui::display(void) {
	glClearColor(1,1,1,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	drawFeeds();
	drawGPS();
	drawTilt();
	drawBattery();
	drawSignal();
	drawUltrasonic();
	glutSwapBuffers();
}

void OwrGui::GPSAddRandPos() {
    printf("random pos");
	double randx, randy;
	ListNode rest = path;
	path = (ListNode) malloc(sizeof(_vector2D));
	randx = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	randy = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	if (rest == NULL) {
		path->x = randx + 151.139;
		path->y = randy - 33.718;
	} else {
		randx -= 1.0/1000.0/2.0;
		path->x = rest->x + randx;
		randy -= 1.0/1000.0/2.0;
		path->y = rest->y + randy;
	}
	path->next = rest;
}

void OwrGui::GPSAddPos(double x, double y) {
    printf("GPSAddPos");
	ListNode rest = path;
	path = (ListNode) malloc(sizeof(_vector2D));
	path->x = x;
	path->y = y;
	path->next = rest;
}

void OwrGui::generateTarget() {
	double randn;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	target.x = randn + 151.139;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	target.y = randn - 33.718;
	target.next = NULL;
}

void OwrGui::printGPSPath() {
	ListNode curr = path;
	std::cout << "Start" << std::endl;
	while (curr != NULL) {
		printf("%.15f, %.15f\n", curr->x, curr->y);
		curr = curr->next;
	}
	std::cout << "End" << std::endl;
}

double distance(double x1, double y1, double x2, double y2) {
	return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

// draws GPS path and co-ordinates near the centre of the window
void OwrGui::drawGPS() {
    //printf("%lx", this);
	char GPSLat[30];
	char GPSLong[30];
	char scale[] = "Scale: 1 metre";
	glPushMatrix();
	glTranslated(400, -WINDOW_H/3, 0);
	
	// draw the rover as a red square
	glPointSize(10);
	glBegin(GL_POINTS);
	glColor3f(1,0,0);
	glVertex2d(0, 0);
	glEnd();
		
	printf("drawing gps\n");
	//printf("THe path %lx\n", &path);
	ListNode list = NULL;
	if (gpsGui) {
	    list =  ((GPSGUI *)gpsGui)->list;
	    printf("\nyay\n");
	}
	
	if (list != NULL) {
	    printf("path exists!!\n");
		longitude = list->x;
		latitude = list->y;
		
		// draw out the scale of the path
		glPushMatrix();
		glTranslated(-20, -WINDOW_H/3 - 15, 0);
		glColor3f(0,0,0);
		glScaled(SCALE,SCALE,1);
		glBegin(GL_LINE_STRIP);
		glColor3f(0,0,0);
		glVertex2d(0,0);
		glVertex2d(0.0001 / 0.9059, 0); // scale - this is 1 metre (approx)
		glEnd();
		glPopMatrix();
		glPushMatrix();
		glTranslated(10, -WINDOW_H/3 - 20, 0);
		drawText(scale, 0,0);
		glPopMatrix();
		
		// draws out the path so that the forward direction of the rover always facing up on the screen
		glPushMatrix();
		glScaled(SCALE,SCALE,1);
		
		glColor3f(0, 0, 1);
		double xoff = list->x, yoff = list->y;
		if (list->next != NULL) {
			double angle = -atan2(list->next->y - list->y, list->next->x - list->x) * 180.0/PI - 90;
			// if the rover did not move, its orientation is the same as previously
			if (angle == 0)
				angle = prevAngle;
			else
				prevAngle = angle;
			glRotated(angle, 0, 0, 1);
		}
		glTranslated(-xoff, -yoff, 0);
		ListNode curr = list;
		ListNode prev = NULL;
		
		glBegin(GL_LINE_STRIP);
		while (curr != NULL) {
			glVertex2d(curr->x, curr->y);
			prev = curr;
			curr = curr->next;
		}
		glEnd();
		
		// draw the origin point bigger
		glPointSize(10);
		glBegin(GL_POINTS);
		glVertex2d(prev->x, prev->y);
		glEnd();
		
		// draw the path to target green
		glColor3f(0,1,0);
		glBegin(GL_LINES);
		glVertex2d(list->x, list->y);
		glVertex2d(target.x, target.y);
		glEnd();
		glBegin(GL_POINTS);
		glVertex2d(target.x, target.y);
		glEnd();
		
		glPointSize(1);
		glPopMatrix();
		
		// draw text for GPS co-ordinates
		glColor3f(0, 0, 0);
		glTranslated(-50, -WINDOW_H/2, 0);
		sprintf(GPSLat, "Lat: %.10f", latitude);
		sprintf(GPSLong, "Lon: %.10f", longitude);
		drawText(GPSLat, 0, 0);
		glTranslated(0, -20, 0);
		drawText(GPSLong, 0, 0);
	}
	glPopMatrix();
}

// draw the ultrasonic
void OwrGui::drawUltrasonic() {
	char ultrasonicText[30];
	glPushMatrix();
	glTranslated(400, -WINDOW_H/3, 0);
		
	// draw text for GPS co-ordinates
	glColor3f(0, 0, 0);
	glTranslated(-50, 50.0, 0);
	sprintf(ultrasonicText, "Ultrasonic: %f m", ultrasonic);
	drawText(ultrasonicText, 0, 0);
	glPopMatrix();
}

// draw the buttons
void OwrGui::drawButton(float a, float b, float c, bool active, char feedNumber) {
    
    if (active) {
        glColor3ub(VID_FEED_ACTIVE_BUTTON_RED, VID_FEED_ACTIVE_BUTTON_GREEN, VID_FEED_ACTIVE_BUTTON_BLUE);
    } else {
        glColor3ub(VID_FEED_ACTIVE_NOT_LIVE_BUTTON_RED, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_GREEN, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_BLUE);
    }
    
    glTranslated(a, b, c);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, feedNumber);
    
    
}

// draws feeds boxes on the left side of the window
void OwrGui::drawFeeds(void) {
	glPushMatrix();
	
	drawButton(50,-37.5,0,true, '0');
	
	drawButton(0, -75, 0,false, '1');
	
	drawButton(0, -75, 0, false, '2');
	
	drawButton(0, -75, 0, false, '3');
	
	/*
	glColor3ub(VID_FEED_ACTIVE_NOT_LIVE_BUTTON_RED, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_GREEN, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_BLUE);
	glTranslated(0, -75, 0);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '2');
	
	glColor3ub(VID_FEED_ACTIVE_NOT_LIVE_BUTTON_RED, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_GREEN, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_BLUE);
	glTranslated(0, -75, 0);
	glRectd(-30, -25, 30, 25);

	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '3');
	
	glColor3ub(VID_FEED_INACTIVE_BUTTON_RED, VID_FEED_INACTIVE_BUTTON_GREEN, VID_FEED_INACTIVE_BUTTON_BLUE);
	glTranslated(0, -75, 0);
	glRectd(-30, -25, 30, 25);
	glColor3f(0.0, 0.0, 1.0);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '4');
	*/
	glPopMatrix();
}

// draws the tilt angles for the left-right and front-back directions near the right of the screen
void OwrGui::drawTilt() {
	char text[30];
	glPushMatrix();

	glTranslated(720, -WINDOW_H/2, 0);

   glPushMatrix();

        // draw horizon
	glTranslated(0, 100*tan(-tiltY*PI/180),0);
	glRotated(-tiltX,0,0,1);
	glBegin(GL_LINE_STRIP);
	glVertex2d(-70,0);
	glVertex2d(70,0);
	glEnd();

        // draw sky indicator lines
        glPushMatrix();
	glColor3f(0,0,1);
        //glTranslated(0, 15, 0);
        glBegin(GL_QUADS);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	//glEnd();
        //glTranslated(0, 15, 0);
        //glBegin(GL_LINE_STRIP);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glEnd();
        glPopMatrix();

        // draw ground (ie below horizon) indicator lines
        glPushMatrix();
	glColor3f(0.1,0.1,0.1);
        glBegin(GL_QUADS);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	//glEnd();
        //glTranslated(0, 15, 0);
        //glBegin(GL_LINE_STRIP);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,-ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,-ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glEnd();
        glPopMatrix();

        glPopMatrix();

        //DRAW robot indicator
	glColor3f(1,0,0);
	glBegin(GL_LINE_STRIP);
	glVertex2d(-70,0);
	glVertex2d(70,0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glVertex2d(0,-70);
	glVertex2d(0,70);
	glEnd();

        // Draw Tilt-Text
	glTranslated(-50, -100, 0);
	sprintf(text, "Left-Right: %.2fdeg", tiltX);
	drawText(text, 0, 0);
	glTranslated(0, -20, 0);
	sprintf(text, "Front-Back: %.2fdeg", tiltY);
	drawText(text, 0, 0);
	
	glPopMatrix();
	
	// todo: write leftright inside the crosshair
}

// draws the battery level near the top-right of the window
void OwrGui::drawBattery() {
	glPushMatrix();
	
	if (owr_battery < 3)
		glColor3f(1,0,0);
	else
		glColor3f(0,1,0);
	if (owr_battery < 0)
		owr_battery = 0;
	if (owr_battery > 10)
		owr_battery = 10;
	
	glTranslated(currentWindowW - 125, -50, 0);
	glBegin(GL_LINE_LOOP);
	glVertex2i(0, 30);
	glVertex2i(0, -30);
	glVertex2i(100, -30);
	glVertex2i(100, 30);
	glEnd();
	glBegin(GL_QUADS);
	glVertex2i(0, 30);
	glVertex2i(0, -30);
	glVertex2i(owr_battery * 10, -30);
	glVertex2i(owr_battery * 10, 30);
	glEnd();
	
	glBegin(GL_LINE_LOOP);
	glVertex2i(100, 15);
	glVertex2i(100, -15);
	glVertex2i(110, -15);
	glVertex2i(110, 15);
	glEnd();
	
	glTranslated(0, -50, 0);
	glColor3f(0, 0, 0);
	char text[] = "battery";
	drawText(text, 15, 0);
	
	glPopMatrix();
}

// draw the signal level near the bottom-right of the window
void OwrGui::drawSignal() {
	glPushMatrix();
	
	if (owr_signal < 3)
		glColor3f(1,0,0);
	else
		glColor3f(0,1,0);
	if (owr_signal < 0)
		owr_signal = 0;
	if (owr_signal > 10)
		owr_signal = 10;
	
	glTranslated(currentWindowW - 125, -((int) currentWindowH - 100), 0);
	
	glBegin(GL_LINE_LOOP);
	glVertex2i(0, 0);
	glVertex2i(100, 0);
	glVertex2i(100, 50);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex2i(0, 0);
	glVertex2i(owr_signal * 10, 0);
	glVertex2i(owr_signal * 10, 5 * owr_signal);
	glEnd();
	
	glTranslated(0, -20, 0);
	glColor3f(0, 0, 0);
	char text[] = "signal";
	drawText(text, 25, 0);
	
	glPopMatrix();
}




void OwrGui::toggleStream(int stream, bool active) {

    owr_camera_control::stream msg;
    msg.stream = stream;
    msg.on = active;
    streamPub.publish(msg);
    
    ros::spinOnce();
    printf("%d\n",stream);
   #define PROGRAM_PATH "/opt/ros/hydro/bin/rosrun image_view image_view image:=/camera/image_raw"
    //#define PROGRAM_PATH "gedit&"
    FILE* proc = popen(PROGRAM_PATH,"r");
    if (proc) {
        printf("fail\n");
    } else {
        printf("success\n");
        
    }

}



void OwrGui::keydown(unsigned char key, int x, int y) {

    
	if (key==27) {
		exit(0);
    } else if (key >= '0' && key <= '9')  {
        
        char str[2];
        str[0] = key;
        str[1] = '\0';

        toggleStream(atoi(str),true);
        
        printf("%c\n",key);
       #define PROGRAM_PATH "/opt/ros/hydro/bin/rosrun image_view image_view image:=/camera/image_raw"
       //#define PROGRAM_PATH "gedit&"
	    FILE* proc = popen(PROGRAM_PATH,"r");
	    if (proc) {
	        printf("fail\n");
	    } else {
	        printf("success\n");
	        
	    }

	}
}

void OwrGui::special_keydown(int keycode, int x, int y) {
	switch (keycode) {
	case GLUT_KEY_UP:
		arrowKeys[0] = 1;
		break;
	case GLUT_KEY_DOWN:
		arrowKeys[1] = 1;
		break;
	case GLUT_KEY_LEFT:
		arrowKeys[2] = 1;
		break;
	case GLUT_KEY_RIGHT:
		arrowKeys[3] = 1;
		break;
	}
}

void OwrGui::special_keyup(int keycode, int x, int y) {
	switch (keycode) {
	case GLUT_KEY_UP:
		arrowKeys[0] = 0;
		break;
	case GLUT_KEY_DOWN:
		arrowKeys[1] = 0;
		break;
	case GLUT_KEY_LEFT:
		arrowKeys[2] = 0;
		break;
	case GLUT_KEY_RIGHT:
		arrowKeys[3] = 0;
		break;
	}
}

void OwrGui::reshape(int w, int h) {
	currentWindowH = h;
	currentWindowW = w;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, (GLdouble)w, -(GLdouble)h, 0); // (0,0) is top left corner of window, use cartesian co-ordinates
	glMatrixMode(GL_MODELVIEW);
}

void OwrGui::drawText(char *text, int x, int y) {
	for (unsigned int i = 0; i < strlen(text); i++) {
		glRasterPos3f(x, y, 0);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
		x += 10;
	}
}


