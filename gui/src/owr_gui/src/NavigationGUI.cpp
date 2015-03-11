/*
	Main source for OffWorld Robotics Navigation GUI
	Displayed features include:
	- Video feed
	- Video feed controls
	- GPS co-ordinates
	- Path history of rover
	- Signal strength
	- Robot Battery level
	- Tilting
	- Ultrasonic sensors

	Compile using: catkin_make
	Setup environment using: source devel/setup.bash
	Run using: rosrun owr_gui navigation
*/

#include "NavigationGUI.h"
#include "NavigationNode.h"

//#define DEBUG 1

#ifdef DEBUG
// debug gps co-ords makes up the area of UNSW
static double cords[] = {
	-33.914867, 151.225601,
	-33.916532, 151.236523,
	-33.919007, 151.235980,
	-33.918413, 151.232138,
	-33.920139, 151.231758,
	-33.919419, 151.226454,
	-33.914867, 151.225601
};
#endif

int main(int argc, char **argv) {
	ros::init(argc, argv, "NavigationGUI");
	NavigationGUI gui(&argc, argv);
	gui.run();
	return EXIT_SUCCESS;
}

NavigationGUI::NavigationGUI(int *argc, char **argv) : GLUTWindow() {
	streamPub = node.advertise<owr_camera_control::stream>("control/activateFeeds",  1000);
	
	navigationNode = new NavigationNode(this);
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_W, WINDOW_H);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Navigation");
	
	glGenTextures(1, &feedTexture);
	
	glBindTexture(GL_TEXTURE_2D, feedTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glClearColor(1, 1, 1, 0);
	glShadeModel(GL_FLAT);
	//glEnable(GL_DEPTH_TEST);
	//glEnable(GL_BLEND); // enables transparency
	//glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glutKeyboardFunc(glut_keydown);	//glutKeyboardUpFunc(glut_keyup);
	glutSpecialFunc(glut_special_keydown);
	glutSpecialUpFunc(glut_special_keyup);
	glutDisplayFunc(glut_display);
	glutReshapeFunc(glut_reshape);
	glutIdleFunc(glut_idle);
	
	battery = 5;
	signal = 5;
	tiltX = 0; // tilt of left-right in degrees
	tiltY = 0; // tilt of forward-back in degrees
	ultrasonic = 10;
	longitude = 0;
	latitude = 0;
	altitude = 0;
	pathRotation = 90;
	prevRotation = 90;
	cursorSpin = 0;

	for (int i = 0;i < NUM_ARROWKEYS;i++)
		arrowKeys[i] = false;

	for (int i = 0;i < NUM_FEEDS;i++)
		feedStatus[i] = false;
	currFeed = -1;
	numActiveFeeds = 0;

	scale = DEFAULT_SCALE;
	srand(time(NULL));
	//generateTarget();
	
	//start on stream 0
	toggleStream(0, true);
}

void NavigationGUI::run(void) {
	glutMainLoop();
}

void NavigationGUI::updateConstants(float bat, float sig, float ultrason, ListNode cur, double alt, vector2D t, unsigned char *f) {
	battery = bat;
	signal = sig;

	if (cur != NULL) {
		GPSList.push_front(cur);
	}
	
	altitude = alt;
	target = t;
	ultrasonic = ultrason;
	
	if (f != NULL) {
		glBindTexture(GL_TEXTURE_2D, feedTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, VIDEO_W, VIDEO_H, 0, GL_RGB, GL_UNSIGNED_BYTE, f);
	}
	
	ROS_INFO("Updated Constants");
}

void NavigationGUI::idle() {
	ros::spinOnce();

	// debug - arrow key control
	if (arrowKeys[UP]) {
		scale += 1000;
	}
	if (arrowKeys[DOWN]) {
		scale -= 1000;
	}
	if (arrowKeys[LEFT]) {
		scale += 10000;
	}
	if (arrowKeys[RIGHT]) {
		scale -= 10000;
	}

	#ifdef DEBUG
	// debug - randomly generate GPS values every second
	if (frameCount % 60 == 0) {
		GPSAddRandPos();
		//printGPSPath(); // debug - print out the list of GPS co-ordinates
	}

	// debug - animate battery and signal
	battery += 0.01;
	signal -= 0.01;
	ultrasonic -= 0.01;
	if (battery < 0)
		battery = 10;
	if (battery > 10)
		battery = 0;
	if (signal < 0)	
		signal = 10;
	if (signal > 10)
		signal = 0;
	if (ultrasonic < 0)
		ultrasonic = 10;
	#endif
	
	// calculate the path rotation angle if possible
	if (GPSList.size() >= 2) {
		ListNode first = *GPSList.begin();
		ListNode second = *(++GPSList.begin());
		pathRotation = -atan2(second->y - first->y, second->x - first->x) * 180.0 / PI - 90;
	}
	
	frameCount++;
	if (frameCount > 6001)
		frameCount -= 6000;
	
	cursorSpin += 1;
	if (cursorSpin > 360)
		cursorSpin -= 360;
	
	display();
	usleep(15000);
}

void NavigationGUI::drawBackground(void) {
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glColor3f(1, 1, 1);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, feedTexture);

	// data from frame array is flipped, texcoords were changed to compensate
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(0, -currWinH); // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(currWinW, -currWinH); // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(currWinW, 0); // Top Right
		glTexCoord2f(0, 0); glVertex2i(0, 0); // Top Left
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

void NavigationGUI::display(void) {
	glClearColor(1, 1, 1, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	drawBackground();
	drawFeeds();
	drawGPS();
	drawTilt();
	drawBattery();
	drawSignal();
	drawUltrasonic();

	glutSwapBuffers();
}

void NavigationGUI::GPSAddRandPos() {
	printf("Generated random position\n");
	double randx, randy;
	ListNode n = (ListNode) malloc(sizeof(vector2D));
	randx = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	randy = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	if (GPSList.size() == 0) {
		n->x = randx + 151.139;	
	n->y = randy - 33.718;
	} else {
		randx -= 1.0/1000.0/2.0;
		n->x = (*GPSList.begin())->x + randx;
		randy -= 1.0/1000.0/2.0;
		n->y = (*GPSList.begin())->y + randy;
	}
	GPSList.push_front(n);
}

void NavigationGUI::GPSAddPos(double x, double y) {
	printf("GPSAddPos\n");
	ListNode n = (ListNode) malloc(sizeof(vector2D));
	n->x = x;
	n->y = y;
	GPSList.push_front(n);
}

void NavigationGUI::generateTarget() {
	double randn;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	target.x = randn + 151.139;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	target.y = randn - 33.718;
}

void NavigationGUI::printGPSPath() {
	printf("Begin\n");
	for(std::list<ListNode>::iterator i = GPSList.begin();i != GPSList.end(); ++i) {
		printf("%.15f, %.15f\n", (*i)->y, (*i)->x);
	}
	printf("End\n");
}

// draws GPS path and co-ordinates near the centre of the window
void NavigationGUI::drawGPS() {
	glPushMatrix();
	glTranslated(currWinW/2, -3*currWinH/5, 0);

	if (GPSList.size() > 0) {
		// draws out the path so that the forward direction of the rover always faces up on the screen
		glPushMatrix();
		glRotated(pathRotation, 0, 0, 1);
		
		glScaled(scale, scale, 1);
		glColor3f(0, 0, 0);
		
		vector2D currentPos = *(*GPSList.begin());

		glBegin(GL_LINE_STRIP);
		for (std::list<ListNode>::iterator i = GPSList.begin(); i != GPSList.end(); ++i)
			glVertex2d((*i)->x - currentPos.x, (*i)->y - currentPos.y);
		glEnd();
		glPopMatrix();
	}
	
	// draw a cursor to indicate current rover position
	glPushMatrix();
	//glRotated(cursorSpin, 0, 1, 0);
	glColor4f(0, 0, 1, ALPHA);
	glBegin(GL_POLYGON);
	glVertex2d(0, 0);
	glVertex2d(-10, -20);
	glVertex2d(10, -20);
	glEnd();
	glPopMatrix();

	// draw text for GPS co-ordinates
	char GPSLat[30];
	char GPSLong[30];
	char GPSAlt[30];
	if (GPSList.size() == 0) {
		sprintf(GPSLat, "Lat: Unknown");
		sprintf(GPSLong, "Long: Unknown");
		sprintf(GPSAlt, "Alt: Unknown");
	} else {
		sprintf(GPSLat, "Lat: %.10f", (*GPSList.begin())->y);
		sprintf(GPSLong, "Long: %.10f", (*GPSList.begin())->x);	
		sprintf(GPSAlt, "Alt: %.10f", altitude);
	}
	glColor4f(0, 0, 0, ALPHA);
	glTranslated(-50, -currWinH/4, 0);
	drawText(GPSLat, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -20, 0);
	drawText(GPSLong, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -20, 0);
	drawText(GPSAlt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);

	// draw text to display OpenGL scale value
	char scaleValue[50];
	glTranslated(0, -40, 0);
	sprintf(scaleValue, "OpenGL Scale: %.0f", scale);
	drawText(scaleValue, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glPopMatrix();
}

// draw the ultrasonic
void NavigationGUI::drawUltrasonic() {
	char text[50];
	glPushMatrix();
	glTranslated(100, 50 - currWinH, 0);	

	// draw text for ultrasonic value
	glColor4f(1 - ultrasonic/ULTRASONIC_MAX, ultrasonic/ULTRASONIC_MAX, 0, ALPHA);
	glTranslated(-50, 50, 0);
	if (ultrasonic >= ULTRASONIC_MAX)
		sprintf(text, "Ultrasonic: No objects detected");
	else
		sprintf(text, "Ultrasonic: %.1f m", ultrasonic);
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glPopMatrix();
}

// draw the buttons
void NavigationGUI::drawButton(bool isActive, int feed) {
	if (isActive)
		glColor4ub(VID_FEED_ACTIVE_BUTTON_RED, VID_FEED_ACTIVE_BUTTON_GREEN, VID_FEED_ACTIVE_BUTTON_BLUE, ALPHA * 255);
	else
		glColor4ub(VID_FEED_ACTIVE_NOT_LIVE_BUTTON_RED, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_GREEN, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_BLUE, ALPHA * 255);

	glRectd(-30, -25, 30, 25);
	glColor4f(0, 0, 1, ALPHA);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, feed + '0');
}

// draws feeds boxes on the left side of the window
void NavigationGUI::drawFeeds() {
	glPushMatrix();
	glTranslated(50, -37.5, 0);
	for(int i = 0;i < NUM_FEEDS;i++) {
		drawButton(feedStatus[i], i);
		glTranslated(0, -75, 0);
	}

	glPopMatrix();
}

// draws the tilt angles for the left-right and front-back directions near the right of the screen
void NavigationGUI::drawTilt() {
	char text[30];
	glPushMatrix();
	glTranslated(3*currWinW/4, -3*currWinH/4, 0);
	glPushMatrix();

	// draw horizon
	glTranslated(0, 100*tan(-tiltY*PI/180.0),0);
	glRotated(-tiltX,0,0,1);
	glBegin(GL_LINE_STRIP);
	glVertex2d(-70,0);
	glVertex2d(70,0);
	glEnd();

	// draw sky indicator lines
	glPushMatrix();
	glColor4f(0, 0, 1, ALPHA);
	glBegin(GL_QUADS);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glEnd();
	glPopMatrix();

	// draw ground (ie below horizon) indicator lines
	glPushMatrix();
	glColor4f(0.1, 0.1, 0.1, ALPHA);
	glBegin(GL_QUADS);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_HALF_WIDTH,0);
	glVertex2d(ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,-ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glVertex2d(-ARTIFICIAL_HORIZON_SKY_MIN_HALF_WIDTH,-ARTIFICIAL_HORIZON_SKY_HEIGHT);
	glEnd();
	glPopMatrix();
	glPopMatrix();

	//DRAW robot indicator
	glColor4f(1, 0, 0, ALPHA);
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
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -20, 0);
	sprintf(text, "Front-Back: %.2fdeg", tiltY);
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);

	glPopMatrix();
}

// draws the battery level near the top-right of the window
void NavigationGUI::drawBattery() {
	glPushMatrix();
	
	if (battery < 3)
		glColor4f(1, 0, 0, ALPHA);
	else
		glColor4f(0, 1, 0, ALPHA);

	if (battery < 0)
		battery = 0;
	if (battery > 10)
		battery = 10;

	glTranslated(currWinW - 125, -50, 0);
	glBegin(GL_LINE_LOOP);
	glVertex2i(0, 30);
	glVertex2i(0, -30);
	glVertex2i(100, -30);
	glVertex2i(100, 30);
	glEnd();
	glBegin(GL_QUADS);
	glVertex2i(0, 30);
	glVertex2i(0, -30);
	glVertex2i(battery * 10, -30);
	glVertex2i(battery * 10, 30);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex2i(100, 15);
	glVertex2i(100, -15);
	glVertex2i(110, -15);
	glVertex2i(110, 15);
	glEnd();

	glTranslated(0, -50, 0);
	glColor4f(0, 0, 0, ALPHA);
	char text[] = "Battery";
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 15, 0);

	glPopMatrix();
}

// draw the signal level near the bottom-right of the window
void NavigationGUI::drawSignal() {
	glPushMatrix();

	if (signal < 3)	
		glColor4f(1, 0, 0, ALPHA);
	else
		glColor4f(0, 1, 0, ALPHA);

	if (signal < 0)
		signal = 0;
	if (signal > 10)
		signal = 10;

	glTranslated(currWinW - 125, 100 - currWinH, 0);
	glBegin(GL_LINE_LOOP);
	glVertex2i(0, 0);
	glVertex2i(100, 0);
	glVertex2i(100, 50);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex2i(0, 0);
	glVertex2i(signal * 10, 0);
	glVertex2i(signal * 10, 5 * signal);
	glEnd();

	glTranslated(0, -20, 0);
	glColor4f(0, 0, 0, ALPHA);
	char text[] = "Signal";
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 25, 0);

	glPopMatrix();
}

void NavigationGUI::toggleStream(int feed, bool active) {
	if (!feedStatus[feed]) {
		printf("Switching to feed %d\n", feed);

		if (currFeed != -1)
			feedStatus[currFeed] = FEED_INACTIVE;
		currFeed = feed;
		feedStatus[feed] = FEED_ACTIVE_DISPLAY;	

		owr_camera_control::stream msg;
		msg.stream = feed;
		msg.on = active;
		streamPub.publish(msg);

		ros::spinOnce();
	}

	/*#define PROGRAM_PATH "/opt/ros/indigo/bin/rosrun image_view image_view image:=/camera/image_raw"
		FILE* proc = popen(PROGRAM_PATH,"r");
	if (proc) {
		printf("Failed to toggle stream\n");
	} else {
		printf("Toggle stream successful\n");
	}*/
}

void NavigationGUI::keydown(unsigned char key, int x, int y) {
	if (key == 27) {
		exit(0);
	} else if (key >= '0' && key <= '3') {
		toggleStream(key - '0', true);
	} else if (key == 'q') {
		printGPSPath();
	} else if(key ==  ' ') {
		/*if (GPSList.size() < NUM_POINTS)
			GPSList.push_front(&gpslist[GPSList.size()]);
		else*/
			GPSList.clear();
	}
}

void NavigationGUI::special_keydown(int keycode, int x, int y) {
	switch (keycode) {
	case GLUT_KEY_UP:
		arrowKeys[UP] = 1;
		break;
	case GLUT_KEY_DOWN:
		arrowKeys[DOWN] = 1;
		break;
	case GLUT_KEY_LEFT:
		arrowKeys[LEFT] = 1;
		break;
	case GLUT_KEY_RIGHT:
		arrowKeys[RIGHT] = 1;
		break;
	}
}

void NavigationGUI::special_keyup(int keycode, int x, int y) {
	switch (keycode) {
	case GLUT_KEY_UP:
		arrowKeys[UP] = 0;
		break;
	case GLUT_KEY_DOWN:
		arrowKeys[DOWN] = 0;
		break;
	case GLUT_KEY_LEFT:
		arrowKeys[LEFT] = 0;
		break;
	case GLUT_KEY_RIGHT:
		arrowKeys[RIGHT] = 0;
		break;
	}
}

void NavigationGUI::keyup(unsigned char key, int x, int y) {

}