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
#include "Video_Feed_Frame.hpp"

//#define DEBUG 1

int main(int argc, char **argv) {
	ros::init(argc, argv, "NavigationGUI");
	NavigationGUI gui(WINDOW_W, WINDOW_H, &argc, argv);
	gui.run();
	return EXIT_SUCCESS;
}

NavigationGUI::NavigationGUI(int width, int height, int *argc, char **argv) : GLUTWindow(width, height, argc, argv, "Navigation") {
	ros::NodeHandle node;
	streamPub = node.advertise<owr_messages::stream>("owr/control/activateFeeds", 1000);
	navigationNode = new NavigationNode(this);
	
	glClearColor(1, 1, 1, 0);
	glShadeModel(GL_FLAT);
	glEnable(GL_BLEND); // enables transparency
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glutKeyboardFunc(glut_keydown);
	//glutKeyboardUpFunc(glut_keyup);
	glutSpecialFunc(glut_special_keydown);
	glutSpecialUpFunc(glut_special_keyup);
	
	battery = 5;
	signal = 5;
	tiltX = 0; // tilt of left-right in degrees
	tiltY = 0; // tilt of forward-back in degrees
	ultrasonic = 10;
	memset(&currentPos, 0, sizeof(currentPos));
	pathRotation = 90;
	prevRotation = 90;
	cursorSpin = 0;

	for (int i = 0;i < NUM_ARROWKEYS;i++)
		arrowKeys[i] = false;

	for (int i = 0;i < TOTAL_FEEDS;i++)
		feedStatus[i] = FEED_OFFLINE;
	onlineFeeds = 0;

	// create videoScreen object
	videoScreen = new Video_Feed_Frame(width, height, 0.5, -0.5, 1, 1);

	scale = DEFAULT_SCALE;
	displayOverlay = true;
	srand(time(NULL));
	//generateTarget();
}

void NavigationGUI::reshape(int w, int h) {
	GLUTWindow::reshape(w, h);
	videoScreen->setNewWindowSize(w, h);
}

void NavigationGUI::updateInfo(float bat, float sig, float ultrason, ListNode cur, vector3D t) {
	battery = bat;
	signal = sig;
	
	if (cur != NULL) GPSList.push_front(cur);
	
	target = t;
	ultrasonic = ultrason;
	
	//ROS_INFO("Updated info");
}

void NavigationGUI::updateVideo(unsigned char *frame, int width, int height) {
	// use the Video_Feed_Frame object method
	videoScreen->setNewStreamFrame(frame, width, height);
	
	//ROS_INFO("Updated video");
}

void NavigationGUI::updateFeedsStatus(unsigned char *feeds, int numOnline) {
	memcpy(feedStatus, feeds, TOTAL_FEEDS*sizeof(unsigned char));
	onlineFeeds = numOnline;
	ROS_INFO("Navigation: Updating feeds: [%d,%d,%d,%d]", feeds[0], feeds[1], feeds[2], feeds[3]);
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
		//GPSAddRandPos();
		//printGPSPath(); // debug - print out the list of GPS co-ordinates
	}

	// debug - animate battery and signal
	battery += 0.01;
	signal -= 0.01;
	ultrasonic -= 0.01;
	if (battery < 0)
		battery = 0;
	if (battery > 10)
		battery = 10;
	if (signal < 0)	
		signal = 0;
	if (signal > 10)
		signal = 10;
	if (ultrasonic < 0)
		ultrasonic = 0;
	#endif
	
	// calculate the path rotation angle if possible
	if (GPSList.size() >= 2) {
		ListNode first = *GPSList.begin();
		ListNode second = *(++GPSList.begin());
		pathRotation = -atan2(second->lat - first->lat, second->lon - first->lon) * 180.0 / PI - 90;
	}
	
	frameCount++;
	if (frameCount > 6001)
		frameCount -= 6000;
	
	cursorSpin += 7*PI/180;
	if (cursorSpin > 2*PI)
		cursorSpin -= 2*PI;
	
	display();
	usleep(15000);
}

void NavigationGUI::drawVideo() {
	//Draw Video Feed to Screen
	videoScreen->draw();
}

void NavigationGUI::display() {
	glClearColor(1, 1, 1, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	drawVideo();
	if (displayOverlay) {
		drawFeedStatus();
		drawGPS();
		drawTilt();
		drawBattery();
		drawSignal();
		drawUltrasonic();
	}

	glutSwapBuffers();
}

void NavigationGUI::GPSAddRandPos() {
	double randlon, randlat;
	ListNode n = (ListNode) malloc(sizeof(vector3D));
	randlon = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	randlat = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	if (GPSList.size() == 0) {
		n->lon = randlon + 151.139;
		n->lat = randlat - 33.718;
	} else {
		randlon -= 1.0/1000.0/2.0;
		n->lon = (*GPSList.begin())->lon + randlon;
		randlat -= 1.0/1000.0/2.0;
		n->lat = (*GPSList.begin())->lat + randlat;
	}
	n->alt = 0;
	GPSList.push_front(n);
	//ROS_INFO("Generated random position: %lf, %lf, %lf\n", n->lat, n->lon, n->alt);
}

void NavigationGUI::GPSAddPos(double lon, double lat) {
	ListNode n = (ListNode) malloc(sizeof(vector3D));
	n->lon = lon;
	n->lat = lat;
	n->alt = 0;
	GPSList.push_front(n);
}

void NavigationGUI::generateTarget() {
	double randn;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	target.lon = randn + 151.139;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	target.lat = randn - 33.718;
	target.alt = 0;
}

void NavigationGUI::printGPSPath() {
	ROS_INFO("Begin");
	for(std::list<ListNode>::iterator i = GPSList.begin();i != GPSList.end(); ++i) {
		ROS_INFO("%.15f, %.15f", (*i)->lat, (*i)->lon);
	}
	ROS_INFO("End");
}

// draws GPS path and co-ordinates near the centre of the window
void NavigationGUI::drawGPS() {
	glPushMatrix();
	glTranslated(currWinW/2.0, -currWinH*3.0/5.0, 0);

	if (GPSList.size() > 0) {
		// draws out the path so that the forward direction of the rover always faces up on the screen
		glPushMatrix();
		glRotated(pathRotation, 0, 0, 1);
		
		glScaled(scale, scale, 1);
		glColor3f(0, 0, 0);
		
		vector3D currentPos = *(*GPSList.begin());
		
		glBegin(GL_LINE_STRIP);
		for (std::list<ListNode>::iterator i = GPSList.begin(); i != GPSList.end(); ++i)
			glVertex2d((*i)->lon - currentPos.lon, (*i)->lat - currentPos.lat);
		glEnd();
		if (GPSList.size() > 1) {
			glPointSize(5);
			glBegin(GL_POINTS);
			for (std::list<ListNode>::iterator i = ++GPSList.begin(); i != GPSList.end(); ++i)
				glVertex2d((*i)->lon - currentPos.lon, (*i)->lat - currentPos.lat);
			glEnd();
			glPointSize(1);
		}
		glPopMatrix();
	}
	
	// draw a cursor to indicate current rover position
	glPushMatrix();
	glTranslated(0, 15, 0);
	glColor4f(0, 0, 1, ALPHA);	
	glBegin(GL_POLYGON);
	glVertex2d(0, 0);
	glVertex2d(-10*cos(cursorSpin), -30);
	glVertex2d(10*cos(cursorSpin), -30);
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
		sprintf(GPSLat, "Lat: %.10f", (*GPSList.begin())->lat);
		sprintf(GPSLong, "Long: %.10f", (*GPSList.begin())->lon);	
		sprintf(GPSAlt, "Alt: %.10f", (*GPSList.begin())->alt);
	}
	
	glTranslated(-50, -currWinH/4.0, 0);
	
	glColor4f(0, 1, 0, TEXTBOX_ALPHA);
	glRecti(-20, 30, 250, -125);
	
	glColor4f(1, 0, 0, ALPHA);
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

// toggles between available streams
void NavigationGUI::toggleStream(int feed) {
	ROS_INFO("Navigation: Switching feed %d", feed);
	
	if (feedStatus[feed] == FEED_OFFLINE) {
		ROS_INFO("Navigation: Error: Feed %d is offline", feed);
		return;
	} else if (feedStatus[feed] == FEED_INACTIVE) {
		feedStatus[feed] = FEED_ACTIVE;
		for(int i = 0;i < TOTAL_FEEDS;i++) {
			if (i != feed && feedStatus[i] == FEED_ACTIVE) {
				owr_messages::stream off;
				feedStatus[i] = FEED_INACTIVE;
				off.stream = i;
				off.on = false;
				streamPub.publish(off);
			}
		}
	} else {
		feedStatus[feed] = FEED_INACTIVE;
	}
	
	owr_messages::stream msg;
	msg.stream = feed;
	if (feedStatus[feed] == FEED_ACTIVE) {
		msg.on = true;
	} else {
		msg.on = false;
	}
	streamPub.publish(msg);
	
	ros::spinOnce();
}

// draw the ultrasonic
void NavigationGUI::drawUltrasonic() {
	char text[50];
	glPushMatrix();
	glTranslated(100, 20 - currWinH, 0);	

	// draw text for ultrasonic value
	
	glTranslated(-50, 50, 0);
	glColor4f(0, 1, 0, TEXTBOX_ALPHA);
	glRecti(-30, 30, 350, -30);
	
	if (ultrasonic >= ULTRASONIC_MAX)
		sprintf(text, "Ultrasonic: No objects detected");
	else
		sprintf(text, "Ultrasonic: %.1fm", ultrasonic);
	glColor4f(1, 0, 0, ALPHA);
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glPopMatrix();
}

// draw the buttons
void NavigationGUI::drawButton(int feed) {
	if (feedStatus[feed] == FEED_ACTIVE)
		glColor4ub(FEED_ACTIVE_BUTTON_R, FEED_ACTIVE_BUTTON_G, FEED_ACTIVE_BUTTON_B, ALPHA*255);
	else if (feedStatus[feed] == FEED_INACTIVE)
		glColor4ub(FEED_INACTIVE_BUTTON_R, FEED_INACTIVE_BUTTON_G, FEED_INACTIVE_BUTTON_B, ALPHA*255);
	else
		glColor4ub(FEED_OFFLINE_BUTTON_R, FEED_OFFLINE_BUTTON_G, FEED_OFFLINE_BUTTON_B, ALPHA*255);

	glRecti(-30, -25, 30, 25);
	glColor4f(0, 0, 1, ALPHA);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, feed + '0');
}

// draws feeds boxes on the left side of the window
void NavigationGUI::drawFeedStatus() {
	glPushMatrix();
	glTranslated(50, -37.5, 0);
	for(int i = 0;i < TOTAL_FEEDS;i++) {
		drawButton(i);
		glTranslated(0, -75, 0);
	}

	glPopMatrix();
}

// draws the tilt angles for the left-right and front-back directions near the right of the screen
void NavigationGUI::drawTilt() {
	char text[30];
	glPushMatrix();
	glTranslated(currWinW - 150, 150 - currWinH, 0);
	
	glColor4f(0, 1, 0, TEXTBOX_ALPHA);
	glRecti(-100, 100, 100, -150);
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
	glColor4f(1, 0, 0, ALPHA);
	sprintf(text, "Roll: %.2fdeg", tiltX);
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glTranslated(0, -20, 0);
	sprintf(text, "Pitch: %.2fdeg", tiltY);
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

	glTranslated(currWinW - 125, -50, 0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(0, 30);
	glVertex2d(0, -30);
	glVertex2d(100, -30);
	glVertex2d(100, 30);
	glEnd();
	glBegin(GL_QUADS);
	glVertex2d(0, 30);
	glVertex2d(0, -30);
	glVertex2d(battery * 10, -30);
	glVertex2d(battery * 10, 30);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex2d(100, 15);
	glVertex2d(100, -15);
	glVertex2d(110, -15);
	glVertex2d(110, 15);
	glEnd();

	glTranslated(0, -50, 0);
	glColor4f(1, 0, 0, ALPHA);
	char text[30];
	sprintf(text, "%.2f%%", battery*10);
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

	glTranslated(currWinW - 300, -75, 0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(0, 0);
	glVertex2d(100, 0);
	glVertex2d(100, 50);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex2d(0, 0);
	glVertex2d(signal * 10, 0);
	glVertex2d(signal * 10, 5 * signal);
	glEnd();

	glTranslated(0, -20, 0);
	glColor4f(1, 0, 0, ALPHA);
	char text[30];
	sprintf(text, "%.2f%%", signal*10);
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 25, 0);

	glPopMatrix();
}

void NavigationGUI::keydown(unsigned char key, int x, int y) {
	if (key == 27) {
		exit(0);
	} else if (key >= '0' && key <= '3') {
		toggleStream(key - '0');
	} /*else if (key == 'q') {
		printGPSPath();
	} else if(key ==  ' ') {
		#ifdef DEBUG
		GPSAddRandPos();
		#endif
	}*/ else if (key == '\t') {
		displayOverlay = !displayOverlay;
	} else if (key == 'w') {
		videoScreen->zoom(ZOOM_IN);
	} else if (key == 's') {
		videoScreen->zoom(ZOOM_OUT);
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
