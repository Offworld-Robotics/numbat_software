/*
 * Main source for OffWorld Robotics Widget Gui
 *
 * Draws a gui for ground station for BLUEsat OffWorld Robotics Groundstation
 *	Gui shows video feed controls
 *	GPS
 *	Signal strength
 *	Robot Battery level
 *
 * Contributers
 *
 */

// catkin_make // (add argument "-j4" for ros indigo)
// source devel/setup.bash
// rosrun owr_gui glutgui

#include "OwrGui.h"
#include "GpsGUI.h"

double cords[] =
{
	-33.914867, 151.225601,
	-33.916532, 151.236523,
	-33.919007, 151.235980,
	-33.918413, 151.232138,
	-33.920139, 151.231758,
	-33.919419, 151.226454,
	-33.914867, 151.225601
};

//#define DEBUG

int main(int argc, char **argv) {
	ros::init(argc, argv, "GUI");
	glutInit(&argc, argv);
	OwrGui gui;
	OwrGui::createInstance(gui);
	gui.instance->init();
	return EXIT_SUCCESS;
}
OwrGui * OwrGui::instance = NULL;

void OwrGui::createInstance(OwrGui gui) {
	instance = &gui;
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
	currentWindowH = WINDOW_H;
	currentWindowW = WINDOW_W;
	frameCounter = 0;
	
	for(int i = 0;i < MAX_FEEDS;i++) {
		feedTextures[i] = 0;
	}
	
	arrowKeys[0] = 0;
	arrowKeys[1] = 0;
	arrowKeys[2] = 0;
	arrowKeys[3] = 0;
	
	feedDisplayStatus = ALL_FEEDS;
	scale = DEFAULT_SCALE;
	
	srand(time(NULL));
	//generateTarget();
	
	ros::NodeHandle node;
	
	streamPub = node.advertise<owr_camera_control::stream>("control/activateFeeds",  1000);
}

void OwrGui::init(void) {
	toggleStream(0, true);
	GPSGUI *gpsnode = new GPSGUI(this);
	this->gpsGui = gpsnode;
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_W, WINDOW_H);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("OWR GUI");
	
	glGenTextures(MAX_FEEDS, this->feedTextures);
	
	for(int i = 0;i < MAX_FEEDS;i++) {
		glBindTexture(GL_TEXTURE_2D, this->feedTextures[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}
	glClearColor(1, 1, 1, 0);
	glShadeModel(GL_FLAT);
	//glEnable(GL_BLEND); // enables transparency in overlay items
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glutKeyboardFunc(keydown_wrapper);
	glutSpecialFunc(special_keydown_wrapper);
	glutSpecialUpFunc(special_keyup_wrapper);
	glutDisplayFunc(display_wrapper);
	glutReshapeFunc(reshape_wrapper);
	glutIdleFunc(idle_wrapper);
	glutMainLoop();
}


void OwrGui::updateConstants(float bat, float sig,float ultrason, ListNode cur, vector2D t, unsigned char *f) {
	owr_battery = bat;
	owr_signal = sig;
	
	if (cur != NULL) {
		GPSList.push_front(cur);
	}
	
	target = t;
	ultrasonic = ultrason;
	
	if (f != NULL) {
		glBindTexture(GL_TEXTURE_2D, feedTextures[0]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, VIDEO_W, VIDEO_H, 0, GL_RGB, GL_UNSIGNED_BYTE, f);
	}
	
	ROS_INFO("Updated Constants");
}


void OwrGui::idle(void) {
	ros::spinOnce();
	
	// debug - arrow key control
	if (arrowKeys[UP]) {
		scale += 100;
	}
	if (arrowKeys[DOWN]) {
		scale -= 100;
	}
	if (arrowKeys[LEFT]) {
		scale += 1000;
	}
	if (arrowKeys[RIGHT]) {
		scale -= 1000;
	}
	
	#ifdef DEBUG
	// debug - randomly generate GPS values every second
	if (frameCounter % 60 == 0) GPSAddRandPos();
	#endif
	
	// debug - print out the list of GPS co-ordinates
	//printGPSPath();
	
	#ifdef DEBUG	
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
	
	frameCounter++;
	if (frameCounter > 6001)
		frameCounter -= 6000;
		
	display();
	usleep(16666);
}

void OwrGui::drawBackground(void) {
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glColor3f(1, 1, 1);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, feedTextures[0]);
	
	// data from frame array is flipped, texcoords were changed to compensate
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(0, -currentWindowH); // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(currentWindowW, -currentWindowH); // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(currentWindowW, 0); // Top Right
		glTexCoord2f(0, 0); glVertex2i(0, 0); // Top Left
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

void OwrGui::display(void) {
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

void OwrGui::GPSAddRandPos() {
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

void OwrGui::GPSAddPos(double x, double y) {
	printf("GPSAddPos\n");
	ListNode n = (ListNode) malloc(sizeof(vector2D));
	n->x = x;
	n->y = y;
	GPSList.push_front(n);
}

void OwrGui::generateTarget() {
	double randn;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	target.x = randn + 151.139;
	randn = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000.0;
	target.y = randn - 33.718;
}

void OwrGui::printGPSPath() {
	printf("Begin\n");
	for(std::list<ListNode>::iterator i = GPSList.begin();i != GPSList.end(); ++i) {
		printf("%.15f, %.15f\n", (*i)->y, (*i)->x);
	}
	printf("End\n");
}

double distance(double x1, double y1, double x2, double y2) {
	return hypot(x2-x1, y2-y1);
}

// draws GPS path and co-ordinates near the centre of the window
void OwrGui::drawGPS() {
	glPushMatrix();
	glTranslated(currentWindowW/2, -3*currentWindowH/5, 0);
	
	if (GPSList.size() >= 2) {
		// draws out the path so that the forward direction of the rover always facing up on the screen
		glPushMatrix();
		
		ListNode first = *GPSList.begin();
		ListNode second = *(++GPSList.begin());
		double angle = -atan2(second->y - first->y, second->x - first->x) * 180.0 / PI - 90;
		glRotated(angle, 0, 0, 1);
		
		glScaled(scale, scale, 1);
		glTranslated(-(*GPSList.begin())->x, -(*GPSList.begin())->y, 0);
		glColor3f(0, 0, 0);
		
		glBegin(GL_LINE_STRIP);
		for (std::list<ListNode>::iterator i = GPSList.begin(); i != GPSList.end(); ++i) {
			glVertex2d((*i)->x, (*i)->y);
		}
		glEnd();
		glPopMatrix();
		glColor4f(0, 0, 1, ALPHA);
		glBegin(GL_POLYGON);
		glVertex2d(0, 0);
		glVertex2d(10, -10);
		glVertex2d(-10, -10);
		glEnd();
	}
	// draw text for GPS co-ordinates
	char GPSLat[30];
	char GPSLon[30];
	if (GPSList.size() == 0) {
		sprintf(GPSLat, "Lat: Unknown");
		sprintf(GPSLon, "Lon: Unknown");
	} else {
		sprintf(GPSLat, "Lat: %.10f", (*GPSList.begin())->y);
		sprintf(GPSLon, "Lon: %.10f", (*GPSList.begin())->x);
	}
	glColor4f(0, 0, 0, ALPHA);
	glTranslated(-50, -currentWindowH/4, 0);
	drawText(GPSLat, 0, 0);
	glTranslated(0, -20, 0);
	drawText(GPSLon, 0, 0);
	
	// draw text for opengl scale value
	char scaleValue[50];
	glTranslated(0, -40, 0);
	sprintf(scaleValue, "OpenGL Scale: %.0f", scale);
	drawText(scaleValue, 0, 0);
	glPopMatrix();
}

// draw the ultrasonic
void OwrGui::drawUltrasonic() {
	char ultrasonicText[30];
	glPushMatrix();
	glTranslated(currentWindowW/4, -3*currentWindowH/4, 0);
		
	// draw text for ultrasonic value
	glColor4f(0, 0, 0, ALPHA);
	glTranslated(-50, 50.0, 0);
	sprintf(ultrasonicText, "Ultrasonic: %f m", ultrasonic);
	drawText(ultrasonicText, 0, 0);
	glPopMatrix();
}

// draw the buttons
void OwrGui::drawButton(float a, float b, float c, bool active, char feedNumber) {
	
	if (active) {
		glColor4ub(VID_FEED_ACTIVE_BUTTON_RED, VID_FEED_ACTIVE_BUTTON_GREEN, VID_FEED_ACTIVE_BUTTON_BLUE, ALPHA * 255);
	} else {
		glColor4ub(VID_FEED_ACTIVE_NOT_LIVE_BUTTON_RED, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_GREEN, VID_FEED_ACTIVE_NOT_LIVE_BUTTON_BLUE, ALPHA * 255);
	}
	
	glTranslated(a, b, c);
	glRectd(-30, -25, 30, 25);
	glColor4f(0, 0, 1, ALPHA);
	glRasterPos2i(-5, -6);
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, feedNumber);
	
	
}

// draws feeds boxes on the left side of the window
void OwrGui::drawFeeds(void) {
	glPushMatrix();
	
	drawButton(50, -37.5, 0, true, '0');
	
	drawButton(0, -75, 0, false, '1');
	
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

	glTranslated(3*currentWindowW/4, -3*currentWindowH/4, 0);

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
	glColor4f(0.1, 0.1, 0.1, ALPHA);
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
	drawText(text, 0, 0);
	glTranslated(0, -20, 0);
	sprintf(text, "Front-Back: %.2fdeg", tiltY);
	drawText(text, 0, 0);
	
	glPopMatrix();
}

// draws the battery level near the top-right of the window
void OwrGui::drawBattery() {
	glPushMatrix();
	
	if (owr_battery < 3)
		glColor4f(1, 0, 0, ALPHA);
	else
		glColor4f(0, 1, 0, ALPHA);
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
	glColor4f(0, 0, 0, ALPHA);
	char text[] = "Battery";
	drawText(text, 15, 0);
	
	glPopMatrix();
}

// draw the signal level near the bottom-right of the window
void OwrGui::drawSignal() {
	glPushMatrix();
	
	if (owr_signal < 3)
		glColor4f(1, 0, 0, ALPHA);
	else
		glColor4f(0, 1, 0, ALPHA);
	if (owr_signal < 0)
		owr_signal = 0;
	if (owr_signal > 10)
		owr_signal = 10;
	
	glTranslated(currentWindowW - 125, 100 - currentWindowH, 0);
	
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
	glColor4f(0, 0, 0, ALPHA);
	char text[] = "Signal";
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
	
	FILE* proc = popen(PROGRAM_PATH,"r");
	if (proc) {
		printf("Failed to toggle stream\n");
	} else {
		printf("Toggle stream successful\n");
	}
}

void OwrGui::keydown(unsigned char key, int x, int y) {
	if (key == 27) {
		exit(0);
	} else if (key >= '0' && key <= '9')  {
		char str[2];
		str[0] = key;
		str[1] = '\0';

		toggleStream(atoi(str),true);
		
		printf("%c\n",key);
		#define PROGRAM_PATH "/opt/ros/hydro/bin/rosrun image_view image_view image:=/camera/image_raw"
		
		FILE* proc = popen(PROGRAM_PATH,"r");
		if (proc) {
			printf("Failed to toggle stream\n");
		} else {
			printf("Toggle stream successful\n");
		}
	} else if (key == 'q') {
		printGPSPath();
	} else if(key ==  ' ') {
		/*if (GPSList.size() < NUM_POINTS)
			GPSList.push_front(&gpslist[GPSList.size()]);
		else*/
			GPSList.clear();
	}
}

void OwrGui::special_keydown(int keycode, int x, int y) {
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

void OwrGui::special_keyup(int keycode, int x, int y) {
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
		x += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, text[i]);
	}
}


