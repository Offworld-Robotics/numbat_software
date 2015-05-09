/*
	Main source for OffWorld Robotics Site Analysis GUI
	
	Displayed features include:
	- Panoramic image of sites
	- High-resolution image of sites
	- GPS co-ordinates
	- pH level
	- Humidity
	- Ultrasonic sensors
	
	
	Compile using: catkin_make
	Setup environment using: source devel/setup.bash
	Run using: rosrun owr_gui analysis
*/

#include "GLUTWindow.h"
#include "AnalysisGUI.h"
#include "AnalysisNode.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "AnalysisGUI");
	AnalysisGUI gui(WINDOW_W, WINDOW_H, &argc, argv);
	gui.run();
	return EXIT_SUCCESS;
}

AnalysisGUI::AnalysisGUI(int width, int height, int *argc, char **argv) : GLUTWindow(width, height, argc, argv, "Analysis") {
	streamPub = node.advertise<owr_messages::stream>("owr/control/activateFeeds", 1000);
	analysisNode = new AnalysisNode(this);
	
	glGenTextures(NUM_IMAGES, imgTextures);
	
	for(int i = 0;i < NUM_IMAGES;i++) {
		glBindTexture(GL_TEXTURE_2D, imgTextures[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}
	
	glClearColor(1, 1, 1, 0);
	glShadeModel(GL_FLAT);
	
	//glEnable(GL_BLEND); // enables transparency
	//glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glutKeyboardFunc(glut_keydown);
	//glutKeyboardUpFunc(glut_keyup);
	glutSpecialFunc(glut_special_keydown);
	glutSpecialUpFunc(glut_special_keyup);
	
	latitude = 0;
	longitude = 0;
	altitude = 0;
	pH = 0;
	humidity = 0;
	ultrasonic = 0;
	
	arrowKeys[0] = 0;
	arrowKeys[1] = 0;
	arrowKeys[2] = 0;
	arrowKeys[3] = 0;
	
	requestPano = false;
	requestHires = false;
	receivedPano[PANORAMIC0] = false;
	receivedPano[PANORAMIC1] = false;
	panoImgs[0] = panoImgs[1] = NULL;
	
	usleep(150000);
	owr_messages::stream msg;
	msg.stream = 0;
	msg.on = true;
	streamPub.publish(msg);
	ros::spinOnce();
}

void AnalysisGUI::updateSiteInfo(double lat, double lon, double alt, float PH, float usonic, float humid) {
	latitude = lat;
	longitude = lon;
	altitude = alt;
	pH = PH;
	ultrasonic = usonic;
	humidity = humid;
	
	//ROS_INFO("Updated Constants");
}

void AnalysisGUI::updateVideo(unsigned char *frame, int width, int height, int channel) {
	if (frame != NULL) {
		glBindTexture(GL_TEXTURE_2D, imgTextures[channel]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, frame);
		if (channel <= 1 && requestPano && !receivedPano[channel]) {
			receivedPano[channel] = true;
			if (panoImgs[channel] != NULL) {
				free(panoImgs[channel]);
			}
			panoImgs[channel] = (unsigned char *)malloc(width*height*3*sizeof(unsigned char));
			memcpy(panoImgs[channel], frame, width*height*3*sizeof(unsigned char));
		}
	}
	
	//ROS_INFO("Updated video");
}

void AnalysisGUI::display() {
	glClear(GL_COLOR_BUFFER_BIT);
	
	drawButtons();
	drawImages();
	drawTextInfo();
	
	glutSwapBuffers();
}

void AnalysisGUI::idle() {
	ros::spinOnce();
	
	if (requestPano && receivedPano[0] && receivedPano[1]) {
		requestPano = 0;
		receivedPano[0] = 0;
		receivedPano[1] = 0;
		
		// img data should be saved in panoImgs
		
		saveBMPFile("/home/yiweih/pano0.bmp", panoImgs[0], 640, 480);
		saveBMPFile("/home/yiweih/pano1.bmp", panoImgs[1], 640, 480);
		
		owr_messages::stream msg;
		msg.stream = 0;
		msg.on = true;
		streamPub.publish(msg);
		ros::spinOnce();
	}
	
	display();
	usleep(15000);
}

void AnalysisGUI::drawButtons() {
	char text[20];
	glPushMatrix();
	glColor3f(0.7, 0.7, 0.7);
	glRecti(10, -10, 160, -90);
	glRecti(170, -10, 320, -90);
	glRecti(330, -10, 480, -90);
	glColor3f(1, 1, 0);
	sprintf(text, "GET PANO");
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 40, -60);
	sprintf(text, "GET HRES");
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 200, -60);
	glPopMatrix();
}

void AnalysisGUI::drawImages() {
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	
	glColor3f(1, 1, 1);
	
	// data from frame array is flipped, texcoords were changed to compensate
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	
	glBindTexture(GL_TEXTURE_2D, imgTextures[PANORAMIC0]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(100, -400);  // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(400, -400);  // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(400, -100);  // Top Right
		glTexCoord2f(0, 0); glVertex2i(100, -100);  // Top Left
	glEnd();
	
	glBindTexture(GL_TEXTURE_2D, imgTextures[PANORAMIC0]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(600, -400);  // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(900, -400);  // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(900, -100);  // Top Right
		glTexCoord2f(0, 0); glVertex2i(600, -100);  // Top Left
	glEnd();
	
	/*glBindTexture(GL_TEXTURE_2D, imgTextures[HIGH_RES]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(100, -800);  // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(700, -800);  // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(700, -420);  // Top Right
		glTexCoord2f(0, 0); glVertex2i(100, -420);  // Top Left
	glEnd();*/
	
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

void AnalysisGUI::drawTextInfo() {
	char text[30];
	glPushMatrix();
	glTranslated(currWinW - 600, 200 - currWinH, 0);
	glColor3f(0, 0, 0);
	sprintf(text, "pH: %.2f", pH);
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	sprintf(text, "Hum: %.2f%%", humidity);
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 0, -30);
	sprintf(text, "Lat: %.2fdeg", latitude);
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 0, -60);
	sprintf(text, "Lon: %.2fdeg", longitude);
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 0, -90);
	sprintf(text, "Alt: %.2fm", altitude);
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 0, -120);
	glPopMatrix();
}

void AnalysisGUI::keydown(unsigned char key, int x, int y) {
	if (key == 27) {
		exit(0);
	} else if (key == '0') {
		if (saveState()) {
			printf("save successful\n");
		} else {
			printf("save unsuccessful\n");
		}
	} else if (key == '1') {
		requestPano = true;
		// turn on second channel
		// save images from both channels
		// turn off second channel
		// run stitcher (for now)
		// TODO: write separate subscribers for multi channel handling
		owr_messages::stream msg;
		msg.stream = 1;
		msg.on = true;
		streamPub.publish(msg);
		ros::spinOnce();
	}
}

void AnalysisGUI::keyup(unsigned char key, int x, int y) {

}

void AnalysisGUI::special_keydown(int keycode, int x, int y) {

}

void AnalysisGUI::special_keyup(int keycode, int x, int y) {

}

bool AnalysisGUI::saveState() {
	char *home = getenv("HOME");
	if (home == NULL) {
		printf("Unable to get home directory\n");
		return false;
	}
	//printf("home dir: %s\n", home);
	
	char foldername[100] = {0};
	sprintf(foldername, "%s/owr_sites", home);
	if (stat(foldername, &st) == -1) {
		printf("Unable to locate sites folder, creating folder 'home/owr_sites'.\n");
		mkdir(foldername, 0700);
	}
	
	char timestamp[30];
	time_t t = time(NULL);
	struct tm time = *localtime(&t);
	sprintf(timestamp, "%d_%d_%d-%d_%d_%d", time.tm_year + 1900, time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
	sprintf(foldername, "%s/owr_sites/%s", home, timestamp);
	
	if (stat(foldername, &st) == -1) {
		mkdir(foldername, 0700);
		
		unsigned char *data = (unsigned char *)malloc(640*480*3);
		
		char filename[100];
		sprintf(filename, "%s/%s-panoramic0.bmp", foldername, timestamp);
		glBindTexture(GL_TEXTURE_2D, imgTextures[PANORAMIC0]);
		glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, data);
		saveBMPFile(filename, data, 640, 480);
		
		sprintf(filename, "%s/%s-panoramic1.bmp", foldername, timestamp);
		glBindTexture(GL_TEXTURE_2D, imgTextures[PANORAMIC1]);
		glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, data);
		saveBMPFile(filename, data, 640, 480);
		
		sprintf(filename, "%s/%s-panoramic1.bmp", foldername, timestamp);
		glBindTexture(GL_TEXTURE_2D, imgTextures[HIGH_RES]);
		glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, data);
		saveBMPFile(filename, data, 640, 480);
		
		sprintf(filename, "%s/%s-analysis.txt", foldername, timestamp);
		FILE *f = fopen(filename, "w");
		char stat[30];
		sprintf(stat, "Latitude: %f\n", latitude);
		fwrite(stat, strlen(stat), 1, f);
		sprintf(stat, "Longitude: %f\n", longitude);
		fwrite(stat, strlen(stat), 1, f);
		sprintf(stat, "pH: %f\n", pH);
		fwrite(stat, strlen(stat), 1, f);
		
		sprintf(stat, "Humidity: %f\n", humidity);
		fwrite(stat, strlen(stat), 1, f);
		sprintf(stat, "Altitude: %f\n", altitude);
		fwrite(stat, strlen(stat), 1, f);
		fclose(f);
		
		return true;
	}
	return false;
}
