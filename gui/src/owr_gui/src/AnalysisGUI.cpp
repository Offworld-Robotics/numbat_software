#include "GLUTWindow.h"
#include "AnalysisGUI.h"
#include "AnalysisNode.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "AnalysisGUI");
	AnalysisGUI gui(&argc, argv);
	gui.run();
	return EXIT_SUCCESS;
}

AnalysisGUI::AnalysisGUI(int *argc, char **argv) : GLUTWindow() {
	analysisNode = new AnalysisNode(this);
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_W, WINDOW_H);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Analysis");
	
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
	glutDisplayFunc(glut_display);
	glutReshapeFunc(glut_reshape);
	glutIdleFunc(glut_idle);
	
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
}

void AnalysisGUI::run(void) {
	glutMainLoop();
}

void AnalysisGUI::updateSiteConstants(double lat, double lon, float alt, float PH, float usonic, float humid, unsigned char *f) {
	latitude = lat;
	longitude = lon;
	altitude = alt;
	pH = PH;
	ultrasonic = usonic;
	humidity = humid;
	
	if (f != NULL) {
		glBindTexture(GL_TEXTURE_2D, imgTextures[PANORAMIC]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, VIDEO_W, VIDEO_H, 0, GL_RGB, GL_UNSIGNED_BYTE, f);
		glBindTexture(GL_TEXTURE_2D, imgTextures[HIGH_RES]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, VIDEO_W, VIDEO_H, 0, GL_RGB, GL_UNSIGNED_BYTE, f);
	}
	
	//ROS_INFO("Updated Constants");
}

void AnalysisGUI::display(void) {
	glClearColor(1, 1, 1, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	drawButtons();
	drawImages();
	drawTextInfo();
	
	glutSwapBuffers();
}

void AnalysisGUI::idle() {
	ros::spinOnce();
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
	sprintf(text, "SAVE");
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 50, -60);
	sprintf(text, "LOAD");
	drawText(text, GLUT_BITMAP_TIMES_ROMAN_24, 210, -60);
	glPopMatrix();
}

void AnalysisGUI::drawImages() {
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	
	glColor3f(1, 1, 1);
	
	// data from frame array is flipped, texcoords were changed to compensate
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	
	glBindTexture(GL_TEXTURE_2D, imgTextures[PANORAMIC]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(100, -400);  // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(1600, -400);  // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(1600, -100);  // Top Right
		glTexCoord2f(0, 0); glVertex2i(100, -100);  // Top Left
	glEnd();
	
	glBindTexture(GL_TEXTURE_2D, imgTextures[HIGH_RES]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(100, -800);  // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(700, -800);  // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(700, -420);  // Top Right
		glTexCoord2f(0, 0); glVertex2i(100, -420);  // Top Left
	glEnd();
	
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
		char filename[100];
		FILE *f;
		unsigned char *bmp;
		
		sprintf(filename, "%s/%s-panoramic.bmp", foldername, timestamp);
		f = fopen(filename, "w");
		if (f == NULL) return false;
		bmp = (unsigned char *)malloc((PANO_DATA_SIZE + BMP_HEADER_SIZE)*sizeof(unsigned char));
		if (bmp == NULL) return false;
		fillBMPHeader(bmp, PANO_W, PANO_H);
		glBindTexture(GL_TEXTURE_2D, imgTextures[PANORAMIC]);
		glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, &bmp[BMP_HEADER_SIZE]);
		fwrite(bmp, PANO_DATA_SIZE + BMP_HEADER_SIZE, 1, f);
		fclose(f);
		free(bmp);
		
		sprintf(filename, "%s/%s-hires.bmp", foldername, timestamp);
		f = fopen(filename, "w");
		if (f == NULL) return false;
		bmp = (unsigned char *)malloc((HIRES_DATA_SIZE + BMP_HEADER_SIZE)*sizeof(unsigned char));
		if (bmp == NULL) return false;
		fillBMPHeader(bmp, HIRES_W, HIRES_H);
		glBindTexture(GL_TEXTURE_2D, imgTextures[HIGH_RES]);
		glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, &bmp[BMP_HEADER_SIZE]);
		fwrite(bmp, HIRES_DATA_SIZE + BMP_HEADER_SIZE, 1, f);
		fclose(f);
		free(bmp);
		
		sprintf(filename, "%s/%s-analysis.txt", foldername, timestamp);
		f = fopen(filename, "w");
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

void AnalysisGUI::keydown(unsigned char key, int x, int y) {
	switch (key) {
	case 27:
		exit(0);
		break;
	case '1':
		if (saveState()) {
			printf("save successful\n");
		} else {
			printf("save unsuccessful\n");
		}
		break;
	}
}

void AnalysisGUI::keyup(unsigned char key, int x, int y) {

}

void AnalysisGUI::special_keydown(int keycode, int x, int y) {

}

void AnalysisGUI::special_keyup(int keycode, int x, int y) {

}
