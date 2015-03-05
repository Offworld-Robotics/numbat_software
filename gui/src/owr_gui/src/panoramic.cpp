
#include "AnalysisGUI.h"
#include "SiteGui.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "GUI");
	glutInit(&argc, argv);
	SiteGui gui;
	SiteGui::createInstance(gui);
	gui.instance->init();
	return EXIT_SUCCESS;
}

SiteGui *SiteGui::instance = NULL;

void SiteGui::createInstance(SiteGui gui) {
	instance = &gui;
}

//glut wrapper functions because it doesn't like c++ :(
void SiteGui::glut_reshape(int w, int h) {
	instance->reshape(w,h);
}
void SiteGui::glut_idle() {
	instance->idle();
}
void SiteGui::glut_display() {
	instance->display();
}
void SiteGui::glut_keydown(unsigned char key, int x, int y) {
	instance->keydown(key, x, y);
}

SiteGui::SiteGui() {
	latitude = 0;
	longitude = 0;
	altitude = 0;
	pH = 0;
	humidity = 0;
	ultrasonic = 0;
	
	// OpenGL related variables
	currentWindowH = WINDOW_H;
	currentWindowW = WINDOW_W;
	frameCounter = 0;
	textures[0] = 0;
	textures[1] = 0;
	arrowKeys[0] = 0;
	arrowKeys[1] = 0;
	arrowKeys[2] = 0;
	arrowKeys[3] = 0;
}

void SiteGui::init(void) {
	ANALYSISGUI *analysisnode = new ANALYSISGUI(this);
	analysisGui = analysisnode;
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_W, WINDOW_H);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("SITE GUI");
	
	glGenTextures(2, textures);
	
	for(int i = 0;i < 2;i++) {
		glBindTexture(GL_TEXTURE_2D, textures[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}
	
	glClearColor(1, 1, 1, 0);
	glShadeModel(GL_FLAT);
	//loadTextures();
	//glEnable(GL_BLEND); // enables transparency in overlay items
	//glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glutKeyboardFunc(glut_keydown);
	//glutSpecialFunc(glut_special_keydown);
	//glutSpecialUpFunc(glut_special_keyup);
	glutDisplayFunc(glut_display);
	glutReshapeFunc(glut_reshape);
	glutIdleFunc(glut_idle);
	glutMainLoop();
}

void SiteGui::updateSiteConstants(double lat, double lon, float alt, float PH, float usonic, float humid, unsigned char *f) {
	latitude = lat;
	longitude = lon;
	altitude = alt;
	pH = PH;
	ultrasonic = usonic;
	humidity = humid;
	
	/*if (f != NULL) {
		printf("New frame received\n");
		glBindTexture(GL_TEXTURE_2D, textures[0]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, VIDEO_W, VIDEO_H, 0, GL_RGB, GL_UNSIGNED_BYTE, f);
		glBindTexture(GL_TEXTURE_2D, textures[1]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, VIDEO_W, VIDEO_H, 0, GL_RGB, GL_UNSIGNED_BYTE, f);
	}*/
	
	ROS_INFO("Updated Constants");
}

void SiteGui::display(void) {
	glClearColor(1, 1, 1, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	drawButtons();
	drawImages();
	drawTextInfo();
	
	glutSwapBuffers();
}

void SiteGui::drawText(char *text, int x, int y) {
	for (unsigned int i = 0; i < strlen(text); i++) {
		glRasterPos3f(x, y, 0);
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, text[i]);
		x += glutBitmapWidth(GLUT_BITMAP_TIMES_ROMAN_24, text[i]);
	}
}

void SiteGui::idle() {
	ros::spinOnce();
	display();
	usleep(16666);
}

void SiteGui::drawButtons() {
	char text[20];
	glPushMatrix();
		glColor3f(0.7,0.7,0.7);
		glRecti(10,-10,160,-90);
		glRecti(170,-10,320,-90);
		glRecti(330,-10, 480, -90);
		glColor3f(1,1,0);
		sprintf(text, "SAVE");
		drawText(text, 50, -60);
		sprintf(text, "LOAD");
		drawText(text, 210, -60);
	glPopMatrix();
}

void SiteGui::drawImages() {
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	
	glColor3f(1, 1, 1);
	
	// data from frame array is flipped, texcoords were changed to compensate
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(100, -400);  // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(1600, -400);  // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(1600, -100);  // Top Right
		glTexCoord2f(0, 0); glVertex2i(100, -100);  // Top Left
	glEnd();
	
	glBindTexture(GL_TEXTURE_2D, textures[1]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(100, -800);  // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(700, -800);  // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(700, -420);  // Top Right
		glTexCoord2f(0, 0); glVertex2i(100, -420);  // Top Left
	glEnd();
	
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

void SiteGui::drawTextInfo() {
	char text[30];
	glPushMatrix();
		glTranslated(currentWindowW - 600, 200 - currentWindowH, 0);
		glColor3f(0, 0, 0);
		sprintf(text, "pH: %.2f", pH);
		drawText(text, 0, 0);
		sprintf(text, "Hum: %.2f%%", humidity);
		drawText(text, 0, -30);
		sprintf(text, "Lat: %.2fdeg", latitude);
		drawText(text, 0, -60);
		sprintf(text, "Lon: %.2fdeg", longitude);
		drawText(text, 0, -90);
		sprintf(text, "Alt: %.2fm", altitude);
		drawText(text, 0, -120);
	glPopMatrix();
}

void SiteGui::reshape(int w, int h) {
	currentWindowH = h;
	currentWindowW = w;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, (GLdouble)w, -(GLdouble)h, 0); // (0,0) is top left corner of window, use cartesian co-ordinates
	glMatrixMode(GL_MODELVIEW);
}

void SiteGui::fillBMPHeader(unsigned char *data, int width, int height) {
	memset(data, 0, 36);
	int datasize = width*height*3;
	data[0x00] = 'B';
	data[0x01] = 'M';
	*(int *)&data[0x02] = datasize+0x36;
	*(int *)&data[0x0A] = 0x36;
	data[0x0E] = 0x28;
	*(int *)&data[0x12] = width;
	*(int *)&data[0x16] = height;
	data[0x1A] = 0x01;
	data[0x1C] = 0x18;
	*(int *)&data[0x22] = datasize;
	data[0x26] = 0x25;
	data[0x27] = 0x16;
	data[0x2A] = 0x25;
	data[0x2B] = 0x16;
}

bool SiteGui::saveState() {
	char *home = getenv("HOME");
	if (home == NULL) {
		printf("unable to get home dir\n");
		return false;
	}
	printf("home dir: %s\n", home);
	
	char foldername[100] = {0};
	sprintf(foldername, "%s/owr_sites", home);
	if (stat(foldername, &st) == -1) {
		printf("Unable to locate sites folder, creating folder.\n");
		mkdir(foldername, 0700);
	}
	
	char timestamp[30] = {0};
	time_t t = time(NULL);
	struct tm time = *localtime(&t);
	sprintf(timestamp, "%d_%d_%d-%d_%d_%d", time.tm_year + 1900, time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
	sprintf(foldername, "%s/owr_sites/%s", home, timestamp);
	
	if (stat(foldername, &st) == -1) {
		mkdir(foldername, 0700);
		char filename[100] = {0};
		FILE *f;
		unsigned char *bmp;
		
		sprintf(filename, "%s/%s-panoramic.bmp", foldername, timestamp);
		f = fopen(filename, "w");
		if (f == NULL) return false;
		bmp = (unsigned char *)malloc((PANO_DATA_SIZE + 0x36)*sizeof(unsigned char));
		if (bmp == NULL) return false;
		fillBMPHeader(bmp, PANO_W, PANO_H);
		glBindTexture(GL_TEXTURE_2D, textures[0]);
		glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, &bmp[0x36]);
		fwrite(bmp, PANO_DATA_SIZE + 0x36, 1, f);
		fclose(f);
		free(bmp);
		
		sprintf(filename, "%s/%s-hires.bmp", foldername, timestamp);
		f = fopen(filename, "w");
		if (f == NULL) return false;
		bmp = (unsigned char *)malloc((HIRES_DATA_SIZE + 0x36)*sizeof(unsigned char));
		if (bmp == NULL) return false;
		fillBMPHeader(bmp, HIRES_W, HIRES_H);
		glBindTexture(GL_TEXTURE_2D, textures[1]);
		glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, &bmp[0x36]);
		fwrite(bmp, HIRES_W*HIRES_H*3 + 0x36, 1, f);
		fclose(f);
		free(bmp);
		
		sprintf(filename, "%s/%s-analysis.txt", foldername, timestamp);
		f = fopen(filename, "w");
		char stat[30] = {0};
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

void SiteGui::keydown(unsigned char key, int x, int y) {
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

void SiteGui::loadTextures() {
	int width, height, size;
	unsigned char dataStart;
	unsigned char *data;
	FILE *pano, *hires;
	
	char *home = getenv("HOME");
	
	char filename[50] = {0};
	sprintf(filename, "%s/panoramic.bmp", home);
	pano = fopen(filename, "r");
	sprintf(filename, "%s/hires.bmp", home);
	hires = fopen(filename, "r");
	if (pano == NULL) {
		printf("Could not open 'panoramic.bmp'!\n");
	} else if (hires == NULL) {
		printf("Could not open 'hires.bmp'!\n");
	} else {
		fseek(pano, 0x0A, SEEK_SET);
		fread(&dataStart, sizeof(unsigned char), 1, pano); // get the location of starting byte of pixel data
		fseek(pano, 0x12, SEEK_SET);
		fread(&width, sizeof(int), 1, pano); // get the width of the image
		fseek(pano, 0x16, SEEK_SET);
		fread(&height, sizeof(int), 1, pano); // get the height of the image
		size = width*height * 3;

		data = (unsigned char *)malloc(size*sizeof(unsigned char));
		fseek(pano, dataStart, SEEK_SET);
		fread(data, size, 1, pano);
		fclose(pano);

		glBindTexture(GL_TEXTURE_2D, textures[0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, data);
		free(data);

		fseek(hires, 0x0A, SEEK_SET);
		fread(&dataStart, sizeof(unsigned char), 1, hires); // get the location of starting byte of pixel data
		fseek(hires, 0x12, SEEK_SET);
		fread(&width, sizeof(int), 1, hires); // get the width of the image
		fseek(hires, 0x16, SEEK_SET);
		fread(&height, sizeof(int), 1, hires); // get the height of the image
		size = width*height * 3;

		data = (unsigned char *)malloc(size*sizeof(unsigned char));
		fseek(hires, dataStart, SEEK_SET);
		fread(data, size, 1, hires);
		fclose(hires);

		glBindTexture(GL_TEXTURE_2D, textures[1]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, data);
		free(data);
		printf("Placeholder textures loaded\n");
	}
}
