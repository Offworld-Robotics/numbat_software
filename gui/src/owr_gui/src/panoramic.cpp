#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <GL/glut.h>
#include "site.h"
#include "AnalysisGUI.h"

using namespace std;

// default window size
#define WINDOW_W 1800
#define WINDOW_H 850

#define PANO_W 640
#define PANO_H 480
#define PANO_DATA_SIZE PANO_W*PANO_H*3
#define HIRES_W 640
#define HIRES_H 480
#define HIRES_DATA_SIZE HIRES_W*HIRES_H*3

// OpenGL control related variables
static int currentWindowH = WINDOW_H;
static int currentWindowW = WINDOW_W;
static GLuint textureNames[2] = {0}; // 1 panoramic image, 1 hi-res image

static double longitude = 0;
static double latitude = 0;
static float pH = 0;
static float humidity = 0;
static float altitude = 0;
static float ultrasonic = 0;
static unsigned char *frame = NULL;

struct stat st = {0};

// OpenGL essential functions
void init();
void reshape(int w, int h);
void idle();
void display();

void keydown(unsigned char key, int x, int y);

// converts BMP color format (BGR) to RGB and vice versa
void BGR2RGB(unsigned char *data, int size) {
	unsigned char temp;
	for (int i = 0; i < size; i += 3) {
		temp = data[i];
		data[i] = data[i + 2];
		data[i + 2] = temp;
	}
}

void updateSiteConstants(double lat, double lon, float alt, float PH, float usonic, float humid, unsigned char *f) {
	latitude = lat;
	longitude = lon;
	altitude = alt;
	pH = PH;
	ultrasonic = usonic;
	humidity = humid;
	
	if (f != NULL) {
		memcpy(frame, f, PANO_DATA_SIZE);
		glBindTexture(GL_TEXTURE_2D, textureNames[0]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, PANO_W, PANO_H, 0, GL_RGB, GL_UNSIGNED_BYTE, frame);
		glBindTexture(GL_TEXTURE_2D, textureNames[1]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, PANO_W, PANO_H, 0, GL_RGB, GL_UNSIGNED_BYTE, frame);
	}
}

void drawText(char *text, int x, int y) {
	for (unsigned int i = 0; i < strlen(text); i++) {
		glRasterPos3f(x, y, 0);
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, text[i]);
		x += glutBitmapWidth(GLUT_BITMAP_TIMES_ROMAN_24, text[i]);
	}
}

void loadTextures() {
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

		glBindTexture(GL_TEXTURE_2D, textureNames[0]);
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

		glBindTexture(GL_TEXTURE_2D, textureNames[1]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, data);
		free(data);
	}
}

int main(int argc, char **argv) {
	frame = (unsigned char *)calloc(PANO_DATA_SIZE, 1);
	ros::init(argc, argv, "ANALYSISGUI");
	ANALYSISGUI *analysis = new ANALYSISGUI();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_W, WINDOW_H);
	glutInitWindowPosition(50, 20);
	glutCreateWindow("OWR Image Manager");
	init();
	//loadTextures();
	glutKeyboardFunc(keydown);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);
	glutMainLoop();
	return 0;
}

void idle() {
	ros::spinOnce();
	display();
	usleep(16666);
}

void drawButtons() {
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

void drawImages() {
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glColor3f(1, 1, 1);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, textureNames[0]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2i(100, -400);  // Bottom Left
		glTexCoord2f(1, 0); glVertex2i(1600, -400);  // Bottom Right
		glTexCoord2f(1, 1); glVertex2i(1600, -100);  // Top Right
		glTexCoord2f(0, 1); glVertex2i(100, -100);  // Top Left
	glEnd();
	glBindTexture(GL_TEXTURE_2D, textureNames[1]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2i(100, -800);  // Bottom Left
		glTexCoord2f(1, 0); glVertex2i(700, -800);  // Bottom Right
		glTexCoord2f(1, 1); glVertex2i(700, -420);  // Top Right
		glTexCoord2f(0, 1); glVertex2i(100, -420);  // Top Left
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

void drawTextInfo() {
	char text[30];
	glPushMatrix();
	glTranslated(currentWindowW - 600, 200-currentWindowH, 0);
	glColor3f(0,0,0);
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

void display(void) {
	glClearColor(1,1,1,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	drawButtons();
	drawImages();
	drawTextInfo();
	
	glutSwapBuffers();
}

void init(void) {
	glGenTextures(2, textureNames);
	glBindTexture(GL_TEXTURE_2D, textureNames[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, textureNames[1]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_FLAT);
}

void reshape(int w, int h) {
	currentWindowH = h;
	currentWindowW = w;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, (GLdouble)w, -(GLdouble)h, 0); // (0,0) is top left corner of window, use cartesian co-ordinates
	glMatrixMode(GL_MODELVIEW);
}

void fillBMPHeader(unsigned char *data, int width, int height) {
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

bool saveState() {
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
		glBindTexture(GL_TEXTURE_2D, textureNames[0]);
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
		glBindTexture(GL_TEXTURE_2D, textureNames[1]);
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

void keydown(unsigned char key, int x, int y) {
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
