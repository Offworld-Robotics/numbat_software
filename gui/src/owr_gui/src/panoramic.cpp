#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <GL/glut.h>

using namespace std;

// default window size
#define WINDOW_W 1600
#define WINDOW_H 800

// OpenGL control related variables
static int currentWindowH = WINDOW_H;
static int currentWindowW = WINDOW_W;
static GLuint textureNames[2]; // 1 panoramic image, 1 hi-res image

static double longitude = 0;
static double latitude = 0;
static double pH = 0;
static double humidity = 0;
static double altitude = 0;
//static double accuracy = 0;

// OpenGL essential functions
void init();
void reshape(int w, int h);
void idle();
void display();

void keydown(unsigned char key, int x, int y);

// converts BMP color format to 
void BGR2RGB(unsigned char *data, int size) {
	unsigned char temp;
	for (int i = 0; i < size; i += 3) {
		temp = data[i];
		data[i] = data[i + 2];
		data[i + 2] = temp;
	}
}

void loadTextures() {
	int width, height, size;
	unsigned char dataStart;
	unsigned char *data;
	FILE *f;

	f = fopen("panoramic.bmp", "r");
	if (f == NULL) {
		printf("Could not open 'space-sunrise.bmp'!\n");
		exit(1);
	}
	fseek(f, 0x0A, SEEK_SET);
	fread(&dataStart, sizeof(unsigned char), 1, f); // get the location of starting byte of pixel data
	fseek(f, 0x12, SEEK_SET);
	fread(&width, sizeof(int), 1, f); // get the width of the image
	fseek(f, 0x16, SEEK_SET);
	fread(&height, sizeof(int), 1, f); // get the height of the image
	size = width*height * 3;

	data = (unsigned char *)malloc(size*sizeof(unsigned char));
	fseek(f, dataStart, SEEK_SET);
	fread(data, size, 1, f);
	fclose(f);

	BGR2RGB(data, size);
	glGenTextures(1, &textureNames[0]);
	glBindTexture(GL_TEXTURE_2D, textureNames[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	free(data);

	f = fopen("hires.bmp", "r");
	if (f == NULL) {
		printf("Could not open 'space-sunrise.bmp'!\n");
		exit(1);
	}
	fseek(f, 0x0A, SEEK_SET);
	fread(&dataStart, sizeof(unsigned char), 1, f); // get the location of starting byte of pixel data
	fseek(f, 0x12, SEEK_SET);
	fread(&width, sizeof(int), 1, f); // get the width of the image
	fseek(f, 0x16, SEEK_SET);
	fread(&height, sizeof(int), 1, f); // get the height of the image
	size = width*height * 3;

	data = (unsigned char *)malloc(size*sizeof(unsigned char));
	fseek(f, dataStart, SEEK_SET);
	fread(data, size, 1, f);
	fclose(f);

	BGR2RGB(data, size);
	glGenTextures(1, &textureNames[1]);
	glBindTexture(GL_TEXTURE_2D, textureNames[1]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	free(data);
}

void drawText(char *text, int x, int y) {
	for (unsigned int i = 0; i < strlen(text); i++) {
		glRasterPos3f(x, y, 0);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
		x += 10;
	}
}

int main(int argc, char **argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_W, WINDOW_H);
	glutInitWindowPosition(100, 50);
	glutCreateWindow("OWR Image Manager");
	loadTextures();
	init();
	glutKeyboardFunc(keydown);
	//glutSpecialFunc(special_keydown);
	//glutSpecialUpFunc(special_keyup);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);
	glutMainLoop();
	return 0;
}

void idle() {
	display();
	usleep(16666);
}

void drawButtons() {
	glPushMatrix();
	glColor3f(0.7,0.7,0.7);
	glRecti(10,-10,160,-90);
	glRecti(170,-10,320,-90);
	glRecti(330,-10, 480, -90);
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
		glTexCoord2f(1, 0); glVertex2i(1200, -400);  // Bottom Right
		glTexCoord2f(1, 1); glVertex2i(1200, -100);  // Top Right
		glTexCoord2f(0, 1); glVertex2i(100, -100);  // Top Left
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();

	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glColor3f(1, 1, 1);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
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
	glTranslated(currentWindowW - 200, 200-currentWindowH, 0);
	glColor3f(0,0,0);
	sprintf(text, "pH: %.2f", pH);
	drawText(text, 0, 0);
	sprintf(text, "Hum: %.2f%%", humidity);
	drawText(text, 0, -20);
	sprintf(text, "Lat: %.2fdeg", latitude);
	drawText(text, 0, -40);
	sprintf(text, "Lon: %.2fdeg", longitude);
	drawText(text, 0, -60);
	sprintf(text, "Alt: %.2fm", altitude);
	drawText(text, 0, -80);
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

void keydown(unsigned char key, int x, int y) {
	switch (key) {
	case 27:
		exit(0);
		break;
	}
}
