#include "GLUTWindow.h"
#include "Button.h"
#include "Video_Feed_Frame.hpp"
#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <vector>
#include "FineControlGUI.h"

void FineControlGUI::idle() {
	display();
	usleep(15000);
}

void FineControlGUI::display() {
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(0,0,0);
	
	glPushMatrix();
	glTranslated(currWinW/2, -currWinH/2, 0);
	glBegin(GL_LINES);
	glVertex2d(0, currWinH);
	glVertex2d(0, -currWinH/4);
	glVertex2d(-currWinW, -currWinH/4);
	glVertex2d(currWinW, -currWinH/4);
	glEnd();
	glPopMatrix();
	
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glColor3f(1, 1, 1);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, vidTex[0]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(0, -3*currWinH/4); // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(currWinW/2, -3*currWinH/4); // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(currWinW/2, 0); // Top Right
		glTexCoord2f(0, 0); glVertex2i(0, 0); // Top Left
	glEnd();
	glTranslated(currWinW/2, 0, 0);
	glBindTexture(GL_TEXTURE_2D, vidTex[1]);
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(0, -3*currWinH/4); // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(currWinW/2, -3*currWinH/4); // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(currWinW/2, 0); // Top Right
		glTexCoord2f(0, 0); glVertex2i(0, 0); // Top Left
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
	
	for(std::vector<Button*>::iterator i = buttons.begin();i != buttons.end();++i) {
		(*i)->draw();
	}
	glPushMatrix();
	glTranslated(currWinW/2, -7*currWinH/8, 0);
	glColor3f(0,0,0);
	char txt[] = "GUI info placeholder";
	drawText(txt, GLUT_BITMAP_TIMES_ROMAN_24, 0, 0);
	glPopMatrix();

        //Draw Video Feeds to Screen
        for(std::vector<Video_Feed_Frame*>::iterator feed=videoFeeds.begin(); feed != videoFeeds.end(); ++feed) {
		(*feed)->draw();
	}

	glutSwapBuffers();
}

void FineControlGUI::keydown(unsigned char key, int x, int y) {
	switch (key) {
		case 27:
			exit(0);
			break;
	}
}

void FineControlGUI::mouse(int button, int state, int x, int y) {
	if (state == GLUT_UP) {
		for(std::vector<Button*>::iterator i = buttons.begin();i != buttons.end();++i) {
			(*i)->unclick();
		}
	} else {
		for(std::vector<Button*>::iterator i = buttons.begin();i != buttons.end();++i) {
			if ((*i)->isPointInBounds(x, -y))
				(*i)->click();
		}
	}
}

FineControlGUI::FineControlGUI(int width, int height, int *argc, char *argv[]) : GLUTWindow(width, height, argc, argv, "Fine Control") {
	glClearColor(1, 1, 1, 1);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	glutMouseFunc(glut_mouse);
	
	glGenTextures(2, vidTex);

        videoFeeds.push_back(new Video_Feed_Frame(width*3/4, height*3/4, width/2, height/2) );


	
	for(int i = 0;i < 2;i++) {
		glBindTexture(GL_TEXTURE_2D, vidTex[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}
	
	// PLACEHOLDER TEXTURE GENERATION
	GLubyte checkImage[64][64][3];

	for (int i = 0; i < 64; i++) {
		for (int j = 0; j < 64; j++) {
			int c = ((((i&0x8)==0)^((j&0x8))==0))*255;
			checkImage[i][j][0] = (GLubyte) 0;
			checkImage[i][j][1] = (GLubyte) c;
			checkImage[i][j][2] = (GLubyte) 0;
		}
	}
	

	glBindTexture(GL_TEXTURE_2D, vidTex[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, 64, 64, 0, GL_RGB, GL_UNSIGNED_BYTE, checkImage);

         // setup placeholder Texture 2 ie BLUE check
	for (int i = 0; i < 64; i++) {
		for (int j = 0; j < 64; j++) {
			int c = ((((i&0x8)==0)^((j&0x8))==0))*255;
			checkImage[i][j][0] = (GLubyte) 0;
			checkImage[i][j][1] = (GLubyte) 0;
			checkImage[i][j][2] = (GLubyte) c;
		}
	}
        // Bind placeholder Texture 2 Blue to VideoFeedFrame
        videoFeeds[0]->setNewStreamFrame((unsigned char*)checkImage, 64,64);

        /*
        // actually draw placeholder Texture 2 to screen
	glBindTexture(GL_TEXTURE_2D, vidTex[1]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, 64, 64, 0, GL_RGB, GL_UNSIGNED_BYTE, checkImage);
	
        */



        // setup Zoom Buttons
	for (int i = 0;i < 4;i++) {
		arrows[i] = false;
	}
	char txt[2] = {'+', '\0'};
	buttons.push_back(new Button(currWinW/2 - 100, -currWinH/2, 50, 50, 0, 0.5, 0.5, txt));
	txt[0] = '-';
	buttons.push_back(new Button(currWinW/2 + 100, -currWinH/2, 50, 50, 0, 0.5, 0.5, txt));
}

int main(int argc, char *argv[]) {
	FineControlGUI gui(1855, 1056, &argc, argv);
	gui.run();
	return 0;
}
