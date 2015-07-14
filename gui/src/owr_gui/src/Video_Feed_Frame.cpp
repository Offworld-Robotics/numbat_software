
/* 
 * Function Implementations for Video_Feed_Frame
 */

#include <Video_Feed_Frame.hpp>
#include <GL/freeglut.h>

// Not providing Zero argument constructor, therefore instances must be initialised in a 
//	 constructor initialisor list in any classes that use this
Video_Feed_Frame::Video_Feed_Frame(int winW, int winH, double _cenXRatio, double _cenYRatio, double _wRatio, double _hRatio) {
	cenXRatio = _cenXRatio;
	cenYRatio = _cenYRatio;
	wRatio = _wRatio;
	hRatio = _hRatio;
	setNewWindowSize(winW, winH);
	
	// Setup the texture to store frames of the video feed into
	glGenTextures(1, &videoTexture);
	glBindTexture(GL_TEXTURE_2D, videoTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

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
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, 64, 64, 0, GL_RGB, GL_UNSIGNED_BYTE, checkImage);
}

// TODO
// finish implementing Constructor
Video_Feed_Frame::~Video_Feed_Frame(){
	/* Replace with actual cleanup
	 */
}

void Video_Feed_Frame::setNewWindowSize(int winW, int winH) {
	centreX = winW*cenXRatio;
	centreY = winH*cenYRatio;
	halfWidth = winW*wRatio/2.0;
	halfHeight = winH*hRatio/2.0;
}

// CHECK IF REQUIRES MODIFICATION FOR COMPATIBILITY
// called by ROS process to set the contents of the texture to be the next frame of the video stream
void Video_Feed_Frame::setNewStreamFrame(unsigned char *frame, int width, int height) {
	if (frame != NULL) {
		glBindTexture(GL_TEXTURE_2D, videoTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, frame);
	}
	
	//ROS_INFO("Updated video");
}

// actually draw the Video_Feed_Frame to screen
void Video_Feed_Frame::draw() {
	glPushMatrix();
	glLoadIdentity();
	glTranslated(centreX, centreY, 0);
	glEnable(GL_TEXTURE_2D);
	glColor3f(1, 1, 1);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, videoTexture);
	
	// data from frame array is flipped, texcoords were changed to compensate
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2d(-halfWidth, -halfHeight); // Bottom Left
		glTexCoord2f(1, 1); glVertex2d(halfWidth, -halfHeight); // Bottom Right
		glTexCoord2f(1, 0); glVertex2d(halfWidth, halfHeight); // Top Right
		glTexCoord2f(0, 0); glVertex2d(-halfWidth, halfHeight); // Top Left
	glEnd();
	
	// must disable texturing when done or graphical glitches occur
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

