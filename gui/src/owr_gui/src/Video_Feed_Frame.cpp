
/* 
 * Function Implementations for Video_Feed_Frame
 */

#include <Video_Feed_Frame.hpp>
#include <GL/freeglut.h>

// Not providing Zero argument constructor, therefore instances must be initialised in a 
//    constructor initialisor list in any classes that use this
Video_Feed_Frame::Video_Feed_Frame(int _centreX, int _centreY, int width, int height){
   centreX = _centreX;
   centreY = _centreY;
   halfWidth = width/2;
   halfHeight = height/2;
	
   // Setup the texture to store frames of the video feed into
   glGenTextures(1, &videoTexture);
   glBindTexture(GL_TEXTURE_2D, videoTexture);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}

/* CHECK IF REQUIRES MODIFICATION FOR COMPATIBILITY
// called by ROS process to set the contents of the texture to be the next frame of the video stream
video_Feed_Frame::setNewStreamFrame(unsigned char *frame, int width, int height) {
	if (frame != NULL) {
		glBindTexture(GL_TEXTURE_2D, videoTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, frame);
	}
	
	//ROS_INFO("Updated video");
}
*/

// actually draw the Video_Feed_Frame to screen
void Video_Feed_Frame::displayFrame() {
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glColor3f(1, 1, 1);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, videoTexture);

	// data from frame array is flipped, texcoords were changed to compensate
	glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex2i(centreX-halfWidth, centreY-halfHeight); // Bottom Left
		glTexCoord2f(1, 1); glVertex2i(centreX+halfWidth, centreY-halfHeight); // Bottom Right
		glTexCoord2f(1, 0); glVertex2i(centreX+halfWidth, centreY+halfHeight); // Top Right
		glTexCoord2f(0, 0); glVertex2i(centreX-halfWidth, centreY+halfHeight); // Top Left
	glEnd();
        
        // must disable texturing when done or graphical glitches occur
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

#endif
