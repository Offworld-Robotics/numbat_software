/*
 * Video Feed Frame
 *
 * Frame to display a video feed in an OpenGL GUI Screen
 *
 * for use with
 *
 */

#ifndef VIDEO_FEED_FRAME

#define VIDEO_FEED_FRAME



#include <GL/freeglut.h>

class Video_Feed_Frame {

   public:
      Video_Feed_Frame();
      ~Video_Feed_Frame();
      void displayFrame();


   private:
      GLuint videoTexture;
      int centreX, centreY; 
      int halfWidth, halfHeight;
};

#endif


