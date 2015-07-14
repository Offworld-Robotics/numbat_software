/*
 * Video Feed Frame
 *
 * Frame to display a video feed in an OpenGL GUI Screen
 *
 * for use with NavigationGUI and FineControlGUI
 *
 */

#ifndef VIDEO_FEED_FRAME
#define VIDEO_FEED_FRAME

#include <GL/freeglut.h>

class Video_Feed_Frame {

   public:
      Video_Feed_Frame(double _centreX, double _centreY, double width, double height);
      ~Video_Feed_Frame();

      void draw();
      void setNewStreamFrame(unsigned char *frame, int width, int height);

   private:
      GLuint videoTexture;
      double centreX, centreY; 
      double halfWidth, halfHeight;
};

#endif


