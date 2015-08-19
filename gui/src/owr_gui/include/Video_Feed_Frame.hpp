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

#define ZOOM_IN 1
#define ZOOM_OUT 0

class Video_Feed_Frame {

	public:
	Video_Feed_Frame(int winW, int winH, double _cenXRatio, double _cenYRatio, double _wRatio, double _hRatio);
	~Video_Feed_Frame();

	void draw();
	void setNewStreamFrame(unsigned char *frame, int width, int height);
	void setNewWindowSize(int winW, int winH);
	void zoom(bool dir);

	private:
	GLuint videoTexture;
	double cenXRatio, cenYRatio;
	double wRatio, hRatio;
	double centreX, centreY;
	double halfWidth, halfHeight;
	
	int zoomLevel;
	double texCutoff;
};

#endif


