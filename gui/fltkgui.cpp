//-L/usr/local/lib -lfltk -lXext -lX11 -lm -lfltk -lXext -lX11 -lm -lfltk_forms -lfltk_gl -lfltk_images

#include <FL/Fl.H>
#include <Fl/gl.h>
#include <Fl/glu.h>
#include <FL/Fl_Window.H>
#include <FL/Fl_Box.H>
#include <Fl/Fl_Gl_Window.H>
#include <FL/Fl_Light_Button.H>

int main(int argc, char **argv) {
	Fl_Window *win1 = new Fl_Window(1000,700);
	Fl_Box *box = new Fl_Box(10,10,500,500);
	box->box(FL_UP_BOX);
		
	Fl_Light_Button *cam1 = new Fl_Light_Button(50, 50, 50, 50, "1");
	Fl_Light_Button *cam2 = new Fl_Light_Button(50, 125, 50, 50, "2");
	Fl_Light_Button *cam3 = new Fl_Light_Button(50, 200, 50, 50, "3");
	
	cam1->color(FL_GREEN);
	cam2->color(FL_YELLOW);
	cam3->color(FL_BLUE);
	
	win1->end();
	win1->show(argc, argv);
	
	return Fl::run();
}
